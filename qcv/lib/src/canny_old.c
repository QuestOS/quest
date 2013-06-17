/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "canny.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include <string.h>


#define GAUSSIAN_CUT_OFF (float)0.005f
#define MAGNITUDE_SCALE (float)100
#define MAGNITUDE_LIMIT (float)1000
#define MAGNITUDE_MAX (int)(MAGNITUDE_SCALE * MAGNITUDE_LIMIT)

typedef struct {
  int *data, *magnitude;
  int data_len, mag_len;
  float *xConv, *yConv, *xGradient, *yGradient;
  int xC_len, yC_len, xG_len, yG_len;
  size_t width, height;
  size_t picsize;
} qcv_canny_internal_params_t;

#define QCV_DEFAULT_CANNY_INTERNAL_PARAMS {                             \
    .data = NULL, .magnitude = NULL,                                    \
      .data_len = 0, .mag_len = 0,                                      \
      .xConv = NULL, .yConv = NULL, .xGradient = NULL, .yGradient = NULL, \
      .xC_len = 0, .yC_len = 0, .xG_len = 0, .yG_len = 0,               \
      .width = 0, .height = 0,                                          \
      .picsize = 0}


static int qcv_canny_init_arrays(qcv_frame_t* frame,
                                 qcv_canny_internal_params_t* internal_params);

static int qcv_canny_read_luminance(qcv_frame_t* frame,
                                    qcv_canny_internal_params_t* internal_params);

static void qcv_canny_normalize_contrast(qcv_frame_t* frame,
                                         qcv_canny_internal_params_t* internal_params);

static int qcv_canny_compute_gradients(qcv_frame_t* frame,
                                        qcv_canny_internal_params_t* internal_params,
                                        qcv_canny_params_t* params);

static void qcv_canny_perform_hysteresis(qcv_frame_t* frame,
                                         qcv_canny_internal_params_t* internal_params,
                                         int low, int high);

static void qcv_canny_write_edges(qcv_canny_internal_params_t* internal_params,
                                  qcv_frame_t* out_frame);

static void qcv_canny_free_arrays(qcv_canny_internal_params_t* internal_params);

int qcv_canny_frame(qcv_frame_t* frame, qcv_canny_params_t* params, qcv_frame_t* out_frame)
{
  qcv_canny_internal_params_t internal_params = QCV_DEFAULT_CANNY_INTERNAL_PARAMS;
  if(!frame || !params || !out_frame) return -1;

  int res = 0;
  if((res = qcv_canny_init_arrays(frame, &internal_params)) < 0) return res;
  if((res = qcv_canny_read_luminance(frame, &internal_params)) < 0) goto cleanup;
  if(params->contrast_normalized) {
    qcv_canny_normalize_contrast(frame, &internal_params);
  }
  if((res = qcv_canny_compute_gradients(frame, &internal_params, params)) < 0) goto cleanup;
  int low = lroundf(params->low_threshold * MAGNITUDE_SCALE);
  int high = lroundf(params->high_threshold * MAGNITUDE_SCALE);
  qcv_canny_perform_hysteresis(frame, &internal_params, low, high);
  if((res = qcv_create_frame(out_frame, qcv_frame_width(frame), qcv_frame_height(frame),
                             QCV_FRAME_TYPE_1BYTE_GREY)) < 0) goto cleanup;
  qcv_canny_write_edges(&internal_params, out_frame);
  
 cleanup:
  
  qcv_canny_free_arrays(&internal_params);
  
  return res;
}


static int qcv_canny_init_arrays(qcv_frame_t* frame,
                                 qcv_canny_internal_params_t* internal_params)
{
  internal_params->data = malloc(qcv_frame_buf_size(frame) * sizeof(int));
  if(!internal_params->data) goto cleanup_0;
  
  internal_params->magnitude = malloc(qcv_frame_buf_size(frame) * sizeof(int));
  if(!internal_params->magnitude) goto cleanup_1;
    
  internal_params->xConv = malloc(qcv_frame_buf_size(frame) * sizeof(float));
  if(!internal_params->xConv) goto cleanup_2;
  
  internal_params->yConv = malloc(qcv_frame_buf_size(frame) * sizeof(float));
  if(!internal_params->yConv) goto cleanup_3;
  
  internal_params->xGradient = malloc(qcv_frame_buf_size(frame) * sizeof(float));
  if(!internal_params->xGradient) goto cleanup_4;
  
  internal_params->yGradient = malloc(qcv_frame_buf_size(frame) * sizeof(float));
  if(!internal_params->yGradient) goto cleanup_5;

  internal_params->data_len = qcv_frame_buf_size(frame);
  internal_params->yG_len = qcv_frame_buf_size(frame);
  internal_params->mag_len = qcv_frame_buf_size(frame);
  internal_params->xG_len = qcv_frame_buf_size(frame);
  internal_params->yC_len = qcv_frame_buf_size(frame);
  internal_params->xC_len = qcv_frame_buf_size(frame);
  internal_params->width = qcv_frame_width(frame);
  internal_params->height = qcv_frame_height(frame);
  internal_params->picsize = qcv_frame_width(frame) * qcv_frame_height(frame);
  
  return 0;

 cleanup_5: free(internal_params->xGradient);
 cleanup_4: free(internal_params->yConv);
 cleanup_3: free(internal_params->xConv);
 cleanup_2: free(internal_params->magnitude);
 cleanup_1: free(internal_params->data);
 cleanup_0: return -1;
}


static int
luminance(float r, float g, float b)
{
  return lroundf((float)0.299 * r + (float)0.587 * g + (float)0.114 * b);
}

int qcv_canny_read_luminance(qcv_frame_t* frame,
                             qcv_canny_internal_params_t* internal_params)
{
  int r, g, b, i;
  unsigned char *pixels;
  int offset;
  switch(qcv_frame_type(frame)) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    
    pixels = (unsigned char *)qcv_frame_buf(frame);
    offset = 0;
    /* -- EM -- Assuming that there is no padding after a row,
          should fix later*/
    for(i = 0; i < qcv_frame_buf_size(frame); i++) {
      r = pixels[offset++] & 0xff;
      g = pixels[offset++] & 0xff;
      b = pixels[offset++] & 0xff; 
      internal_params->data[i] = luminance(r, g, b);
      //printf("rl:%d=%d(%d,%d,%d)\n", i, internal_params->data[i],r,g,b);
    }
    return 0;
  default:
    return -1;
  }
  
  
}


static void qcv_canny_normalize_contrast(qcv_frame_t* frame,
                                        qcv_canny_internal_params_t* internal_params)
{
  int histogram[256], remap[256];
  int i;
  for(i = 0; i < internal_params->data_len; i++)
    histogram[internal_params->data[i]]++;

  int sum = 0, j = 0, target, k;
  for(i = 0; i < 256; i++){
    sum += histogram[i];
    target = sum * 255 / qcv_frame_buf_size(frame);
    for(k = j + 1; k <= target; k++)
      remap[k] = i;
    j = target;
  }

  for(i = 0; i < internal_params->data_len; i++){
    internal_params->data[i] = remap[internal_params->data[i]];
  }
}

static float
gaussian(float x, float sigma)
{
  return (float)pow(M_E, - (x * x) / ((float)2 * sigma * sigma));
}

static int qcv_canny_compute_gradients(qcv_frame_t* frame,
                                        qcv_canny_internal_params_t* internal_params,
                                        qcv_canny_params_t* params)
{
  float kernelWidth = params->gaussian_kernel_width;
  float kernelRadius = params->gaussian_kernel_radius;
  float *kernel;
  float *diffKernel;
  int kwidth;
  float g1, g2, g3;

  kernel = (float *)malloc(kernelWidth * sizeof(float));
  if(!kernel) return -1;

  diffKernel = (float *)malloc(kernelWidth * sizeof(float));
  if(!diffKernel) {
    free(kernel);
    return -1;
  }
  
  for(kwidth = 0; kwidth < kernelWidth; kwidth++){
    g1 = gaussian(kwidth, kernelRadius);
    if(g1 <= GAUSSIAN_CUT_OFF && kwidth >= 2) break;
    g2 = gaussian(kwidth - 0.5, kernelRadius);
    g3 = gaussian(kwidth + 0.5, kernelRadius);
    kernel[kwidth] = (g1 + g2 + g3) / (float)3 / ((float)2 * M_PI * kernelRadius * kernelRadius);
    diffKernel[kwidth] = g3 - g2;

//printf("kwidth:%d ,kernel:%x ,diffKernel:%x\n", kwidth, *((int*)&kernel[kwidth]), *((int*)&diffKernel[kwidth]));
  }

  int initX = kwidth - 1;
  int maxX = qcv_frame_width(frame) - (kwidth - 1);
  int initY = qcv_frame_width(frame) * (kwidth - 1);
  int maxY = qcv_frame_width(frame) * (qcv_frame_height(frame) - (kwidth - 1));

  int x, y, index;
  float sumX, sumY;
  int xOffset, yOffset;
  for(x = initX; x < maxX; x++){
    for(y = initY; y < maxY; y += qcv_frame_width(frame)){
      index = x + y;
      sumX = internal_params->data[index] * kernel[0];
      sumY = sumX;
      xOffset = 1;
      yOffset = qcv_frame_width(frame);
      for( ; xOffset < kwidth; ){
        sumY += kernel[xOffset] * (internal_params->data[index - yOffset] + internal_params->data[index + yOffset]);
        sumX += kernel[xOffset] * (internal_params->data[index - xOffset] + internal_params->data[index + xOffset]);
        yOffset += qcv_frame_width(frame);
        xOffset++;

      }

      internal_params->yConv[index] = sumY;
      internal_params->xConv[index] = sumX;

    }
  }

  float sum;
  int i;
  for(x = initX; x < maxX; x++){
    for(y = initY; y < maxY; y += qcv_frame_width(frame)){
      sum = 0;
      index = x + y;
      for(i = 1; i < kwidth; i++)
        sum += diffKernel[i] * (internal_params->yConv[index - i] - internal_params->yConv[index + i]);

      internal_params->xGradient[index] = sum;
    }
  }

  for(x = kwidth; x < qcv_frame_width(frame) - kwidth; x++){
    for(y = initY; y < maxY; y += qcv_frame_width(frame)){
      sum = 0;
      index = x + y;
      yOffset = qcv_frame_width(frame);
      for(i = 1; i < kwidth; i++){
        sum += diffKernel[i] * (internal_params->xConv[index - yOffset] - internal_params->xConv[index + yOffset]);
        yOffset += qcv_frame_width(frame);
      }

      internal_params->yGradient[index] = sum;
    }
  }

  initX = kwidth;
  maxX = qcv_frame_width(frame) - kwidth;
  initY = qcv_frame_width(frame) * kwidth;
  maxY = qcv_frame_width(frame) * (qcv_frame_height(frame) - kwidth);
  int indexN, indexS, indexW, indexE, indexNW, indexNE, indexSW, indexSE;
  float xGrad, yGrad, gradMag;
  float nMag, sMag, wMag, eMag, neMag, seMag, swMag, nwMag, tmp;
  for(x = initX; x < maxX; x++){
    for(y = initY; y < maxY; y += qcv_frame_width(frame)){
      index = x + y;
      indexN = index - qcv_frame_width(frame);
      indexS = index + qcv_frame_width(frame);
      indexW = index - 1;
      indexE = index + 1;
      indexNW = indexN - 1;
      indexNE = indexN + 1;
      indexSW = indexS - 1;
      indexSE = indexS + 1;

      xGrad = internal_params->xGradient[index];
      yGrad = internal_params->yGradient[index];
      gradMag = hypot(xGrad, yGrad);

      nMag = hypot(internal_params->xGradient[indexN], internal_params->yGradient[indexN]);
      sMag = hypot(internal_params->xGradient[indexS], internal_params->yGradient[indexS]);
      wMag = hypot(internal_params->xGradient[indexW], internal_params->yGradient[indexW]);
      eMag = hypot(internal_params->xGradient[indexE], internal_params->yGradient[indexE]);
      neMag = hypot(internal_params->xGradient[indexNE], internal_params->yGradient[indexNE]);
      seMag = hypot(internal_params->xGradient[indexSE], internal_params->yGradient[indexSE]);
      swMag = hypot(internal_params->xGradient[indexSW], internal_params->yGradient[indexSW]);
      nwMag = hypot(internal_params->xGradient[indexNW], internal_params->yGradient[indexNW]);

      if(xGrad * yGrad <= (float)0 
        ? fabsf(xGrad) >= fabsf(yGrad)
          ? (tmp = fabsf(xGrad * gradMag)) >= fabsf(yGrad * neMag - (xGrad + yGrad) * eMag)
            && tmp > fabsf(yGrad * swMag - (xGrad + yGrad) * wMag)
          : (tmp = fabsf(yGrad * gradMag)) >= fabsf(xGrad * neMag - (yGrad + xGrad) * nMag)
            && tmp > fabsf(xGrad * swMag - (yGrad + xGrad) * sMag)
        : fabsf(xGrad) >= fabsf(yGrad)
          ? (tmp = fabsf(xGrad * gradMag)) >= fabsf(yGrad * seMag + (xGrad - yGrad) * eMag)
            && tmp > fabsf(yGrad * nwMag + (xGrad - yGrad) * wMag)
          : (tmp = fabsf(yGrad * gradMag)) >= fabsf(xGrad * seMag + (yGrad - xGrad) * sMag)
            && tmp > fabsf(xGrad * nwMag + (yGrad - xGrad) * nMag)
        ){
        if(gradMag >= MAGNITUDE_LIMIT){
          internal_params->magnitude[index] = MAGNITUDE_MAX;
        }
        else{
          internal_params->magnitude[index] = (int)(MAGNITUDE_SCALE * gradMag);
        } 
      }
      else{
        internal_params->magnitude[index] = 0;
      }
    }
  }

  free(kernel);
  free(diffKernel);
  return 0;
}

static void canny_follow(qcv_frame_t* frame,
                         qcv_canny_internal_params_t* internal_params,
                         int x1, int y1, int i1, int threshold)
{
  int x0 = x1 == 0 ? x1 : x1 - 1;
  int x2 = x1 == qcv_frame_width(frame) - 1 ? x1 : x1 + 1;
  int y0 = y1 == 0 ? y1 : y1 - 1;
  int y2 = y1 == qcv_frame_height(frame) - 1 ? y1 : y1 + 1;
  
  internal_params->data[i1] = internal_params->magnitude[i1];
  //printf("cf:%d=%d\n", i1, internal_params->data[i1]);
  int x, y, i2;
  for(x = x0; x <= x2; x++){
    for(y = y0; y <= y2; y++){
      i2 = x + y * qcv_frame_width(frame);
      if((y != y1 || x != x1) && internal_params->data[i2] == 0
        && internal_params->magnitude[i2] >= threshold){
        canny_follow(frame, internal_params, x, y, i2, threshold);
        return;
      }
    }
  }
}

static void qcv_canny_perform_hysteresis(qcv_frame_t* frame,
                                         qcv_canny_internal_params_t* internal_params,
                                         int low, int high)
{
  memset(internal_params->data, 0, internal_params->data_len * sizeof(int));
  //printf("internal_params->data_len = %d\n", internal_params->data_len);
  //printf("high = %d\n", high);
  int offset = 0, y, x;
  for(y = 0; y < qcv_frame_height(frame); y++){
    for(x = 0; x < qcv_frame_width(frame); x++){
      //printf("ph: internal_params->data[%d] = %d\n    internal_params->magnitude[%d] = %d\n",
      //       offset, internal_params->data[offset], offset, internal_params->magnitude[offset]);
      if(internal_params->data[offset] == 0 && internal_params->magnitude[offset] >= high) {
        
        canny_follow(frame, internal_params, x, y, offset, low);
      }
      offset++;
    }
  }
}

static void qcv_canny_write_edges(qcv_canny_internal_params_t* internal_params,
                                  qcv_frame_t* out_frame)
{
  int i;
  for(i = 0; i < internal_params->picsize; i++) {
    qcv_frame_buf(out_frame)[i] = internal_params->data[i] > 0 ? 255 : 0;
    //printf("we:%d=%d\n", i, internal_params->data[i]);
  }
}


static void qcv_canny_free_arrays(qcv_canny_internal_params_t* internal_params)
{
  free(internal_params->data);
  free(internal_params->magnitude);
  free(internal_params->xConv);
  free(internal_params->yConv);
  free(internal_params->xGradient);
  free(internal_params->yGradient);
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
