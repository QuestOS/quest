#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "canny.h"

/* 
 * Quest doesn't provide much user space memory. 
 * In order to reduce memory usage, some buffers are shared 
 */
extern unsigned char buf[];
//int a_buf[WIDTH * HEIGHT * 2];
//unsigned char img_buf[256 * 256 * 3];
unsigned char * img_buf = buf;
//int edg_buf[256 * 256];
int dat_buf[WIDTH * HEIGHT];
int mag_buf[WIDTH * HEIGHT];
float xC_buf[WIDTH * HEIGHT];
float xG_buf[WIDTH * HEIGHT];
float yC_buf[WIDTH * HEIGHT];
float yG_buf[WIDTH * HEIGHT];



void
I_init(Image *img, int width, int height, int type)
{  
  img->width = width;
  img->height = height;
  img->type = type;
  img->data = NULL;
  
  if(type == TYPE_INT_RGB || type == TYPE_INT_ARGB)
    img->data = (unsigned char *)malloc(sizeof(int) * width * height);
  else if(type == TYPE_3BYTE_BGR)
    img->data = img_buf;
 //   img->data = (unsigned char *)malloc(sizeof(char) * 3 * width * height);
  else if(type == TYPE_BYTE_GRAY)
    img->data = (unsigned char *)malloc(sizeof(char) * width * height);
  else if(type == TYPE_USHORT_GRAY)
    img->data = (unsigned char *)malloc(sizeof(short) * width * height);
  else{
    printf("error type\n");
    exit(1);
  }
}

void
I_deinit(Image *img)
{
 // if(img->data != NULL)
 //   free(img->data);
}

void
I_setData(Image *img, int *pixels)
{
  int i, j;
  for(i = 0; i < img->height; i++){
    for(j = 0; j < img->width; j++){
      img->data[(i * img->width + j) * 4 + 2] = pixels[i * img->width + j] == 0xff000000 ? 0 : 0xff;
      img->data[(i * img->width + j) * 4 + 1] = img->data[(i * img->width + j) * 4 + 2];
      img->data[(i * img->width + j) * 4] = img->data[(i * img->width + j) * 4 + 2];
    }
  }
}

void 
C_init(CEDetector *detect)
{
  int s = WIDTH * HEIGHT;
  bzero(xC_buf, s * sizeof(float));
  bzero(xG_buf, s * sizeof(float));
  bzero(yC_buf, s * sizeof(float));
  bzero(yG_buf, s * sizeof(float));
//  bzero(edg_buf, 256 * 256 * sizeof(int));
  bzero(dat_buf, s * sizeof(int));
  bzero(mag_buf, s * sizeof(int));

  detect->lowThreshold = (float)2.5;
  detect->highThreshold = (float)7.5;
  detect->gaussianKernelRadius = (float)2.0;
  detect->gaussianKernelWidth = 16;
  detect->contrastNormalized = FALSE;
  detect->data_len = 0;
  detect->mag_len = 0;
  detect->xC_len = 0;
  detect->yC_len = 0;
  detect->xG_len = 0;
  detect->yG_len = 0;
  detect->data = NULL;
  detect->magnitude = NULL;
  detect->sourceImage = NULL;
  detect->edgesImage = NULL;
  detect->xConv = NULL;
  detect->yConv = NULL;
  detect->xGradient = NULL;
  detect->yGradient = NULL;
}

void 
C_deinit(CEDetector *detect)
{
//  if(detect->data != NULL)
//    free(detect->data);
//  if(detect->magnitude != NULL)
//    free(detect->magnitude);
  if(detect->edgesImage != NULL){
//    if(detect->edgesImage->data != NULL)
//      free(detect->edgesImage->data);
    free(detect->edgesImage);
  }
//  if(detect->xConv != NULL)
//    free(detect->xConv);
//  if(detect->yConv != NULL)
//    free(detect->yConv);
//  if(detect->xGradient != NULL)
//    free(detect->xGradient);
//  if(detect->yGradient != NULL)
//    free(detect->yGradient);

}



void
C_process(CEDetector *detect)
{
  detect->width = detect->sourceImage->width;
  detect->height = detect->sourceImage->height;
  detect->picsize = detect->width * detect->height;
  C_initArrays(detect);
  C_readLuminance(detect);
  if(detect->contrastNormalized){ 
    C_normalizeContrast(detect);
  }
  C_computeGradients(detect, detect->gaussianKernelRadius, detect->gaussianKernelWidth);
  int low = lroundf(detect->lowThreshold * MAGNITUDE_SCALE);
  int high = lroundf(detect->highThreshold * MAGNITUDE_SCALE);
  C_performHysteresis(detect, low, high);
  C_thresholdEdges(detect);
  C_writeEdges(detect, detect->data);
}

void
C_initArrays(CEDetector *detect)
{
  if(detect->data == NULL || detect->picsize != detect->data_len){
    detect->data = dat_buf;
//    detect->data = (int *)malloc(detect->picsize * sizeof(int));
    detect->data_len = detect->picsize;
    detect->magnitude = mag_buf;
//    detect->magnitude = (int *)malloc(detect->picsize * sizeof(int));
    detect->mag_len = detect->picsize;

    detect->xConv = xC_buf;
//    detect->xConv = (float *)malloc(detect->picsize * sizeof(float));
    detect->xC_len = detect->picsize;
    detect->yConv = yC_buf;
//    detect->yConv = (float *)malloc(detect->picsize * sizeof(float));
    detect->yC_len = detect->picsize;
    detect->xGradient = xG_buf;
//    detect->xGradient = (float *)malloc(detect->picsize * sizeof(float));
    detect->xG_len = detect->picsize;
    detect->yGradient = yG_buf;
//    detect->yGradient = (float *)malloc(detect->picsize * sizeof(float));
    detect->yG_len = detect->picsize;
  }
}

void
C_computeGradients(CEDetector *detect, float kernelRadius, int kernelWidth)
{
  float *kernel = (float *)malloc(kernelWidth * sizeof(float));
  float *diffKernel = (float *)malloc(kernelWidth * sizeof(float));
  int kwidth;
  float g1, g2, g3;
  for(kwidth = 0; kwidth < kernelWidth; kwidth++){
    g1 = gaussian(kwidth, kernelRadius);
    if(g1 <= GAUSSIAN_CUT_OFF && kwidth >= 2) break;
    g2 = gaussian(kwidth - 0.5, kernelRadius);
    g3 = gaussian(kwidth + 0.5, kernelRadius);
    kernel[kwidth] = (g1 + g2 + g3) / (float)3 / ((float)2 * PI * kernelRadius * kernelRadius);
    diffKernel[kwidth] = g3 - g2;

//printf("kwidth:%d ,kernel:%x ,diffKernel:%x\n", kwidth, *((int*)&kernel[kwidth]), *((int*)&diffKernel[kwidth]));
  }

  int initX = kwidth - 1;
  int maxX = detect->width - (kwidth - 1);
  int initY = detect->width * (kwidth - 1);
  int maxY = detect->width * (detect->height - (kwidth - 1));

  int x, y, index;
  float sumX, sumY;
  int xOffset, yOffset;
  for(x = initX; x < maxX; x++){
    for(y = initY; y < maxY; y += detect->width){
      index = x + y;
      sumX = detect->data[index] * kernel[0];
      sumY = sumX;
      xOffset = 1;
      yOffset = detect->width;
      for( ; xOffset < kwidth; ){
        sumY += kernel[xOffset] * (detect->data[index - yOffset] + detect->data[index + yOffset]);
        sumX += kernel[xOffset] * (detect->data[index - xOffset] + detect->data[index + xOffset]);
        yOffset += detect->width;
        xOffset++;

      }

      detect->yConv[index] = sumY;
      detect->xConv[index] = sumX;

    }
  }

  float sum;
  int i;
  for(x = initX; x < maxX; x++){
    for(y = initY; y < maxY; y += detect->width){
      sum = 0;
      index = x + y;
      for(i = 1; i < kwidth; i++)
        sum += diffKernel[i] * (detect->yConv[index - i] - detect->yConv[index + i]);

      detect->xGradient[index] = sum;
    }
  }

  for(x = kwidth; x < detect->width - kwidth; x++){
    for(y = initY; y < maxY; y += detect->width){
      sum = 0;
      index = x + y;
      yOffset = detect->width;
      for(i = 1; i < kwidth; i++){
        sum += diffKernel[i] * (detect->xConv[index - yOffset] - detect->xConv[index + yOffset]);
        yOffset += detect->width;
      }

      detect->yGradient[index] = sum;
    }
  }

  initX = kwidth;
  maxX = detect->width - kwidth;
  initY = detect->width * kwidth;
  maxY = detect->width * (detect->height - kwidth);
  int indexN, indexS, indexW, indexE, indexNW, indexNE, indexSW, indexSE;
  float xGrad, yGrad, gradMag;
  float nMag, sMag, wMag, eMag, neMag, seMag, swMag, nwMag, tmp;
  for(x = initX; x < maxX; x++){
    for(y = initY; y < maxY; y += detect->width){
      index = x + y;
      indexN = index - detect->width;
      indexS = index + detect->width;
      indexW = index - 1;
      indexE = index + 1;
      indexNW = indexN - 1;
      indexNE = indexN + 1;
      indexSW = indexS - 1;
      indexSE = indexS + 1;

      xGrad = detect->xGradient[index];
      yGrad = detect->yGradient[index];
      gradMag = hypot(xGrad, yGrad);

      nMag = hypot(detect->xGradient[indexN], detect->yGradient[indexN]);
      sMag = hypot(detect->xGradient[indexS], detect->yGradient[indexS]);
      wMag = hypot(detect->xGradient[indexW], detect->yGradient[indexW]);
      eMag = hypot(detect->xGradient[indexE], detect->yGradient[indexE]);
      neMag = hypot(detect->xGradient[indexNE], detect->yGradient[indexNE]);
      seMag = hypot(detect->xGradient[indexSE], detect->yGradient[indexSE]);
      swMag = hypot(detect->xGradient[indexSW], detect->yGradient[indexSW]);
      nwMag = hypot(detect->xGradient[indexNW], detect->yGradient[indexNW]);

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
          detect->magnitude[index] = MAGNITUDE_MAX;
        }
        else{
          detect->magnitude[index] = (int)(MAGNITUDE_SCALE * gradMag);
        } 
      }
      else{
        detect->magnitude[index] = 0;
      }
    }
  }
}


float
gaussian(float x, float sigma)
{
  return (float)pow(M_E, - (x * x) / ((float)2 * sigma * sigma));
}

void
C_performHysteresis(CEDetector *detect, int low, int high)
{
  memset(detect->data, 0, detect->data_len * sizeof(int));

  int offset = 0, y, x;
  for(y = 0; y < detect->height; y++){
    for(x = 0; x < detect->width; x++){
      if(detect->data[offset] == 0 && detect->magnitude[offset] >= high){
        C_follow(detect, x, y, offset, low);
      }
      offset++;
    }
  }
//  printf("%d %d\n", low, high);
}

void
C_follow(CEDetector *detect, int x1, int y1, int i1, int threshold)
{
  int x0 = x1 == 0 ? x1 : x1 - 1;
  int x2 = x1 == detect->width - 1 ? x1 : x1 + 1;
  int y0 = y1 == 0 ? y1 : y1 - 1;
  int y2 = y1 == detect->height - 1 ? y1 : y1 + 1;

  detect->data[i1] = detect->magnitude[i1];
  int x, y, i2;
  for(x = x0; x <= x2; x++){
    for(y = y0; y <= y2; y++){
      i2 = x + y * detect->width;
      if((y != y1 || x != x1) && detect->data[i2] == 0
        && detect->magnitude[i2] >= threshold){
        C_follow(detect, x, y, i2, threshold);
        return;
      }
    }
  }
}

void
C_thresholdEdges(CEDetector *detect)
{
  int i;
  for(i = 0; i < detect->picsize; i++)
    detect->data[i] = detect->data[i] > 0 ? -1 : 0xff000000;
}

int
luminance(float r, float g, float b)
{
  return lroundf((float)0.299 * r + (float)0.587 * g + (float)0.114 * b);
}

void
C_readLuminance(CEDetector *detect)
{
  int type = detect->sourceImage->type;
  int *pixels;
  int p, r, g, b, i;
  unsigned char *pixels2;
  unsigned short *pixels3;
  int offset;
  if(type == TYPE_INT_RGB || type == TYPE_INT_ARGB){
    pixels = (int *)detect->sourceImage->data;
    for(i = 0; i < detect->picsize; i++){
      p = pixels[i];
      r = (p & 0xff0000) >> 16;
      g = (p & 0xff00) >> 8;
      b = p & 0xff;
      detect->data[i] = luminance(r, g, b);
    }
  }
  else if(type == TYPE_BYTE_GRAY){
    pixels2 = (unsigned char *)detect->sourceImage->data;
    for(i = 0; i < detect->picsize; i++)
      detect->data[i] = (pixels2[i] & 0xff);
  }
  else if(type == TYPE_USHORT_GRAY){
    pixels3 = (unsigned short *)detect->sourceImage->data;
    for(i = 0; i < detect->picsize; i++)
      detect->data[i] = (pixels3[i] & 0xffff) / 256;
  }
  else if(type == TYPE_3BYTE_BGR){
    pixels2 = (unsigned char *)detect->sourceImage->data;
    offset = 0;
    for(i = 0; i < detect->picsize; i++){
      b = pixels2[offset++] & 0xff;
      g = pixels2[offset++] & 0xff;
      r = pixels2[offset++] & 0xff; 
      detect->data[i] = luminance(r, g, b);
    }
  }
  else
    printf("C_readLuminance error\n");
}

void
C_normalizeContrast(CEDetector *detect)
{
  int *histogram = (int *)malloc(256 * sizeof(int));
  int i;
  for(i = 0; i < detect->data_len; i++)
    histogram[detect->data[i]]++;

  int *remap = (int *)malloc(256 * sizeof(int));
  int sum = 0, j = 0, target, k;
  for(i = 0; i < 256; i++){
    sum += histogram[i];
    target = sum * 255 / detect->picsize;
    for(k = j + 1; k <= target; k++)
      remap[k] = i;
    j = target;
  }

  for(i = 0; i < detect->data_len; i++){
    detect->data[i] = remap[detect->data[i]];
  }
}

void
C_writeEdges(CEDetector *detect, int *pixels)
{
  /* edgesImage only support TYPE_INT_ARGB now */
  if(detect->edgesImage == NULL){
    detect->edgesImage = (Image *)malloc(sizeof(Image));
//    I_init(detect->edgesImage, detect->width, detect->height, TYPE_INT_ARGB);
    detect->edgesImage->width = detect->width;
    detect->edgesImage->height = detect->height;
    detect->edgesImage->type = TYPE_INT_ARGB;
    detect->edgesImage->data = (unsigned char *)mag_buf;
    bzero(mag_buf, WIDTH * HEIGHT * sizeof(int));
  }
  I_setData(detect->edgesImage, pixels);
}

/* vi: set et sw=2 sts=2: */
