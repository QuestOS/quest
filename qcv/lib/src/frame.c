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

#include "frame.h"
#include <unistd.h>
#include <fcntl.h>
#include "jpeg.h"
#include "qcv_error.h"
#include <math.h>



void qcv_release_frame(qcv_frame_t* frame)
{
  if(frame->pixel_matrix.buf) {
    free(frame->pixel_matrix.buf);
    frame->pixel_matrix.buf = NULL;
  }
}

qcv_matrix_type_t qcv_matrix_type_for_frame_type(qcv_frame_type_t frame_type)
{
  switch(frame_type) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    return QCV_8UC3;
  case QCV_FRAME_TYPE_1BYTE_GREY:
    return QCV_8UC1;
  default:
    return -1;
  }
}

int qcv_create_frame(qcv_frame_t* frame, size_t width, size_t height, qcv_frame_type_t type)
{
  qcv_matrix_type_t matrix_type;

  qcv_frame_type(frame) = type;
  
  matrix_type = qcv_matrix_type_for_frame_type(type);
  if(matrix_type < 0) return matrix_type;
  
  return qcv_create_matrix(qcv_frame_matrix(frame), width, height, matrix_type);
}

int qcv_frame_from_file(qcv_frame_t* frame, char* file)
{
#define MAX_IMG_SIZE (0x10000)
  static unsigned char img_buffer[MAX_IMG_SIZE];
  int img_fd;
  int bytes_read;
  
  if((img_fd = open(file, O_RDONLY)) < 0) {
    return img_fd;
  }
  
  if((bytes_read = read(img_fd, img_buffer, MAX_IMG_SIZE)) < 0) {
    return bytes_read;
  }
  
  if(bytes_read == MAX_IMG_SIZE) {
    /* The call to read filled up the entire buffer, most likely
       didn't get the entire image */
    return -1;
  }
  
  return qcv_jpeg_to_rgb(img_buffer, bytes_read, frame);
}

#define luminance(r, g,  b)                                             \
  lroundf(0.299f * r + 0.587f * g + 0.114f * b);


int qcv_frame_luminance(qcv_frame_t* frame, int x, int y)
{
  switch(qcv_frame_type(frame)) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    return luminance(qcv_frame_element(frame, unsigned char, x + y * frame->pixel_matrix.width, 0),
                     qcv_frame_element(frame, unsigned char, x + y * frame->pixel_matrix.width, 1),
                     qcv_frame_element(frame, unsigned char, x + y * frame->pixel_matrix.width, 2));
  case QCV_FRAME_TYPE_1BYTE_GREY:
    return qcv_frame_element(frame, unsigned char, x + y * frame->pixel_matrix.width, 0);

  default:
    qcv_error("Known frame type passed to qcv_frame_luminance");
  }
}


int qcv_frame_convert_to(qcv_frame_t* src_frame, qcv_frame_t* target_frame,
                         qcv_frame_type_t target_type)
{
  size_t x,y;
  size_t width = qcv_frame_width(src_frame);
  size_t height = qcv_frame_width(src_frame);
  if(qcv_create_frame(target_frame, qcv_frame_width(src_frame), qcv_frame_height(src_frame),
                      target_type) < 0) return -1;
  
  switch(target_type) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    switch(qcv_frame_type(src_frame)) {
    case QCV_FRAME_TYPE_3BYTE_RGB:
      
    case QCV_FRAME_TYPE_1BYTE_GREY:
      
    default:
      qcv_release_frame(target_frame);
      return -1;
    }
  case QCV_FRAME_TYPE_1BYTE_GREY:
    for(x = 0; x < width; ++x) {
      for(y = 0; y < height; ++y) {
        qcv_frame_element_coord(target_frame, unsigned char, x, y, 0) = qcv_frame_luminance(src_frame, x, y);
      }
    }
    return 0;
    
  default:
    qcv_release_frame(target_frame);
    return -1;
  }
}

static enum PixelFormat qcv_av_pxl_fmt_from_frame_type(qcv_frame_type_t type) {
  switch(type) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    return PIX_FMT_RGB24;
  default:
    qcv_error("Unsupport qcv type");
  }
}

int qcv_create_frame_from_av_frame(qcv_frame_t* frame, size_t width,
                                   size_t height, qcv_frame_type_t type,
                                   AVFrame* av_frame,
                                   enum PixelFormat av_frame_pxl_fmt)
{
  unsigned char* rgb_frame;
  enum PixelFormat av_target_pxl_fmt = qcv_av_pxl_fmt_from_frame_type(type);
  if(qcv_create_frame(frame, width, height, type) < 0) {
    return -1;
  }
  
  rgb_frame = av_malloc(avpicture_get_size(av_target_pxl_fmt, width, height));
  if(!rgb_frame) {
    qcv_release_frame(frame);
    return -1;
  }
  
  AVFrame *avFrameRGB = avcodec_alloc_frame();
  if(!avFrameRGB) {
    qcv_release_frame(frame);
    av_free(rgb_frame);
    return -1;
  }
  struct SwsContext *img_convert_ctx;
  img_convert_ctx = sws_getContext(av_frame->width, av_frame->height, av_frame_pxl_fmt,
				   width, height,
				   av_target_pxl_fmt, SWS_BICUBIC, NULL, NULL, NULL);

  
  if(!img_convert_ctx) {
    avcodec_free_frame(&avFrameRGB);
    qcv_release_frame(frame);
    av_free(rgb_frame);
    return -1;
  }
  
  avpicture_fill((AVPicture *)avFrameRGB, rgb_frame, PIX_FMT_RGB24, width, height);
  sws_scale(img_convert_ctx, (const uint8_t * const*)av_frame->data, av_frame->linesize, 0,
	    av_frame->height, avFrameRGB->data, avFrameRGB->linesize);
  memcpy(qcv_frame_buf(frame), avFrameRGB->data[0], qcv_frame_buf_size(frame));


  sws_freeContext(img_convert_ctx);
  avcodec_free_frame(&avFrameRGB);
  av_free(rgb_frame);
  return 0;
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
