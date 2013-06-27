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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "capture.h"
#include "jpeg.h"
#include <usb.h>

static void init_capture(qcv_capture_t* capture, capture_source_t source)
{
  memset(capture, 0, sizeof(*capture));
  capture->source = source;
}

int qcv_capture_from_camera(qcv_capture_t* capture, int index)
{
  char cam_name[20];
  int res;

  init_capture(capture, CAPTURE_SOURCE_CAMERA);

  capture->source_camera.uncompressed_frame =
    malloc(capture->source_camera.uncompressed_frame_buf_len = CAMERA_UNCOMPRSSED_FRAME_BUF_LEN);

  if(!capture->source_camera.uncompressed_frame) return -1;
  
  
  res = snprintf(cam_name, sizeof(cam_name)/sizeof(cam_name[0]),  "camera%d", index);

  if((res < 0) || (res == sizeof(cam_name)/sizeof(cam_name[0]))) {
    free(capture->source_camera.uncompressed_frame);
    return -1;
  }
    
  res = usb_open(cam_name);

  if(res < 0) {
    free(capture->source_camera.uncompressed_frame);
    return -1;
  }

  capture->source_fd = res;
  
  return 0;
}

#define DEFAULT_FILE_CAPTURE_BUF_SIZE 4096

int qcv_capture_from_file(qcv_capture_t* capture, char * const filename)
{
  static BOOL avcodecs_registered = FALSE;

  if(!avcodecs_registered) { avcodec_register_all(); avcodecs_registered = TRUE; }

  init_capture(capture, CAPTURE_SOURCE_FILE);
  
  capture->source_file.done = FALSE;
  capture->source_file.av_pkt = malloc(sizeof(AVPacket));

  if(!capture->source_file.av_pkt) return -1;
  
  capture->source_file.buf_size = DEFAULT_FILE_CAPTURE_BUF_SIZE;
  capture->source_file.buf = malloc(capture->source_file.buf_size + FF_INPUT_BUFFER_PADDING_SIZE);
  
  if(!capture->source_file.buf) goto cleanup_1;
  
  memset(capture->source_file.buf + capture->source_file.buf_size, 0, FF_INPUT_BUFFER_PADDING_SIZE);

  av_init_packet(capture->source_file.av_pkt);

  /* -- EM -- Only supporting MEPG1 right now, change this later to
     look at the file extension and/or examine file internally to
     determine codec */
  
  capture->source_file.av_codec = avcodec_find_decoder(AV_CODEC_ID_MPEG1VIDEO);
  
  if(!capture->source_file.av_codec) goto cleanup_2;

  capture->source_file.av_codec_context = avcodec_alloc_context3(capture->source_file.av_codec);

  if(!capture->source_file.av_codec_context) goto cleanup_2;

  if(capture->source_file.av_codec->capabilities & CODEC_CAP_TRUNCATED) {
    /* we do not send complete frames */
    capture->source_file.av_codec_context->flags|= CODEC_FLAG_TRUNCATED; 
  }
  
  if (avcodec_open2(capture->source_file.av_codec_context,
                    capture->source_file.av_codec, NULL) < 0) goto cleanup_3;
  
  if((capture->source_fd = open(filename, O_RDONLY)) < 0) goto cleanup_4;

  capture->source_file.av_frame = avcodec_alloc_frame();
  if(!capture->source_file.av_frame) goto cleanup_5;

  return 0;

 cleanup_5:
  close(capture->source_fd);
 cleanup_4:
  avcodec_close(capture->source_file.av_codec_context);
 cleanup_3:
  av_free(capture->source_file.av_codec_context);
 cleanup_2:
  free(capture->source_file.buf);
 cleanup_1:
  free(capture->source_file.av_pkt);
  return -1;
  
}

int qcv_grab_frame(qcv_capture_t* capture)
{
  int bytes_read;
  switch(capture->source) {
  case CAPTURE_SOURCE_CAMERA:
    bytes_read = usb_read(capture->source_fd, capture->source_camera.uncompressed_frame,
                          capture->source_camera.uncompressed_frame_buf_len);
    if(bytes_read < 0) return -1;
    capture->source_camera.uncompressed_frame_len = bytes_read;
    
    return 0;

    case CAPTURE_SOURCE_FILE:
      /* -- EM -- I don't know how to best separate grabbing the frame
         decompressing it using the av libs and since the distinction
         most important for cameras (to synchronize images) just do
         nothing here and everything in qcv_retrieve_frame (at least
         for now) except check to see if the file is done*/
      return capture->source_file.done ? -1 : 0;
  default:
    return -1;
  }
}

/* Returns < 0 on error, 0 if no frame is obtained and > 0 if a frame is
   obtained */
static int decode_av_file_frame(qcv_capture_t* capture, qcv_frame_t* frame)
{
  int ret_val = 0;
  int q;
  int len, got_frame;
  unsigned int *temp_ptr = capture->source_file.av_pkt->data;
  len = avcodec_decode_video2(capture->source_file.av_codec_context,
                              capture->source_file.av_frame, &got_frame,
                              capture->source_file.av_pkt);
  if (len < 0) {
    return len;
  }
  if (got_frame) {
    if(qcv_create_frame_from_av_frame(frame, 320, 200, QCV_FRAME_TYPE_3BYTE_RGB,
                                      capture->source_file.av_frame,
                                      PIX_FMT_YUV420P) < 0) {
      return -1;
    }
    ret_val = 1;
  }
  if (capture->source_file.av_pkt->data) {
    capture->source_file.av_pkt->size -= len;
    capture->source_file.av_pkt->data += len;
  }
  else {
    capture->source_file.done = TRUE;
  }
  return ret_val;
}

int qcv_retrieve_frame(qcv_capture_t* capture, qcv_frame_t* frame)
{
  switch(capture->source) {
  case CAPTURE_SOURCE_CAMERA:

    if(!capture->source_camera.uncompressed_frame_len) return -1;
    return qcv_jpeg_to_rgb(capture->source_camera.uncompressed_frame,
                           capture->source_camera.uncompressed_frame_len,
                        frame);

  case CAPTURE_SOURCE_FILE:
    while(1) {
      int q;
      unsigned int *temp_ptr;
      if(capture->source_file.av_pkt->size == 0) {
        capture->source_file.av_pkt->size =
          read(capture->source_fd, capture->source_file.buf, capture->source_file.buf_size);
                
        if(capture->source_file.av_pkt->size < 0) {
          if(errno == EINTR) continue;
          return -1;
        }
      
      
        if(capture->source_file.av_pkt->size == 0) {
          capture->source_file.av_pkt->data = NULL;
        }
        else {
          capture->source_file.av_pkt->data = capture->source_file.buf;
        }
      }
      switch(decode_av_file_frame(capture, frame)) {
      case -1:
        return -1;

      case 1:
        return 0;
      }
    }
  default:
    return -1;
  }
}

int qcv_query_frame(qcv_capture_t* capture, qcv_frame_t* frame)
{
  int res;
  if((res = qcv_grab_frame(capture)) < 0) {
    return res;
  }
  return qcv_retrieve_frame(capture, frame);
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
