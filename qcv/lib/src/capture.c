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

  capture->uncompressed_frame =
    malloc(capture->uncompressed_frame_buf_len = CAMERA_UNCOMPRSSED_FRAME_BUF_LEN);

  if(!capture->uncompressed_frame) return -1;
  
  
  res = snprintf(cam_name, sizeof(cam_name)/sizeof(cam_name[0]),  "camera%d", index);

  if((res < 0) || (res == sizeof(cam_name)/sizeof(cam_name[0]))) return -1;
    
  res = usb_open(cam_name);

  if(res < 0) return -1;

  capture->camera_fd = res;
  
  return 0;
}

int qcv_grab_frame(qcv_capture_t* capture)
{
  int bytes_read;
  switch(capture->source) {
  case CAPTURE_SOURCE_CAMERA:
    bytes_read = usb_read(capture->camera_fd, capture->uncompressed_frame,
                          capture->uncompressed_frame_buf_len);
    if(bytes_read < 0) return -1;
    capture->uncompressed_frame_len = bytes_read;
    
    return 0;
    
  default:
    return -1;
  }
}

int qcv_retrieve_frame(qcv_capture_t* capture, qcv_frame_t* frame)
{
  switch(capture->source) {
  case CAPTURE_SOURCE_CAMERA:

    if(!capture->uncompressed_frame_len) return -1;
    return qcv_jpeg_to_rgb(capture->uncompressed_frame, capture->uncompressed_frame_len,
                        frame);
    
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
