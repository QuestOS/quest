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



void qcv_release_frame(qcv_frame_t* frame)
{
  if(frame->pixel_matrix.buf) {
    free(frame->pixel_matrix.buf);
    frame->pixel_matrix.buf = NULL;
  }
}

int qcv_create_frame(qcv_frame_t* frame, size_t width, size_t height, qcv_frame_type_t type)
{
  qcv_frame_width(frame) = width;
  qcv_frame_height(frame) = height;

  qcv_frame_type(frame) = type;
  
  switch(type) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    qcv_frame_element_size(frame) = 3;
    break;
  case QCV_FRAME_TYPE_1BYTE_GREY:
    qcv_frame_element_size(frame) = 1;
    break;
  default:
    return -1;
  }
  
  qcv_frame_row_stride(frame) = width * qcv_frame_element_size(frame);

  
  qcv_frame_buf_size(frame) = width * height * qcv_frame_element_size(frame);
  qcv_frame_buf(frame) = (unsigned char*) malloc(qcv_frame_buf_size(frame));

  if(!qcv_frame_buf(frame)) return -1;
  
  return 0;
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


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
