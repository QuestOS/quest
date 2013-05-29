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
#include <stdlib.h>



void qcv_release_frame(qcv_frame_t* frame)
{
  if(frame->pixel_matrix.buf) {
    free(frame->pixel_matrix.buf);
    frame->pixel_matrix.buf = NULL;
  }
}

int qcv_create_frame(qcv_frame_t* frame, size_t width, size_t height, qcv_frame_type_t type)
{
  frame->pixel_matrix.width = width;
  frame->pixel_matrix.height = height;
  frame->pixel_matrix.row_stride = height;

  frame->type = type;
  
  switch(type) {
  case QCV_FRAME_TYPE_3BYTE_RGB:
    frame->pixel_matrix.element_size = 3;
    break;
  case QCV_FRAME_TYPE_1BYTE_GREY:
    frame->pixel_matrix.element_size = 1;
    break;
  default:
    return -1;
  }
  
  frame->pixel_matrix.row_stride = (frame->pixel_matrix.width) * (frame->pixel_matrix.element_size);

  
  frame->pixel_matrix.buf_size =
    (frame->pixel_matrix.width) * (frame->pixel_matrix.height) * (frame->pixel_matrix.element_size);
  frame->pixel_matrix.buf = (unsigned char*) malloc(frame->pixel_matrix.buf_size);

  if(!frame->pixel_matrix.buf) return -1;
  
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
