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

#ifndef _QCV_CAPTURE_H_
#define _QCV_CAPTURE_H_

#include "frame.h"

typedef enum {
  CAPTURE_SOURCE_CAMERA,
} capture_source_t;

typedef struct {
  int camera_fd;
  capture_source_t source;
  unsigned char* uncompressed_frame;
  size_t uncompressed_frame_buf_len;
  size_t uncompressed_frame_len;
} qcv_capture_t;

#define CAMERA_UNCOMPRSSED_FRAME_BUF_LEN ((size_t)40960)

int qcv_capture_from_camera(qcv_capture_t* capture, int index);
int qcv_grab_frame(qcv_capture_t* capture);
int qcv_retrieve_frame(qcv_capture_t* capture, qcv_frame_t* frame);
int qcv_query_frame(qcv_capture_t* capture, qcv_frame_t* frame);

#endif // _QCV_CAPTURE_H_


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
