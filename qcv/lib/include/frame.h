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

#ifndef _QCV_FRAME_H_
#define _QCV_FRAME_H_

#include "qcv_types.h"
#include <stdlib.h>
#include "matrix.h"


typedef enum {
  QCV_FRAME_TYPE_3BYTE_RGB,
  QCV_FRAME_TYPE_1BYTE_GREY,
} qcv_frame_type_t;

typedef struct {

  qcv_matrix_t pixel_matrix;
  qcv_frame_type_t type;
} qcv_frame_t;

#define qcv_frame_matrix(f) (&((f)->pixel_matrix))
#define qcv_frame_width(f) (qcv_matrix_width(qcv_frame_matrix(f)))
#define qcv_frame_height(f) (qcv_matrix_height(qcv_frame_matrix(f)))
#define qcv_frame_row_stride(f) (qcv_matrix_row_stride(qcv_frame_matrix(f)))
#define qcv_frame_element_size(f) (qcv_matrix_element_size(qcv_frame_matrix(f)))
#define qcv_frame_buf_size(f) (qcv_matrix_buf_size(qcv_frame_matrix(f)))
#define qcv_frame_buf(f) (qcv_matrix_buf(qcv_frame_matrix(f)))
#define qcv_frame_type(f) ((f)->type)
#define qcv_frame_element(f, t, i, c) qcv_matrix_element(qcv_frame_matrix(f), t, i, c)
#define qcv_frame_element_coord(f, t, x, y, c)          \
  qcv_matrix_element(qcv_frame_matrix(f), t, (x) + (y) * qcv_frame_width(f), c)

void qcv_release_frame(qcv_frame_t* frame);
int qcv_create_frame(qcv_frame_t* frame, size_t width, size_t height, qcv_frame_type_t type);
int qcv_frame_from_file(qcv_frame_t* frame, char* file);
int qcv_frame_luminance(qcv_frame_t* frame, int x, int y);
int qcv_frame_convert_to(qcv_frame_t* src_frame, qcv_frame_t* target_frame,
                         qcv_frame_type_t target_type);
qcv_matrix_type_t qcv_matrix_type_for_frame_type(qcv_frame_type_t frame_type);


#endif // _QCV_FRAME_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
