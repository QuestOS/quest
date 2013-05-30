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

#ifndef _QCV_MATRIX_H_
#define _QCV_MATRIX_H_

#include <stdlib.h>

typedef int qcv_matrix_type_t;

typedef struct {

  size_t width;
  size_t height;
  size_t element_size;
  size_t row_stride;
  size_t channels;
  qcv_matrix_type_t type;
  
  size_t buf_size;
  unsigned char* buf;
  
} qcv_matrix_t;

#define qcv_matrix_width(m) ((m)->width)
#define qcv_matrix_height(m) ((m)->height)
#define qcv_matrix_cols(m) ((m)->width)
#define qcv_matrix_rows(m) ((m)->height)
#define qcv_matrix_type(m) ((m)->type)
#define qcv_matrix_buf(m) ((m)->buf)
#define qcv_matrix_buf_size(m) ((m)->buf_size)
#define qcv_matrix_channels(m) ((m)->channels)
#define qcv_matrix_element_size(m) ((m)->element_size)
#define qcv_matrix_row_stride(m) ((m)->row_stride)
#define qcv_matrix_element(m, t, i, c) (*((t*)&((m)->buf[sizeof(t) * ((i) * (m)->channels) + (c)])))

int qcv_create_matrix(qcv_matrix_t* matrix, size_t width, size_t height, qcv_matrix_type_t type);
void qcv_free_matrix(qcv_matrix_t* matrix);

#endif // _QCV_MATRIX_H_


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
