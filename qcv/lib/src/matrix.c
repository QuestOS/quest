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

#include "matrix.h"
#include "qcv_types.h"

int qcv_create_matrix(qcv_matrix_t* matrix, size_t width, size_t height, qcv_matrix_type_t type)
{
  matrix->width = width;
  matrix->height = height;
  matrix->type = type;
  matrix->channels = QCV_MAT_CN(type);
  switch(QCV_MAT_DEPTH(type)) {
  case QCV_8U:
  case QCV_8S:
    matrix->element_size = 1;
    break;
    
  case QCV_16U:
  case QCV_16S:
    matrix->element_size = 2;
    break;

  case QCV_32S:
  case QCV_32F:
    matrix->element_size = 4;
    break;

  case QCV_64F:
    matrix->element_size = 8;
    
  default:
    return -1;
  }
  
  matrix->row_stride = matrix->element_size * width * matrix->channels;
  matrix->buf_size = matrix->row_stride * height;

  matrix->buf = malloc(matrix->buf_size);

  return matrix->buf ? 0 : -1;
}

void qcv_free_matrix(qcv_matrix_t* matrix)
{
  if(matrix->buf) { free(matrix->buf); matrix->buf = NULL;}
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
