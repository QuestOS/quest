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

#ifndef _QCV_CANNY_H_
#define _QCV_CANNY_H_

#include "frame.h"

typedef struct {
  float gaussian_kernel_radius;
  float low_threshold, high_threshold;
  int gaussian_kernel_width;
  int contrast_normalized;
} qcv_canny_params_t;

#define QCV_DEFAULT_CANNY_PARAMS {                        \
    .gaussian_kernel_radius = 2.0,                        \
      .low_threshold = 2.5, .high_threshold = 7.5,        \
      .gaussian_kernel_width = 16,                        \
      .contrast_normalized = 0 }

int qcv_canny_frame(qcv_frame_t* frame, qcv_canny_params_t* params, qcv_frame_t* out_frame);


#endif // _QCV_CANNY_H_


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
