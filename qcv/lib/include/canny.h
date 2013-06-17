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
  float low_threshold, high_threshold;
  int aperture_size;
} qcv_canny_params_t;

#define BORDER_REPLICATE 0

#define QCV_DEFAULT_CANNY_PARAMS {                        \
    .low_threshold = 2.5, .high_threshold = 7.5,          \
      .aperture_size = 16}

int qcv_canny(qcv_frame_t * img_in, qcv_canny_params_t* params, qcv_frame_t * img_out);


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
