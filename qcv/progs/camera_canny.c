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

#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include "usb.h"
#include <qcv/qcv.h>

void main()
{
  qcv_window_t window;
  qcv_capture_t camera_capture;
  int camera_fd;
  qcv_frame_t frame, canny_frame;
  qcv_canny_params_t canny_params = QCV_DEFAULT_CANNY_PARAMS;
  
  if(qcv_capture_from_camera(&camera_capture, 0) < 0) {
    printf("Failed to initialise camera capture \n");
    exit(EXIT_FAILURE);
  }

  printf("About to create the window\n");
  if(qcv_create_window(&window) < 0) {
    printf("Failed to create window\n");
    exit(EXIT_FAILURE);
  }

  while(1) {
    if(qcv_query_frame(&camera_capture, &frame) < 0) {
      printf("Failed to pull frame\n");
      exit(EXIT_FAILURE);
    }

    if(qcv_canny(&frame, &canny_params, &canny_frame) < 0) {
      printf("canny failed\n");
    }
    
    qcv_window_display_frame(&window, &canny_frame);
    qcv_release_frame(&frame);
    qcv_release_frame(&canny_frame);
  }

  printf("At end of camera\n");
  while(1);
  
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
