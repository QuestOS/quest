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
#include "string.h"
#include <qcv/qcv.h>

#define IMG_NAME "/boot/test.jpg"


void main()
{
  qcv_frame_t frame;
  qcv_window_t window;

  if(qcv_frame_from_file(&frame, IMG_NAME) < 0) {
    printf("Failed to read image\n");
    exit(EXIT_FAILURE);
  }

  printf("width = %u, height = %u\n", qcv_frame_width(&frame), qcv_frame_height(&frame));

  if(qcv_create_window(&window) < 0) {
    printf("Failed to create window\n");
    exit(EXIT_FAILURE);
  }

  if(qcv_window_display_frame(&window, &frame) < 0) {
    printf("Failed to display image\n");
    exit(EXIT_FAILURE);
  }
  
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
