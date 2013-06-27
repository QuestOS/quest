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
#include <qcv/qcv.h>


#define MPEG_FILE "/boot/test.mpg"

void main()
{
  qcv_window_t window;
  qcv_capture_t file_capture;
  qcv_frame_t frame;
  printf("*********************Not Working Version****************\n\n\n\n\n");
  if(qcv_capture_from_file(&file_capture, MPEG_FILE) < 0) {
    printf("Failed to initialise file capture \n");
    exit(EXIT_FAILURE);
  }

  printf("About to create the window\n");
  if(qcv_create_window(&window) < 0) {
    printf("Failed to create window\n");
    exit(EXIT_FAILURE);
  }

  while(1) {
    if(qcv_query_frame(&file_capture, &frame) < 0) {
      printf("Failed to pull frame\n");
      while(1);
    }
    
    qcv_window_display_frame(&window, &frame);
    qcv_release_frame(&frame);
  }

  printf("At end of mpeg_test");
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
