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

int qcv_display_frame(qcv_frame_t* frame, unsigned char* video_memory)
{
  int x, y;
  printf("height = %d, width = %d\n", frame->height, frame->width);
  for(y = 0; y < 200; ++y) {
    //printf("y = %d\n", y);
    for(x = 0; x < frame->width; ++x) {
      video_memory[x + y * frame->width] = frame->img_buf[(x + y * frame->width) * 3] / 16;
    }
  }
  return 0;
}

void qcv_release_frame(qcv_frame_t* frame)
{
  if(frame->img_buf) {
    free(frame->img_buf);
    frame->img_buf = NULL;
  }
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
