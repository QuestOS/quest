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

static void putpixel(unsigned char* screen, int x, int y, int color) {
  screen[x + y*320] = color & 255;
}

void main()
{
  char* video_memory;
  int i, j;
  printf("About to enable video mode\n");
  if(enable_video(1, &video_memory) < 0) {
    printf("Failed to enable video mode\n");
    exit(1);
  }

  //memset(video_memory, 0, 0x1000 * 16);
  
  /* printf("video_memory = %p\n", video_memory); */
  /* for(j = 0; j < 200; ++j) { */
  /*   for(i = 0; i < 320; ++i) { */
  /*     putpixel(video_memory, i, j, 0); */
  /*   } */
  /*   usleep(100000); */
  /* } */


  for(i = 0; i < 2; ++i) {
    memset(video_memory, i, 320 * 200 /2);
    usleep(1000000);
    memset(video_memory + (320 * 200 /2), i, 320 * 200 /2);
    usleep(1000000);
  }

  if(enable_video(0, NULL) < 0) {
    printf("Failed to set text mode\n");
  }
  
  /* memset(video_memory, 0, 320 * 480 / 4); */
  /* for(j = 1; j < 255; ++j) { */
  /*   for(i = 0; i < 320 * 480 / 4; ++i) { */
  /*     video_memory[i] = j; */
  /*     usleep(10000); */
  /*   } */
  /* } */

  //while(1);
  
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
