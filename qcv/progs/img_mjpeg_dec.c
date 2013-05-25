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

#define IMG_NAME "test.jpg"
#define IMG_SIZE (56216)
char img_buffer[IMG_SIZE];


void main()
{
  char* video_memory;
  int img_fd;
  int i;
  int bytes_read;
  size_t width, height, row_stride, pixel_size, rgb_size;
  unsigned char* rgb_buf;
  
  if((img_fd = open("/boot/" IMG_NAME)) < 0) {
    printf("Failed to open image\n");
    exit(1);
  }

  printf("image file descriptor = %d\n", img_fd);

  
  printf("About to enable video mode\n");
  
  bytes_read = read(img_fd, img_buffer, IMG_SIZE);

  if(bytes_read < 0) {
    printf("Read returned %d\n", bytes_read);
    exit(1);
  }

  if(bytes_read != IMG_SIZE) {
    printf("Read did not return the entire image\n");
    exit(1);
  }

  printf("Read %d bytes\n", bytes_read);
  if(mjpeg_to_rgb(img_buffer, bytes_read, &width, &height, &row_stride, &pixel_size,
                  &rgb_buf, &rgb_size) < 0) {
    printf("Failed to decompress image\n");
    exit(1);
  }

  printf("Converted img\n");
  
  if(enable_video(1, &video_memory) < 0) {
    printf("Failed to enable video mode\n");
    exit(1);
  }

  for(i = 0; i < 256; ++i) {
    printf("i = %d\n", i);
    memset(video_memory, i, 320*200);
    usleep(10000);
  }

  printf("At end of img_mjpeg_dec\n");
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
