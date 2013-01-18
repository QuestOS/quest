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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <usb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

//#define HOST_DEVICE_NUM 1
#define HOST_DEVICE_NUM 2

int main()
{
  int res;
  char buf[1024];
  char rw_buf1[1024];
  char rw_buf2[1024];
  printf("usb_gadget_test started\n");
  
  if(usb_open(0, NULL, 0) < 0) {
    printf("Failed to open gadget\n");
    while(1);
  }

  printf("Gadget opened\n");

  if(usb_open(HOST_DEVICE_NUM, buf, 1024) < 0) {
    printf("Failed to open device %d\n", HOST_DEVICE_NUM);
    while(1);
  }

  printf("Device %d opened\n", HOST_DEVICE_NUM);

  memcpy(rw_buf1, "Hello from " __FILE__, 40);

  if(HOST_DEVICE_NUM == 1) {
    res = usb_write(HOST_DEVICE_NUM, rw_buf1, 1024);
    if(res < 0) {
      printf("Failed to write to device %d\n", HOST_DEVICE_NUM);
      while(1);
    }

    printf("usb write on %d returned %d\n", HOST_DEVICE_NUM, res);

    usleep(1000000);

    res = usb_read(0, rw_buf2, 512);
  }
  else {
    res = usb_write(0, rw_buf1, 1024);
    if(res < 0) {
      printf("Failed to write to device 0\n");
      while(1);
    }

    printf("usb write on 0 returned %d\n", res);
    
    usleep(1000000);
    
   res = usb_read (HOST_DEVICE_NUM, rw_buf2, 512);
  }

  printf("usb read returned %d\n", res);

  rw_buf2[30] = 0;

  printf("%s\n", rw_buf2);

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
