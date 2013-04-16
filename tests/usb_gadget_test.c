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



#define BUFFER_SIZE 4096

char rw_buf1[BUFFER_SIZE];
char rw_buf2[BUFFER_SIZE];

int main()
{
  int res;
  int beagle_fd_out, beagle_fd_in, net2280_fd;
  printf("usb_gadget_test started\n");
  
 retry_net2280_open:
  if((net2280_fd = usb_open("net2280_communication0")) < 0) {
    printf("Failed to open net2280 device\n");
    usleep(1000000);
    goto retry_net2280_open;
  }  
  printf("Gadget opened net2280_fd = %d\n", net2280_fd);

 retry_beagle_open:
  if((beagle_fd_in = usb_open("beagle_communication1")) < 0) {
    printf("Failed to open device beagle_communication1\n");
    usleep(1000000);
    goto retry_beagle_open;
  }

  if((beagle_fd_out = usb_open("beagle_communication0")) < 0) {
    printf("Failed to open device beagle_communication0\n");
    while(1);
  }

  printf("beagle_communication0 device opened, fd = %d\n", beagle_fd_out);
  printf("beagle_communication1 device opened, fd = %d\n", beagle_fd_in);

  memcpy(rw_buf1, "Hello from " __FILE__, 40);

  int i;
#define INCREMENTS 1
  for(i = 0; i < INCREMENTS; ++i) {
    res = usb_write(beagle_fd_out, rw_buf1, BUFFER_SIZE / INCREMENTS);
    usleep(1000000);
    if(res < 0) {
      printf("Failed to write to device %d\n", beagle_fd_out);
      while(1);
    }
    
    printf("usb write on %d returned %d\n", beagle_fd_out, res);
  }
  
  usleep(2000000);
  res = usb_read(net2280_fd, rw_buf2, BUFFER_SIZE);
  printf("usb read on net2280_fd returned %d\n", res);


  for(i = 0; i < INCREMENTS; ++i) {
    res = usb_write(net2280_fd, rw_buf1, BUFFER_SIZE / INCREMENTS);
    usleep(1000000);
    if(res < 0) {
      printf("Failed to write to device %d\n", net2280_fd);
      while(1);
    }
    
    printf("usb write on %d returned %d\n", net2280_fd, res);
  }
  
  usleep(2000000);
  res = usb_read(beagle_fd_in, rw_buf2, BUFFER_SIZE);
  printf("usb read on beagle_fd_in returned %d\n", res);


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
