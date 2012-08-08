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
#include <usb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define ARENA_START (0x400000 * 10)

int
main ()
{
  int i = 0;
  int result;
  void* arena = ARENA_START;
  char* picture_arena = ARENA_START + (0x400000 * 4);
  struct sockaddr_in name, cli_name;
  int sd;
  int image_len;
  int port = 54000;
  int slen = sizeof(cli_name);
  /*
  if((sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    printf("socket() error!\n");
    exit(-1);
  }
  
  
  bzero(&name, sizeof(name));
  name.sin_family = AF_INET;
  name.sin_port = htons(port);
  name.sin_addr.s_addr = htonl(INADDR_ANY);
  if(bind(sd, (struct sockaddr *)&name, sizeof(name)) < 0){
    printf("bind() fails!\n");
    exit(-1);
  }
  */
  printf("Entering usb test program\n");
  
  if(usb_open(0, arena, 0x400000 * 4) < 0) {
    printf("Call to open failed");
    while(1);
  }
  
  //printf("About to start reading frames");
  
  
  while(1) {
    *picture_arena = 0;
    if(i > 50) {
      //*picture_arena = 1;
    }
    result = usb_read(0, picture_arena, 0x400000);
    if(result != 0) {
      printf("usb read returned %d\n", result);
      printf("%d\n", i++);
    }
  }


  printf("leaving usb test program");
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
