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

#define ARENA_START_PAGE 10
#define ARENA_PAGE_SIZE 40

#define ARENA_START (0x400000 * ARENA_START_PAGE)
//#define NUM_DEVICES 19
#define NUM_DEVICES 11

#define DEVICE_MEMORY_SIZE (0x400000 * 2)


int
main ()
{
  int i;
  int dev_num;
  int result;
  char* arena = ARENA_START;
  char* read_arena = ARENA_START + (0x400000 * (ARENA_PAGE_SIZE - 2));
  char* device_memory[NUM_DEVICES];
  int device_is_input_map[20];

  device_is_input_map[0]  = 1;
  device_is_input_map[1]  = 1;
  device_is_input_map[2]  = 1;
  device_is_input_map[3]  = 1;
  device_is_input_map[4]  = 1;
  device_is_input_map[5]  = 1;
  device_is_input_map[6]  = 1;
  device_is_input_map[7]  = 1;
  device_is_input_map[8]  = 1;
  device_is_input_map[9]  = 1;
  device_is_input_map[10] = 1;
  device_is_input_map[11] = 1;
  device_is_input_map[12] = 1;
  device_is_input_map[13] = 1;
  device_is_input_map[14] = 1;
  device_is_input_map[15] = 1;
  device_is_input_map[16] = 1;
  device_is_input_map[17] = 1;
  device_is_input_map[18] = 1;
  device_is_input_map[19] = 1;
  
  for(i = 0; i < NUM_DEVICES; ++i) {
    device_memory[i] = (i * DEVICE_MEMORY_SIZE) + ARENA_START;
  }
  
  for(i = 0; i < (40 * 0x400000); ++i) {
    arena[i] = 'a';
    if(arena[i] != 'a') {
      printf("Couldn't set memory area");
      exit(1);
    }
  }
  
  printf("Entering usb test program\n");

  for(i = 0; i < NUM_DEVICES; ++i) {
    while(1) {
      int res;
      if((res = usb_open(i, device_memory[i], DEVICE_MEMORY_SIZE)) < 0) {
        if(res == -2) { continue; }
        printf("Call to open failed");
        while(1);
      }
      break;
    }
  }
  
  //usleep(2000000);
  
  printf("About to start reading frames");
  
  dev_num = 0;
  i = 0;
  while(1) {
    
    if(device_is_input_map[dev_num]) {
      result = usb_read(dev_num, read_arena, 0x400000);
      printf("device %d: %d: usb read returned %d\n", dev_num, i, result);
    }
    else {
      result = usb_write(dev_num, read_arena, 0x400000);
      printf("device %d: %d: usb write returned %d\n", dev_num, i, result);
    }
    
    ++i; ++dev_num;
    
    if(dev_num == NUM_DEVICES) {
      dev_num = 0;
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
