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
#include <string.h>

#define ARENA_START_PAGE 10
#define ARENA_PAGE_SIZE 40

#define ARENA_START (0x400000 * ARENA_START_PAGE)

#define NUM_RTT_DEVICES (2)

#define DEVICE_MEMORY_SIZE (0x400000 * 2)
#define NUM_RTT_BEFORE_OPENING_OTHER_DEV 250
#define MAX_TRIP_TIMES 500      /* Make sure this matches with what is in gadget2.c */
#define NUM_NON_RRT_DEVICES 7


int
main ()
{
  int i, j;
  int dev_num;
  int result;
  char* arena = (char*)ARENA_START;
  char* read_arena = (char*)(ARENA_START + (0x400000 * (ARENA_PAGE_SIZE - 2)));
  char* device_memory[(NUM_RTT_DEVICES*2) + NUM_NON_RRT_DEVICES];
  int report_results = 0;
  int next_write;
  int num_rtt = 0;
  int successful_other_dev_num = 0;
  int reported_other_devs = 0;
  
  for(i = 0; i < (40 * 0x400000); ++i) {
    arena[i] = 'a';
    if(arena[i] != 'a') {
      printf("Couldn't set memory area");
      exit(1);
    }
  }

  for(i = 0; i < (NUM_RTT_DEVICES*2) + NUM_NON_RRT_DEVICES; ++i) {
    device_memory[i] = (char*)(i * DEVICE_MEMORY_SIZE) + ARENA_START;
  }
  
  printf("Entering usb test program\n");

  for(i = 0; i < NUM_RTT_DEVICES*2; ++i) {
    while(1) {
      
      int res;
      if((res = usb_open(i, device_memory[i], DEVICE_MEMORY_SIZE)) < 0) {
        if(res == -2) { continue; }
        printf("Call to open failed for device %d", i);
        while(1);
      }
      break;
    }
  }
  
  dev_num = 0;
  i = 0;
  next_write = 1;
  memset(read_arena, 'a', 512);
  result = usb_write(1, read_arena, 512);
  printf("Wrote %d bytes to first device\n", result);
  while(1) {

    result = usb_read(dev_num*2, read_arena, 0x400000);
    if(result > 0) {
      result = usb_write(1 + next_write * 2, read_arena, 512);
      next_write++;
      if(next_write == NUM_RTT_DEVICES) {
        next_write = 0;
        num_rtt++;
        if(num_rtt == NUM_RTT_BEFORE_OPENING_OTHER_DEV) {
          for(j = 0; j < NUM_NON_RRT_DEVICES; ++j) {
            while(1) {
              int res;
              int dev_num2 = (NUM_RTT_DEVICES * 2) + j;
              if((res = usb_open(dev_num2, device_memory[dev_num2], DEVICE_MEMORY_SIZE)) < 0) {
                if(res == -2) { continue; }
                printf("Call to open failed for device %d\n", dev_num2);
                break;
              }
              successful_other_dev_num++;
              printf("Call to open for device %d was successful\n", dev_num2);
              break;
            }
            
          }
        }
      }
    }
    ++i; ++dev_num;

    if(num_rtt >= NUM_RTT_BEFORE_OPENING_OTHER_DEV) {
      int report = 0;

      if(num_rtt == (MAX_TRIP_TIMES-1) && next_write == (NUM_RTT_DEVICES - 1)
         && !reported_other_devs) {
        report = 1;
        reported_other_devs = 1;
      }
      
      for(j = 0; j < successful_other_dev_num; ++j) {
        int dev_num2 = (NUM_RTT_DEVICES * 2) + j;
        if(report) {
          read_arena[0] = 'e';
        }
        result = usb_read(dev_num2, read_arena, 0x400000);
      }
    }
    
    if(dev_num == NUM_RTT_DEVICES) {
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
