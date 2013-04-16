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

#define BUFFER_SIZE 6

#define BUTTONS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define THROTTLE 3
#define SECONDARY_JOYSTICK 5
#define NEWLINES_AT_END 20

int main()
{
  int joystick_fd = usb_open("joystick0");
  int bytes_read;
  unsigned char buffer[BUFFER_SIZE];
  char newline_buffer[NEWLINES_AT_END];
  int i;
  
  if(joystick_fd < 0) {
    printf("No joystick found\n");
    return 0;
  }

  while(1) {
    int buttons_pressed = 0;
    bytes_read = usb_read(joystick_fd, buffer, BUFFER_SIZE);
    if(bytes_read < 0) {
      printf("usb_read returned %d\n", bytes_read);
    }
    else if(bytes_read > 0) {
      printf("(%d, %d, %d, %d, %d, %d)\n", buffer[0], buffer[1],
             buffer[2], buffer[3], buffer[4], buffer[5]);
      
      printf("(");
      if(buffer[X_AXIS] == 128) {
        printf("Neutral");
      }
      else if(buffer[X_AXIS] < 128) {
        printf("Left");
      }
      else {
        printf("Right");
      }

      printf(", ");

      if(buffer[Y_AXIS] == 128) {
        printf("Neutral");
      }
      else if(buffer[Y_AXIS] < 128) {
        printf("Down");
      }
      else {
        printf("Up");
      }
      printf(", ");

      if(buffer[Z_AXIS] == 128) {
        printf("Netural");
      }
      else if(buffer[Z_AXIS] < 128) {
        printf("Rotate Left");
      }
      else {
        printf("Rotate Right");
      }
      printf(")\n");

      printf("Throttle = %.2f%%\n", buffer[THROTTLE] / 2.55);

      for(i = 0; i < 8; ++i) {
        if(buffer[BUTTONS] & (1 << i)) {
          printf("Button %d Pressed\n", i+1);
        }
        buttons_pressed++;
      }

      switch(buffer[SECONDARY_JOYSTICK]) {
      case 0:
        printf("Up\n");
        break;
      case 16:
        printf("Up-Right\n");
        break;
      case 32:
        printf("Right\n");
        break;
      case 48:
        printf("Down-Right\n");
        break;
      case 64:
        printf("Down\n");
        break;
      case 80:
        printf("Down-Left\n");
        break;
      case 96:
        printf("Left\n");
        break;
      case 112:
        printf("Up-Left\n");
        break;
      case 128:
        printf("Center\n");
        break;
      }

      memset(newline_buffer, 0, NEWLINES_AT_END);
      memset(newline_buffer, '\n', NEWLINES_AT_END-buttons_pressed);
      printf("%s", newline_buffer);
    }
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
