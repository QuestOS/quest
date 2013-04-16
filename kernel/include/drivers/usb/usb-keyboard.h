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


#ifndef _USB_KEYBOARD_H_
#define _USB_KEYBOARD_H_

#define USB_KEYBOARD_BUFFER_SIZE 64


typedef struct _usb_keyboard_dev{
  struct urb* urb;
  USB_DEVICE_INFO *dev;
  USB_EPT_DESC int_ep;
  char buffer[USB_KEYBOARD_BUFFER_SIZE];
  uint8 old[8];

} usb_keyboard_dev_t;

void init_usb_keyboard_dev(usb_keyboard_dev_t* dev)
{
  memset(dev, 0, sizeof(usb_keyboard_dev_t));
}


#endif // _USB_KEYBOARD_H_
