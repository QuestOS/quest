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

#ifndef _USER_USB_H_
#define _USER_USB_H_

#define USB_USER_READ  0
#define USB_USER_WRITE 1
#define USB_USER_OPEN  2
#define USB_USER_CLOSE 3


static inline int
usb_syscall(int device_id, int operation, void* buf, int data_len)
{
  fprintf(stderr, "usb_syscall not supported on arm");
  return -1;
}

static inline int usb_read(int device_id, void* buf, int data_len)
{
  return usb_syscall(device_id, USB_USER_READ, buf, data_len);
}


static inline int usb_write(int device_id, void* buf, int data_len)
{
  return usb_syscall(device_id, USB_USER_WRITE, buf, data_len);
}

static inline int usb_open(int device_id, void* buf, int data_len)
{
  return usb_syscall(device_id, USB_USER_OPEN, buf, data_len);
}

static inline int usb_close(int device_id)
{
  return usb_syscall(device_id, USB_USER_CLOSE, 0, 0);
}

#endif //_USER_USB_H
