/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#ifndef _FTDI_H_
#define _FTDI_H_

#include <types.h>

/* FTDI Commands */
#define USB_FTDI_RESET                  0x00
#define USB_FTDI_MODEM_CTRL             0x01
#define USB_FTDI_SET_FLOW_CTRL          0x02
#define USB_FTDI_SET_BAUD_RATE          0x03
#define USB_FTDI_SET_DATA               0x04
#define USB_FTDI_GET_MODEM_STATUS       0x05
#define USB_FTDI_SET_EVENT_CHAR         0x06
#define USB_FTDI_SET_ERROR_CHAR         0x07
#define USB_FTDI_SET_LATENCY_TIMER      0x09
#define USB_FTDI_GET_LATENCY_TIMER      0x0A

extern bool usb_ftdi_driver_init (void);
extern void usb_ftdi_putc (char);

#endif

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
