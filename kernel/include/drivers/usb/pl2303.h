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

#ifndef _PL2303_H_
#define _PL2303_H_

#include <types.h>

#define USB_PL2303_SET_LINE                0x20
#define USB_PL2303_SET_CONTROL             0x22
#define USB_PL2303_CONTROL_DTR             0x01
#define USB_PL2303_CONTROL_RTS             0x02
#define USB_PL2303_BREAK                   0x23
#define USB_PL2303_BREAK_ON                0xFFFF
#define USB_PL2303_BREAK_OFF               0x0000
#define USB_PL2303_GET_LINE                0x21

/* PL2303 configuration structure */
struct pl2303_config
{
  uint32_t baud_rate;
  uint8_t stop_bits; /* Stop Bits, 0 = 1, 1 = 1.5, 2 = 2 */
  uint8_t parity; /* 0 = none, 1 = odd, 2 = even, 3 = mark, 4 = space */
  uint8_t data_bits; /* Data Bits, 5, 6, 7 or 8 */
} __attribute__ ((packed));

typedef struct pl2303_config PL2303_CONFIG;
extern bool pl2303_initialized;

extern char usb_pl2303_getc (void);
extern void usb_pl2303_putc (char);

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
