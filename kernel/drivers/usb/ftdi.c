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

#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <drivers/usb/ftdi.h>
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_FTDI

#ifdef DEBUG_FTDI
#define DLOG(fmt,...) DLOG_PREFIX("ftdi",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

bool usb_ftdi_driver_init (void);

static bool
ftdi_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  // TODO
  return FALSE;
}

static USB_DRIVER ftdi_driver = {
  .probe = ftdi_probe
};

bool
usb_ftdi_driver_init (void)
{
  return usb_register_driver (&ftdi_driver);
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
