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

/* The Realtek 8187B is a common USB Wi-Fi chipset */

#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_RTL8187B

#ifdef DEBUG_RTL8187B
#define DLOG(fmt,...) DLOG_PREFIX("rtl8187b",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static struct { uint16 v, p; char *s; } compat_list[] = {
  { 0x0846, 0x4260, "Netgear WG111v3 [RTL8187B]" },
  { 0xFFFF, 0xFFFF, NULL }
};

static bool
probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  int i;
  for (i=0; compat_list[i].v != 0xFFFF; i++) {
    if (compat_list[i].v == info->devd.idVendor &&
        compat_list[i].p == info->devd.idProduct)
      break;
  }
  if (compat_list[i].v == 0xFFFF)
    return FALSE;
  DLOG ("Found USB device address=%d: %s", info->address, compat_list[i].s);

  if (usb_set_configuration (info, cfgd->bConfigurationValue) != 0) {
    DLOG ("set_configuration: failed");
    return FALSE;
  }

  return TRUE;
}

static USB_DRIVER rtl8187b_usb_driver = {
  .probe = probe
};

extern bool
usb_rtl8187b_driver_init (void)
{
  return usb_register_driver (&rtl8187b_usb_driver);
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
