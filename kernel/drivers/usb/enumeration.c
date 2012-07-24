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

#include "kernel.h"
#include "drivers/usb/usb.h"
#include "module/header.h"
#include <util/printf.h>

#define DEBUG_USB_ENUM


#ifdef DEBUG_USB_ENUM
#define DLOG(fmt,...) DLOG_PREFIX("USB ENUM",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


/* Enumerate the USB bus */
bool
usbenumeration_init (void)
{
  // Turn UHCI enumeration off
  //extern bool usb_do_enumeration (void);
  //if(!usb_do_enumeration()) return FALSE;

  uint32_t index = 0;
  usb_hcd_t* hcd = NULL;

  DLOG("IN USB ENUM");

  while( (hcd = get_usb_hcd(index)) != NULL) {
    index++;
    if(!hcd->reset_root_ports(hcd)) {
      DLOG("Failed to reset root ports for device %d", index-1);
      panic("Failed to reset root ports for device");
      continue;
    }
    if(!usb_enumerate(hcd))  {
      DLOG("Failed to enumerate device %d", index-1);
      continue;
    }
    
    if(!hcd->post_enumeration(hcd)) {
      DLOG("post enumeration failed for device %d", index-1);
      continue;
    }
  }

  DLOG("Number of USB HCD enumerated %d", index);
  
  return TRUE;
}

static const struct module_ops mod_ops = {
  .init = usbenumeration_init
};

DEF_MODULE (usbenumeration, "USB enumeration", &mod_ops, {"usb___"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
