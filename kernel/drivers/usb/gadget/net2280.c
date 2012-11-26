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


#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <util/printf.h>
#include <kernel.h>
#include <drivers/pci/pci.h>


#define DEBUG_NET2280
#define DEBUG_NET2280_VERBOSE

#ifdef DEBUG_NET2280
#define DLOG(fmt,...) DLOG_PREFIX("Net2280",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_NET2280_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("Net2280",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif


bool
initialise_net2280(void)
{
  uint32_t pci_base = 0;
  uint i, device_index, irq_pin;
  pci_device net2280_device;
  pci_irq_t irq;
  DLOG("In net2280");

    if(mp_ISA_PC) {
    DLOG("Cannot operate without PCI");
    DLOGV("Exiting %s with FALSE", __FUNCTION__);
    return FALSE;
  }
   
  /* Find the Net2280 device on the PCI bus */
  device_index = ~0;
  i = 0;
  
  while (pci_find_device (0x17cc, 0x2280, 0xFF, 0xFF, i, &i)) {
    if (pci_get_device (i, &net2280_device)) {
      device_index = i;
      break;
    }
    else {
      break;
    }
  }

  DLOG("device_index = 0X%X", device_index);
  
  while(1);
}


#include "module/header.h"

static const struct module_ops mod_ops = {
.init = initialise_net2280
};

DEF_MODULE (usb___gadget_net2280, "NET2280 driver", &mod_ops, {"usb", "pci"});



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
