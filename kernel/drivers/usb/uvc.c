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
#include <drivers/usb/uvc.h>
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_UVC

#ifdef DEBUG_UVC
#define DLOG(fmt,...) DLOG_PREFIX("UVC",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

int
uvc_read (void *buf, int count)
{
  int status = 0;
  uint8_t iso[1024];

#if 0
  memset (conf, 0, 1300);

  int status =
    uhci_get_descriptor (1, TYPE_CFG_DESC, 0, 0, 1000, (addr_t) conf);
  DLOG ("Status Code: 0x%.08x", status);

#endif

#if 1
  status = uhci_isochronous_transfer (1, 5, (addr_t) iso, 1023, 1, DIR_IN, 0);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso2, 1023, 10, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso3, 1023, 20, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso4, 1023, 30, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso5, 1023, 40, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso6, 1023, 50, DIR_IN);
#endif

  delay (1000);
  delay (1000);

#if 0
  int i = 0;

  for (i = 1; i < 256; i++) {
    putx ((uint32_t) ((uint32_t *) iso + i));
    if (i % 5 == 0)
      putchar ('\n');
  }
#endif
  return status;

}

int
uvc_init (void)
{
#if 0
  uint8_t data[20];
  uint8_t conf[1300];

  USB_DEV_DESC *desc;
  USB_CFG_DESC *cfgd;
  UVC_IA_DESC *iad;
  USB_IF_DESC *vcifd;
  UVC_CSVC_IF_HDR_DESC *csvcifd;

  port_reset (0);
  port_reset (1);

  memset (data, 0, 20);

  uhci_get_descriptor (0, TYPE_DEV_DESC, 0, 0, 18, (addr_t) data);
  DLOG ("After get descriptor!");
  while (1);

  desc = (USB_DEV_DESC *) data;

  port_reset (0);
  port_reset (1);

  uhci_set_address (0, 1);

  memset (data, 0, 20);

  uhci_get_descriptor (1, TYPE_DEV_DESC, 0, 0, 18, (addr_t) data);
  desc = (USB_DEV_DESC *) data;

#if 0
  DLOG ("dev desc: len=%d type=%x class=%x subclass=%x proto=%x ven=%x numcfg=%d",
        desc->bLength,
        desc->bDescriptorType,
        desc->bDeviceClass,
        desc->bDeviceSubClass,
        desc->bDeviceProtocol,
        desc->idVendor,
        desc->bNumConfigurations);
#endif

  memset (data, 0, 20);

  uhci_get_descriptor (1, TYPE_CFG_DESC, 0, 0, 9, (addr_t) data);
  cfgd = (USB_CFG_DESC *) data;

#if 0
  DLOG ("cfg desc: len=%d type=%x totlen=%d numI=%d cfgv=%x cfg=%x att=%x mp=%x",
        cfgd->bLength,
        cfgd->bDescriptorType,
        cfgd->wTotalLength,
        cfgd->bNumInterfaces,
        cfgd->bConfigurationValue,
        cfgd->iConfiguration,
        cfgd->bmAttributes,
        cfgd->MaxPower);
#endif

  memset (conf, 0, 1300);

  uhci_get_descriptor (1, TYPE_CFG_DESC, 0, 0, cfgd->wTotalLength,
                       (addr_t) conf);
  iad = (UVC_IA_DESC *) (&conf[cfgd->bLength]);
  vcifd = (USB_IF_DESC *) (&conf[cfgd->bLength + iad->bLength]);
  csvcifd =
    (UVC_CSVC_IF_HDR_DESC *)
    (&conf[cfgd->bLength + iad->bLength + vcifd->bLength]);

#if 0
  DLOG ("Class-Specific VC Interface Header Descriptor");
  DLOG ("  len=%d type=%x sub=%x UVC=%x totlen=%d clock=%x inCol=%x iface1=%x",
        csvcifd->bLength,
        csvcifd->bDescriptorType,
        csvcifd->bDescriptorSubType,
        csvcifd->bcdUVC,
        csvcifd->wTotalLength,
        csvcifd->dwClockFrequency,
        csvcifd->bInCollection,
        csvcifd->baInterface1);
#endif

  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength +
               csvcifd->bLength]);

#if 0
  DLOG ("Class-Specific VC Interface Header Descriptor");
  DLOG ("  len=%d type=%x sub=%x totlen=%d inCol=%x",
        csvcifd->bLength,
        csvcifd->bDescriptorType,
        csvcifd->bDescriptorSubType,
        csvcifd->wTotalLength,
        csvcifd->bInCollection);
#endif

  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength + 13 +
               csvcifd->bLength]);

#if 0
  DLOG ("Class-Specific VC Interface Header Descriptor");
  DLOG ("  len=%d type=%x sub=%x totlen=%d inCol=%x",
        csvcifd->bLength,
        csvcifd->bDescriptorType,
        csvcifd->bDescriptorSubType,
        csvcifd->wTotalLength,
        csvcifd->bInCollection);
#endif

  uhci_set_configuration (1, 1);
#endif
  return 0;
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
