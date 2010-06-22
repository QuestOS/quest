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
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_USB_NET

#ifdef DEBUG_USB_NET
#define DLOG(fmt,...) DLOG_PREFIX("USB-net",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define CDC_TYPE_CS_INTERFACE 0x24

#define CDC_SUBTYPE_HEADER 0x00
#define CDC_SUBTYPE_UNION  0x06
#define CDC_SUBTYPE_ETHERNET 0x0F


struct cdc_header_fundesc {
  uint8 bFunctionLength;
  uint8 bDescriptorType;
  uint8 bDescriptorSubtype;
  uint16 bcdCDC;
} PACKED;
typedef struct cdc_header_fundesc CDC_HEADER_FUNDESC;

struct cdc_union_fundesc {
  uint8 bFunctionLength;
  uint8 bDescriptorType;
  uint8 bDescriptorSubtype;
  uint8 bControlInterface;
  uint8 bSubordinateInterface[];
} PACKED;
typedef struct cdc_union_fundesc CDC_UNION_FUNDESC;

struct cdc_ethernet_fundesc {
  uint8 bFunctionLength;
  uint8 bDescriptorType;
  uint8 bDescriptorSubtype;
  uint8 iMACAddress;
  uint32 bmEthernetStatistics;
  uint16 wMaxSegmentSize;
  uint16 wNumberMCFilters;
  uint8 bNumberPowerFilters;
} PACKED;
typedef struct cdc_ethernet_fundesc CDC_ETHERNET_FUNDESC;

typedef union {
  struct {
    uint8 bFunctionLength;
    uint8 bDescriptorType;
    uint8 bDescriptorSubtype;
  };
  CDC_HEADER_FUNDESC   header;
  CDC_UNION_FUNDESC    cdc_union;
  CDC_ETHERNET_FUNDESC ethernet;
} CDC_FUNDESC;

static bool
probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  uint i;
  uint8 ethaddr[13] = { [12] = 0 };
  uint8 strbuf[26];
  CDC_FUNDESC *fund = (CDC_FUNDESC *)&ifd[1];
  CDC_ETHERNET_FUNDESC *ethd;

  if (!(ifd->bInterfaceClass == 0x02 &&
        ifd->bInterfaceSubClass == 0x06 &&
        ifd->bInterfaceProtocol == 0x00))
    return FALSE;

  while (fund->bDescriptorType == CDC_TYPE_CS_INTERFACE) {
    DLOG ("found functional desc. subtype=0x%x", fund->bDescriptorSubtype);
    switch (fund->bDescriptorSubtype) {
    case CDC_SUBTYPE_ETHERNET:
      ethd = &fund->ethernet;
      DLOG ("eth iMAC=%d mss=%d", ethd->iMACAddress, ethd->wMaxSegmentSize);
      if (usb_get_descriptor (info, USB_TYPE_STR_DESC, ethd->iMACAddress, 0,
                              sizeof (strbuf), strbuf) != 0) {
        DLOG ("unable to get ethernet address");
        return FALSE;
      }
      /* seems to be UTF-16 encoded */
      for (i=0; i<12; i++)
        ethaddr[i] = strbuf[2*i+2];
      DLOG ("ethernet address=%s", ethaddr);
      break;
    }
    fund = (CDC_FUNDESC *)((uint8 *)fund + fund->bFunctionLength);
  }

  usb_set_configuration (info, cfgd->bConfigurationValue);

  return TRUE;
}

static USB_DRIVER net_driver = {
  .probe = probe
};

extern bool
usb_net_driver_init (void)
{
  return usb_register_driver (&net_driver);
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
