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

#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <drivers/net/ethernet.h>
#include <util/printf.h>
#include <kernel.h>
#include "sched/sched.h"

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

static inline uint8
hex2byte (uint8 h[2])
{
  return
    (('0' <= h[0] && h[0] <= '9') ? h[0] - '0' : h[0] - 'A' + 10) * 16 +
    (('0' <= h[1] && h[1] <= '9') ? h[1] - '0' : h[1] - 'A' + 10);
}

static uint8 ethaddr[6];
static uint status_ept, status_maxpkt;
static uint data_ept_in, data_ept_out, data_maxpkt;
static uint data_iface, data_nop_alt, data_real_alt;
static USB_DEVICE_INFO *ethusbdev;
static ethernet_device usbnet_ethdev;

static bool
reset (void)
{
  if (usb_set_interface (ethusbdev, data_nop_alt, data_iface) != 0) {
    DLOG ("set_interface -> nop data: failed");
    return FALSE;
  }

  if (usb_set_interface (ethusbdev, data_real_alt, data_iface) != 0) {
    DLOG ("set_interface -> real data: failed");
    return FALSE;
  }

  return TRUE;
}

static bool
get_hwaddr (uint8 addr[ETH_ADDR_LEN])
{
  int i;
  for (i=0; i<ETH_ADDR_LEN; i++)
    addr[i] = ethaddr[i];
  return TRUE;
}

static sint
transmit (uint8* buf, sint len)
{
  sint ret;
  uint32 act_len;

  DLOG ("transmitting data len=%d: %.02X %.02X %.02X %.02X", len,
        buf[0], buf[1], buf[2], buf[3]);
  if (usb_bulk_transfer (ethusbdev, data_ept_out, buf, len,
                         data_maxpkt, USB_DIR_OUT, &act_len) == 0)
    ret = len;
  else
    ret = 0;

  return ret;
}

static void
poll (void)
{
  uint8 buffer[1600];
  uint32 act_len;
  if (usb_bulk_transfer (ethusbdev, data_ept_in, buffer, 1514,
                         data_maxpkt, USB_DIR_IN, &act_len) == 0) {
    if (act_len > 0) {
      DLOG ("receiving data len=%d: %.02X %.02X %.02X %.02X",
            act_len, buffer[0], buffer[1], buffer[2], buffer[3]);
      usbnet_ethdev.recv_func (&usbnet_ethdev, buffer, act_len);
    }
  }
}

static void
irq_loop (void)
{
  uint32 tick = 0;
  DLOG ("irq_loop pid=0x%x", str ());
  for (;;) {
    poll ();
    DLOG ("iteration %d", tick);
    tick++;
  }
}

static uint32 irq_stack[1024] ALIGNED(0x1000);
static task_id irq_pid;

static bool
probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  uint i;
  uint8 addrstr[13] = { [12] = 0 };
  uint8 strbuf[26];
  CDC_FUNDESC *fund = (CDC_FUNDESC *)&ifd[1];
  CDC_ETHERNET_FUNDESC *ethd;
  USB_EPT_DESC *eptd;

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
        addrstr[i] = strbuf[2*i+2];
      for (i=0; i<6; i++)
        ethaddr[i] = hex2byte (&addrstr[2*i]);
      DLOG ("ethernet string=%s address=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            addrstr, ethaddr[0], ethaddr[1], ethaddr[2],
            ethaddr[3], ethaddr[4], ethaddr[5]);
      break;
    }
    fund = (CDC_FUNDESC *)((uint8 *)fund + fund->bFunctionLength);
  }

  /* expect fund to point to an endpoint (Status) */
  if (fund->bDescriptorType != USB_TYPE_EPT_DESC) {
    DLOG ("did not find Status endpoint: bDescriptorType=0x%x",
          fund->bDescriptorType);
    return FALSE;
  }

  eptd = (USB_EPT_DESC *)fund;
  status_ept = eptd->bEndpointAddress & 0x7F;
  status_maxpkt = eptd->wMaxPacketSize;

  /* expect CDC Data (nop) to follow */
  ifd = (USB_IF_DESC *) &eptd[1];

  if (ifd->bDescriptorType != USB_TYPE_IF_DESC) {
    DLOG ("did not find CDC Data (nop): bDescriptorType=0x%x",
          ifd->bDescriptorType);
    return FALSE;
  }

  data_iface = ifd->bInterfaceNumber;
  data_nop_alt = ifd->bAlternateSetting;

  /* then CDC Data (actual) */
  ifd = &ifd[1];

  if (ifd->bDescriptorType != USB_TYPE_IF_DESC) {
    DLOG ("did not find CDC Data: bDescriptorType=0x%x",
          ifd->bDescriptorType);
    return FALSE;
  }

  data_real_alt = ifd->bAlternateSetting;

  DLOG ("CDC Data (nop) iface#=%d alt=%d.  CDC Data alt=%d",
        data_iface, data_nop_alt, data_real_alt);

  /* now two endpoints for data */
  eptd = (USB_EPT_DESC *) &ifd[1];
  data_maxpkt = eptd->wMaxPacketSize;
  if (eptd->bEndpointAddress & 0x80)
    data_ept_in = eptd->bEndpointAddress & 0x7F;
  else
    data_ept_out = eptd->bEndpointAddress & 0x7F;

  eptd = &eptd[1];
  if (data_maxpkt != eptd->wMaxPacketSize) {
    DLOG ("cannot handle: in/out packet sizes differ");
    return FALSE;
  }
  if (eptd->bEndpointAddress & 0x80)
    data_ept_in = eptd->bEndpointAddress & 0x7F;
  else
    data_ept_out = eptd->bEndpointAddress & 0x7F;

  DLOG ("data_ept_in=%d data_ept_out=%d data_maxpkt=%d",
        data_ept_in, data_ept_out, data_maxpkt);
  if (usb_set_configuration (info, cfgd->bConfigurationValue) != 0) {
    DLOG ("set_configuration: failed");
    return FALSE;
  }

  ethusbdev = info;

  if (!reset ())
    return FALSE;

  /* Register network device with net subsystem */
  usbnet_ethdev.recv_func = NULL;
  usbnet_ethdev.send_func = transmit;
  usbnet_ethdev.get_hwaddr_func = get_hwaddr;
  usbnet_ethdev.poll_func = poll;

  if (!net_register_device (&usbnet_ethdev)) {
    DLOG ("registration failed");
    return FALSE;
  }

  irq_pid = start_kernel_thread ((uint) irq_loop, (uint) &irq_stack[1023], "USB Net");

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

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_net_driver_init
};

//DEF_MODULE (usb___net, "USB net driver", &mod_ops, {"usb", "net___ethernet"});


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
