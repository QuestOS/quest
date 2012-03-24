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

/* USB Hub driver */
#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <util/printf.h>
#include <kernel.h>
#include <sched/sched.h>

#define USB_HUB_CLASS 0x9

#define DEBUG_USB_HUB

#ifdef DEBUG_USB_HUB
#define DLOG(fmt,...) DLOG_PREFIX("usb-hub",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

bool uhci_enumerate (void);

#define HUB_PORT_STAT_POWER 0x0100
#define HUB_PORT_RESET 4
#define HUB_PORT_POWER 8
#define HUB_PORT_C_RESET 20

struct usb_hub_desc
{
  uint8 bDescLength;
  uint8 bDescriptorType;
  uint8 bNbrPorts;
  union {
    uint16 wHubCharacteristics;
    struct {
      uint16 lpsMode:2;         /* logical power switching */
      uint16 compound:1;        /* identifies compound device */
      uint16 opMode:2;          /* over-current protection */
      uint16 _reserved:11;
    };
  };
  uint8 bPwrOn2PwrGood;         /* (in 2ms intervals) */
  uint8 bHubContrCurrent;       /* max power requirement in mA */
  /* followed by DeviceRemovable / PortPwrCtrlMask variable-length fields */
} PACKED;
typedef struct usb_hub_desc USB_HUB_DESC;

static uint16
hub_port_status (uint address, uint port)
{
  sint status;
  USB_DEV_REQ req;
  uint8 data[4];

  req.bmRequestType = 0xA3;
  req.bRequest = USB_GET_STATUS;
  req.wValue = 0;
  req.wIndex = port;
  req.wLength = 4;
  /* We assume this is a full speed device, use the maximum, 64 bytes */
  status = uhci_control_transfer (address, &req, sizeof (req), data, 4, 64);
  DLOG ("GET_PORT_STATUS: status=%d port status: %.04X",
        status, *((uint16 *)data));

  return *((uint16 *)data);
}

static bool
hub_set_port_feature (uint address, uint port, uint feature)
{
  sint status;
  USB_DEV_REQ req;

  req.bmRequestType = 0x23;
  req.bRequest = USB_SET_FEATURE;
  req.wValue = feature;
  req.wIndex = port;
  req.wLength = 0;
  /* We assume this is a full speed device, use the maximum, 64 bytes */
  status = uhci_control_transfer (address, &req, sizeof (req), NULL, 0, 64);
  DLOG ("SET_PORT_FEATURE: status=%d", status);

  return status == 0;
}

static bool
hub_clr_port_feature (uint address, uint port, uint feature)
{
  sint status;
  USB_DEV_REQ req;

  req.bmRequestType = 0x23;
  req.bRequest = USB_CLEAR_FEATURE;
  req.wValue = feature;
  req.wIndex = port;
  req.wLength = 0;
  /* We assume this is a full speed device, use the maximum, 64 bytes */
  status = uhci_control_transfer (address, &req, sizeof (req), NULL, 0, 64);
  DLOG ("CLEAR_PORT_FEATURE: status=%d", status);

  return status == 0;
}

static bool
probe_hub (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  sint status, i, address = info->address;
  USB_DEV_REQ req;
  USB_HUB_DESC hubd;

  if (ifd->bInterfaceClass != USB_HUB_CLASS)
    return FALSE;

  /* it's a hub, set the configuration */
  usb_set_configuration (info, cfgd->bConfigurationValue);

  memset (&hubd, 0, sizeof (hubd));

  DLOG ("Probing hub @ %d", address);
  delay (100);
  req.bmRequestType = 0xA0;
  req.bRequest = USB_GET_DESCRIPTOR;
  req.wValue = 0x29 << 8;
  req.wIndex = 0;
  req.wLength = sizeof (USB_HUB_DESC);
  /* We assume this is a full speed device, use the maximum, 64 bytes */
  status = uhci_control_transfer (address, &req, sizeof (req), &hubd, sizeof (hubd), 64);
  DLOG ("GET_HUB_DESCRIPTOR: status=%d len=%d nbrports=%d delay=%d",
        status, hubd.bDescLength, hubd.bNbrPorts, hubd.bPwrOn2PwrGood);
  if (status != 0) return FALSE;
  for (i=1; i<=hubd.bNbrPorts; i++) {
    /* power-on port if necessary */
    while (!((status=hub_port_status (address, i)) & HUB_PORT_STAT_POWER)) {
      hub_set_port_feature (address, i, HUB_PORT_POWER);
      delay (2*hubd.bPwrOn2PwrGood);
    }
    if (status & 1) {
      /* potential device on port i */
      hub_set_port_feature (address, i, HUB_PORT_RESET);
      delay (10);
      hub_port_status (address, i);
      hub_clr_port_feature (address, i, HUB_PORT_C_RESET);
      hub_port_status (address, i);
      delay (2*hubd.bPwrOn2PwrGood);
      usb_enumerate(info->hcd);
    }
  }
  return TRUE;
}

static USB_DRIVER hub_driver = {
  .probe = probe_hub
};

extern bool
usb_hub_driver_init (void)
{
  return usb_register_driver (&hub_driver);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_hub_driver_init
};

DEF_MODULE (usb___hub, "USB hub driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
