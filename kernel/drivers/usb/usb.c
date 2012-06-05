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
#include <drivers/usb/ehci.h>
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>
#include "sched/sched.h"
#include <mem/pow2.h>

#define DEBUG_USB

#ifdef DEBUG_USB
#define DLOG(fmt,...) DLOG_PREFIX("USB",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define MAX_USB_HCD 10

#define USB_MAX_DEVICE_DRIVERS 32
static USB_DRIVER drivers[USB_MAX_DEVICE_DRIVERS];
static uint num_drivers = 0;

usb_hcd_t* usb_hcd_list[MAX_USB_HCD];
uint32_t usb_hcd_list_next = 0;


bool
initialise_usb_hcd(usb_hcd_t* usb_hcd, uint32_t usb_hc_type,
                   usb_reset_root_ports_func reset_root_ports,
                   usb_post_enumeration_func post_enumeration)
{
  usb_hcd->usb_hc_type = usb_hc_type;
  usb_hcd->reset_root_ports = reset_root_ports;
  usb_hcd->post_enumeration = post_enumeration;
  usb_hcd->next_address = 1;
  memset(usb_hcd->toggles, 0 , sizeof(uint32) * TOGGLE_SIZE);
  return TRUE;
}

bool
add_usb_hcd(usb_hcd_t* usb_hcd)
{
  if(usb_hcd_list_next < MAX_USB_HCD) {
    usb_hcd_list[usb_hcd_list_next++] = usb_hcd;
    return TRUE;
  }
  return FALSE;
}


usb_hcd_t*
get_usb_hcd(uint32_t index)
{
  DLOG("usb_hcd_list_next = %d in get call", usb_hcd_list_next);
  if(index < usb_hcd_list_next) return usb_hcd_list[index];
  return NULL;
}


int
usb_control_transfer(
    USB_DEVICE_INFO* dev,
    addr_t setup_req,
    uint16_t req_len,
    addr_t data,
    uint16_t data_len)
{
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      }
      else {
        return uhci_control_transfer(dev->address, setup_req,
                                     req_len, data, data_len,
                                     (dev->devd).bMaxPacketSize0);
      }
      
    case USB_TYPE_HC_EHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return ehci_control_transfer(hcd_to_ehci_hcd(dev->hcd),
                                     dev->address, setup_req,
                                     req_len, data, data_len,
                                     (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
}

int
usb_bulk_transfer(
    USB_DEVICE_INFO* dev,
    uint8_t endp,
    addr_t data,
    uint16_t len,
    uint8_t packet_len,
    uint8_t dir,
    uint32_t *act_len)
{
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_bulk_transfer(dev->address, endp, data,
                                  len, packet_len, dir, act_len);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_bulk_transfer");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
}

int
usb_get_descriptor(USB_DEVICE_INFO* dev,
                   uint16_t dtype,
                   uint16_t dindex,
                   uint16_t index,
                   uint16_t length,
                   addr_t desc)
{
  USB_DEV_REQ setup_req;
  
  if ((dev->devd).bMaxPacketSize0 == 0) {
    DLOG("USB_DEVICE_INFO is probably not initialized!");
    return -1;
  }

  setup_req.bmRequestType = 0x80;  /*
                                    * Characteristics of request,
                                    * see spec, P183, Rev 1.1
                                    */
  
  setup_req.bRequest = USB_GET_DESCRIPTOR;
  setup_req.wValue = (dtype << 8) + dindex;
  setup_req.wIndex = index;
  setup_req.wLength = length;
  
  return usb_control_transfer(dev,
                              (addr_t) & setup_req,
                              sizeof (USB_DEV_REQ),
                              desc, length);
}

int
usb_set_address (USB_DEVICE_INFO* dev, uint8_t new_addr)
{
  sint status;
  USB_DEV_REQ setup_req;
  uint8_t old_addr = dev->address;
  usb_hcd_t* hcd = dev->hcd;
  
  setup_req.bmRequestType = 0x0;
  setup_req.bRequest = USB_SET_ADDRESS;
  setup_req.wValue = new_addr;
  setup_req.wIndex = 0;
  setup_req.wLength = 0;

  status = usb_control_transfer (dev, (addr_t) & setup_req,
                                 sizeof (USB_DEV_REQ), 0, 0);
  if (status == 0) {
    hcd->toggles[old_addr] = hcd->toggles[new_addr] = 0;
  }
  return status;
  /*
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_set_address(dev->address, new_addr,
            (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_set_address");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
  */
}

int
usb_get_configuration(USB_DEVICE_INFO* dev)
{
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_get_configuration(dev->address,
            (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_get_configuration");
      panic("EHCI Host Controller is not supported now! usb_get_configuration");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
  
}

int
usb_set_configuration(USB_DEVICE_INFO * dev, uint8_t conf)
{
  
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = 0x0;
  setup_req.bRequest = USB_SET_CONFIGURATION;
  setup_req.wValue = conf;
  setup_req.wIndex = 0;
  setup_req.wLength = 0;
  /*
   * A bulk endpoint's toggle is initialized to DATA0 when any
   * configuration event is experienced
   */
  dev->hcd->toggles[dev->address] = 0;

  return usb_control_transfer (dev, (addr_t) & setup_req,
                                 sizeof (USB_DEV_REQ), 0, 0);
  /*
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_set_configuration(dev->address, conf,
            (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_set_configuration");
      panic("usb_set_configuration not implemented");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
  */
}

int
usb_get_interface(USB_DEVICE_INFO * dev, uint16_t interface)
{
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_get_interface(dev->address, interface,
            (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_get_interface");
      panic("ehci usb_get_interface not supported");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
}

int
usb_set_interface(USB_DEVICE_INFO * dev, uint16_t alt, uint16_t interface)
{
  switch (dev->host_type)
  {
    case USB_TYPE_HC_UHCI :
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_set_interface(dev->address, alt, interface,
            (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_set_interface");
      panic("EHCI Host Controller is not supported now! usb_set_interface");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
  }
  return -1;
}

bool
usb_register_driver (USB_DRIVER *driver)
{
  if (num_drivers >= USB_MAX_DEVICE_DRIVERS) return FALSE;
  memcpy (&drivers[num_drivers], driver, sizeof (USB_DRIVER));
  num_drivers++;
  return TRUE;
}

static void
find_device_driver (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  int d;
  for (d=0; d<num_drivers; d++)
    if (drivers[d].probe (info, cfgd, ifd)) return;
}

static void
dlog_devd (USB_DEV_DESC *devd)
{
  DLOG ("DEVICE DESCRIPTOR len=%d type=%d bcdUSB=0x%x",
        devd->bLength, devd->bDescriptorType, devd->bcdUSB);
  DLOG ("  class=0x%x subclass=0x%x proto=0x%x maxpkt0=%d",
        devd->bDeviceClass, devd->bDeviceSubClass, devd->bDeviceProtocol,
        devd->bMaxPacketSize0);
  DLOG ("  vendor=0x%x product=0x%x bcdDevice=0x%x numcfgs=%d",
        devd->idVendor, devd->idProduct, devd->bcdDevice,
        devd->bNumConfigurations);
}

static void
dlog_info (USB_DEVICE_INFO *info)
{
  DLOG ("ADDRESS %d", info->address);
  dlog_devd (&info->devd);

#if 1
  uint8 strbuf[64];
  uint8 str[32];
#define do_str(slot,label)                                              \
  if (info->devd.slot != 0 &&                                           \
      uhci_get_string (info->address, info->devd.slot, 0,               \
                       sizeof (strbuf), strbuf,                         \
                       info->devd.bMaxPacketSize0)                      \
      == 0) {                                                           \
    memset (str, 0, sizeof (str));                                      \
    if (uhci_interp_string ((USB_STR_DESC *)strbuf, sizeof (strbuf), 0, \
                            str, sizeof (str)-1) > 0) {                 \
      DLOG ("  "label": %s", str);                                      \
    }                                                                   \
  }

  do_str (iManufacturer, "Manufacturer");
  do_str (iProduct, "Product");
#undef do_str
#endif
}



/* figures out what device is attached as address 0 */
bool
usb_enumerate(usb_hcd_t* usb_hcd)
{
  USB_DEV_DESC devd;
  USB_CFG_DESC *cfgd;
  USB_IF_DESC *ifd;
#define TEMPSZ 256
  uint8 temp[TEMPSZ], *ptr;
  sint status, c, i, total_length=0;
  USB_DEVICE_INFO *info;
  uint curdev = usb_hcd->next_address;

  if (curdev > USB_MAX_DEVICES) {
    DLOG ("usb_enumerate: too many devices");
    return FALSE;
  }

  DLOG ("usb_enumerate: curdev=%d", curdev);

  /* clear device info */
  info = &usb_hcd->devinfo[curdev];
  memset (info, 0, sizeof (USB_DEVICE_INFO));
  info->address = 0;
  info->host_type = usb_hcd->usb_hc_type;
  info->hcd = usb_hcd;

  /* --WARN-- OK, here is the deal. The spec says you should use the
   * maximum packet size in the data phase of control transfer if the
   * data is larger than one packet. Since we do not want to support
   * low speed device for now, the bMaxPacketSize0 is always set to 64
   * bytes for full speed device. So, do not be surprised if your USB
   * mouse does not work in Quest!
   */
  info->devd.bMaxPacketSize0 = 64;

  memset (&devd, 0, sizeof (USB_DEV_DESC));

  /* get device descriptor */
  status = usb_get_descriptor (info, USB_TYPE_DEV_DESC, 0, 0,
                               sizeof (USB_DEV_DESC), &devd);
  if (status < 0) {
    DLOG("Could not get descriptor");
    goto abort;
  }
  
  dlog_devd(&devd);

  if (devd.bMaxPacketSize0 == 8) {
    /* get device descriptor */
    info->devd.bMaxPacketSize0 = 8;
    status = usb_get_descriptor(info, USB_TYPE_DEV_DESC, 0, 0,
                                sizeof(USB_DEV_DESC), &devd);
    if (status < 0)
      goto abort;
  }
  
  if (devd.bNumConfigurations == 255)
    devd.bNumConfigurations = 1; /* --!!-- hack */
  
  /* Update device info structure. Put it in USB core might be better */
  memcpy (&info->devd, &devd, sizeof (USB_DEV_DESC));
  
  /* assign an address */
  if (usb_set_address (info, curdev) < 0)
    goto abort;
  
  DLOG ("usb_enumerate: set (0x%x, 0x%x, 0x%x) to addr %d",
        devd.bDeviceClass, devd.idVendor, devd.idProduct, curdev);
  delay (2);
  
  DLOG ("usb_enumerate: num configs=%d", devd.bNumConfigurations);
  
  /* Update device info structure for new address. */
  info->address = curdev;



  for (c=0; c<devd.bNumConfigurations; c++) {
    /* get a config descriptor for size field */
    memset (temp, 0, TEMPSZ);
    status = usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0,
                                 sizeof (USB_CFG_DESC), temp);
    if (status < 0) {
      DLOG ("usb_enumerate: failed to get config descriptor for c=%d", c);
      goto abort;
    }

    cfgd = (USB_CFG_DESC *)temp;
    DLOG ("usb_enumerate: c=%d cfgd->wTotalLength=%d", c, cfgd->wTotalLength);
    total_length += cfgd->wTotalLength;
  }

  DLOG ("usb_enumerate: total_length=%d", total_length);

  /* allocate memory to hold everything */
  pow2_alloc (total_length, &info->raw);
  if (!info->raw) {
    DLOG ("usb_enumerate: pow2_alloc (%d) failed", total_length);
    goto abort;
  }

  /* read all cfg, if, and endpoint descriptors */
  ptr = info->raw;
  for (c=0; c < devd.bNumConfigurations; c++) {
    /* obtain precise size info */
    memset (temp, 0, TEMPSZ);
    status = usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0,
                                 sizeof (USB_CFG_DESC), temp);
    if (status < 0) {
      DLOG ("usb_enumerate: failed to get config descriptor for c=%d", c);
      goto abort_mem;
    }
    cfgd = (USB_CFG_DESC *)temp;

    /* get cfg, if, and endp descriptors */
    status =
      usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0, cfgd->wTotalLength, ptr);
    if (status < 0)
      goto abort_mem;

    cfgd = (USB_CFG_DESC *)ptr;
    DLOG ("usb_enumerate: cfg %d has num_if=%d", c, cfgd->bNumInterfaces);
    ptr += cfgd->wTotalLength;
  }

  /* incr this here because hub drivers may recursively invoke enumerate */
  usb_hcd->next_address++;

  /* parse cfg and if descriptors */
  ptr = info->raw;
  for (c=0; c < devd.bNumConfigurations; c++) {
    cfgd = (USB_CFG_DESC *) ptr;
    ptr += cfgd->bLength;
    for (i=0; i < cfgd->bNumInterfaces; i++) {
      /* find the next if descriptor, skipping any class-specific stuff */
      for (ifd = (USB_IF_DESC *) ptr;
           ifd->bDescriptorType != USB_TYPE_IF_DESC;
           ifd = (USB_IF_DESC *)((uint8 *)ifd + ifd->bLength)) {
        //DLOG ("ifd=%p len=%d type=0x%x", ifd, ifd->bLength, ifd->bDescriptorType);
      }
      ptr = (uint8 *) ifd;
      DLOG ("usb_enumerate: examining (%d, %d) if_class=0x%X sub=0x%X proto=0x%X #endp=%d",
            c, i, ifd->bInterfaceClass,
            ifd->bInterfaceSubClass,
            ifd->bInterfaceProtocol,
            ifd->bNumEndpoints);

      /* find a device driver interested in this interface */
      find_device_driver (info, cfgd, ifd);

      ptr += ifd->bLength;
    }
    ptr = ((uint8 *)cfgd) + cfgd->wTotalLength;
  }

  /*
   * --??-- what happens if more than one driver matches more than one
   * config?
   */
  

  return TRUE;

 abort_mem:
  pow2_free (info->raw);
 abort:
  return FALSE;
}

bool
usb_init (void)
{
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_init
};

DEF_MODULE (usb, "USB manager", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
