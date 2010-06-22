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

#define DLOG(fmt,...) DLOG_PREFIX("uvc",fmt,##__VA_ARGS__)

#define printf com1_printf
#define print com1_puts
#define putx  com1_putx
#define putchar com1_putc

#else

#define DLOG(fmt,...) ;
#endif

bool usb_uvc_driver_init (void);
static bool uvc_probe (USB_DEVICE_INFO *, USB_CFG_DESC *, USB_IF_DESC *);
static void desc_dump (USB_DEVICE_INFO *, USB_CFG_DESC *);

static bool
uvc_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  /* Avoid multi-entrance in uhci_enumerate(), should be removed soon */
  static int entrance = 0;
  if (entrance) return FALSE;

  USB_SPD_CFG_DESC *scfgd;
  uint8_t tmp[1300];

  memset(tmp, 0, 1300);

  if((dev->devd.bDeviceClass == 0xEF) &&
     (dev->devd.bDeviceSubClass == 0x02) &&
     (dev->devd.bDeviceProtocol == 0x01)) {
    DLOG("Device with one or more Video Interface Collections detected");
  }

#if 1
  /* Dumping UVC device descriptors */
  desc_dump (dev, cfg);
#endif

  DLOG("Now, getting other speed configuration.");
  if (usb_get_descriptor(dev, USB_TYPE_SPD_CFG_DESC, 0, 0, 9, (addr_t)tmp)) {
    DLOG("Other Speed Configuration is not presented.");
  } else {
    scfgd = (USB_SPD_CFG_DESC *)tmp;
    DLOG("Other Speed Configuration:");
    DLOG("  bLength : 0x%x  bDescriptorType : 0x%x  wTotalLength : 0x%x",
        scfgd->bLength, scfgd->bDescriptorType, scfgd->wTotalLength);
    DLOG("  bNumInterfaces : 0x%x  bConfigurationValue : 0x%x",
        scfgd->bNumInterfaces, scfgd->bConfigurationValue);

    memset(tmp, 0, 1300);
    //usb_get_descriptor(dev, USB_TYPE_SPD_CFG_DESC, 0, 0,
    //    1300, (addr_t)tmp);
  }

  DLOG("Set configuration to %d.", cfg->bConfigurationValue);
  usb_set_configuration(dev, cfg->bConfigurationValue);
  DLOG("New configuration is : %d", usb_get_configuration(dev));

  /* Avoid multi-entrance in uhci_enumerate(), should be removed soon */
  entrance++;

  return TRUE;
}

static USB_DRIVER uvc_driver = {
  .probe = uvc_probe
};

bool
usb_uvc_driver_init (void)
{
  return usb_register_driver (&uvc_driver);
}

static void
desc_dump (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg)
{
  /* Dump all the descriptors retreived from UVC device */
  uint8_t conf[1300];

  USB_DEV_DESC *desc;
  USB_CFG_DESC *cfgd;
  UVC_IA_DESC *iad;
  USB_IF_DESC *vcifd;
  UVC_CSVC_IF_HDR_DESC *csvcifd;

  DLOG("Dumping UVC device descriptors");

  desc = &(dev->devd);

#if 1
  print("Device Descriptor: \n");
  print("  bLength : ");
  putx(desc->bLength);
  putchar('\n');
  print("  bDescriptorType : ");
  putx(desc->bDescriptorType);
  putchar('\n');
  print("  bcdUSB : ");
  putx(desc->bcdUSB);
  putchar('\n');
  print("  bDeviceClass : ");
  putx(desc->bDeviceClass);
  putchar('\n');
  print("  bDeviceSubClass : ");
  putx(desc->bDeviceSubClass);
  putchar('\n');
  print("  bDeviceProtocol : ");
  putx(desc->bDeviceProtocol);
  putchar('\n');
  print("  bMaxPacketSize0 : ");
  putx(desc->bMaxPacketSize0);
  putchar('\n');
  print("  idVendor : ");
  putx(desc->idVendor);
  putchar('\n');
  print("  idProduct : ");
  putx(desc->idProduct);
  putchar('\n');
  print("  bNumConfigurations : ");
  putx(desc->bNumConfigurations);
  putchar('\n');
#endif

  cfgd = (USB_CFG_DESC*)cfg;

#if 1
  print("Configuration Descriptor: \n");
  print("  bLength : ");
  putx(cfgd->bLength);
  putchar('\n');
  print("  bDescriptorType : ");
  putx(cfgd->bDescriptorType);
  putchar('\n');
  print("  wTotalLength : ");
  putx(cfgd->wTotalLength);
  putchar('\n');
  print("  bNumInterfaces : ");
  putx(cfgd->bNumInterfaces);
  putchar('\n');
  print("  bConfigurationValue : ");
  putx(cfgd->bConfigurationValue);
  putchar('\n');
  print("  iConfiguration : ");
  putx(cfgd->iConfiguration);
  putchar('\n');
  print("  bmAttributes : ");
  putx(cfgd->bmAttributes);
  putchar('\n');
  print("  MaxPower : ");
  putx(cfgd->MaxPower);
  putchar('\n');
#endif

  memset(conf, 0, 1300);

  print("Getting all descriptors.\n");
  usb_get_descriptor(dev, USB_TYPE_CFG_DESC, 0, 0, cfgd->wTotalLength, (addr_t)conf);
  iad = (UVC_IA_DESC*)(&conf[cfgd->bLength]);

#if 1
  print("Interface Association Descriptor: \n");
  print("  bLength : ");
  putx(iad->bLength);
  putchar('\n');
  print("  bDescriptorType : ");
  putx(iad->bDescriptorType);
  putchar('\n');
  print("  bFirstInterface : ");
  putx(iad->bFirstInterface);
  putchar('\n');
  print("  bInterfaceCount : ");
  putx(iad->bInterfaceCount);
  putchar('\n');
  print("  bFunctionClass : ");
  putx(iad->bFunctionClass);
  putchar('\n');
  print("  bFunctionSubClass : ");
  putx(iad->bFunctionSubClass);
  putchar('\n');
  print("  bFunctionProtocol : ");
  putx(iad->bFunctionProtocol);
  putchar('\n');
  print("  iFunction : ");
  putx(iad->iFunction);
  putchar('\n');
#endif

  vcifd = (USB_IF_DESC*)(&conf[cfgd->bLength + iad->bLength]);

#if 1
  print("VC Interface Descriptor : \n");
  print("  bLength : ");
  putx(vcifd->bLength);
  putchar('\n');
  print("  bDescriptorType : ");
  putx(vcifd->bDescriptorType);
  putchar('\n');
  print("  bInterfaceNumber : ");
  putx(vcifd->bInterfaceNumber);
  putchar('\n');
  print("  bAlternateSetting : ");
  putx(vcifd->bAlternateSetting);
  putchar('\n');
  print("  bNumEndpoints : ");
  putx(vcifd->bNumEndpoints);
  putchar('\n');
  print("  bInterfaceClass : ");
  putx(vcifd->bInterfaceClass);
  putchar('\n');
  print("  bInterfaceSubClass : ");
  putx(vcifd->bInterfaceSubClass);
  putchar('\n');
  print("  bInterfaceProtocol : ");
  putx(vcifd->bInterfaceProtocol);
  putchar('\n');
  print("  iInterface : ");
  putx(vcifd->iInterface);
  putchar('\n');
#endif

  csvcifd = (UVC_CSVC_IF_HDR_DESC*)(&conf[cfgd->bLength + iad->bLength + vcifd->bLength]);

#if 1
  print("Class-Specific VC Interface Header Descriptor : \n");
  print("  bLength : ");
  putx(csvcifd->bLength);
  putchar('\n');
  print("  bDescriptorType : ");
  putx(csvcifd->bDescriptorType);
  putchar('\n');
  print("  bDescriptorSubType : ");
  putx(csvcifd->bDescriptorSubType);
  putchar('\n');
  print("  bcdUVC : ");
  putx(csvcifd->bcdUVC);
  putchar('\n');
  print("  wTotalLength : ");
  putx(csvcifd->wTotalLength);
  putchar('\n');
  print("  dwClockFrequency : ");
  putx(csvcifd->dwClockFrequency);
  putchar('\n');
  print("  bInCollection : ");
  putx(csvcifd->bInCollection);
  putchar('\n');
  print("  baInterface1 : ");
  putx(csvcifd->baInterface1);
  putchar('\n');
#endif

  DLOG("Dumping UVC device descriptors ends");

#if 1

  uint32_t *dump = (uint32_t*)conf;
  int i = 0;

  for(i = 1; i < 325; i++) {
    putx(*dump);
    dump++;
    putchar('\n');
  }
#endif
  
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
