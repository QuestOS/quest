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

#include "drivers/usb/usb_tests.h"
#include "drivers/usb/usb.h"
#include "drivers/usb/umsc.h"
#include "util/printf.h"
#include "kernel.h"

#define DEBUG_USB_TEST

#ifdef DEBUG_USB_TEST
#define DLOG(fmt,...) DLOG_PREFIX("usb-test",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define printf com1_printf
#define print com1_puts
#define putx  com1_putx
#define putchar com1_putc

static void call_back(addr_t buf);

void
show_usb_regs (int bus, int dev, int func)
{
  uint16_t base_addr = 0;
  uint32_t ldata = 0;
  uint16_t wdata = 0;
  uint8_t bdata = 0;
#if 0
  print ("\nUHCI Controller PCI Register Address Map\n\n");

  wdata = pci_config_rd16 (bus, dev, func, 0x00);
  print ("VID: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x02);
  print ("DID: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x04);
  print ("PCICMD: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x06);
  print ("PCISTS: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x08);
  print ("RID: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x09);
  print ("PI: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0A);
  print ("SCC: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0B);
  print ("BCC: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0D);
  print ("MLT: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x0E);
  print ("HEADTYP: 0x");
  putx (bdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x20);
  print ("BASE: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x2C);
  print ("SVID: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0x2E);
  print ("SID: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3C);
  print ("INT_LN: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3D);
  print ("INT_PN: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x60);
  print ("USB_RELNUM: 0x");
  putx (bdata);
  putchar ('\n');

  wdata = pci_config_rd16 (bus, dev, func, 0xC0);
  print ("USB_LEGKEY: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0xC4);
  print ("USB_RES: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0xC8);
  print ("CWP: 0x");
  putx (bdata);
  putchar ('\n');
#endif
  base_addr = pci_config_rd16 (bus, dev, func, 0x20);
  base_addr &= 0xFFE0;

  print ("\nBase address for USB I/O Registers: 0x");
  putx (base_addr);
  putchar ('\n');

  print ("\nUSB I/O Registers\n\n");

  wdata = inw (base_addr + 0x00);
  print ("USBCMD: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x02);
  print ("USBSTS: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x04);
  print ("USBINTR: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x06);
  print ("FRNUM: 0x");
  putx (wdata);
  putchar ('\n');

  ldata = inl (base_addr + 0x08);
  print ("FRBASEADD: 0x");
  putx (ldata);
  putchar ('\n');

  bdata = inb (base_addr + 0x0C);
  print ("SOFMOD: 0x");
  putx (bdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x10);
  print ("PORTSC0: 0x");
  putx (wdata);
  putchar ('\n');

  wdata = inw (base_addr + 0x12);
  print ("PORTSC1: 0x");
  putx (wdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3C);
  print ("INT_LN: 0x");
  putx (bdata);
  putchar ('\n');

  bdata = pci_config_rd8 (bus, dev, func, 0x3D);
  print ("INT_PN: 0x");
  putx (bdata);
  putchar ('\n');


  return;
}

sint
umsc_bo_reset (uint address, uint interface_idx)
{
  USB_DEV_REQ req;
  printf ("bmsc_reset (%d, %d)\n", address, interface_idx);
  memset (&req, 0, sizeof (req));
  req.bmRequestType = 0x21;
  req.bRequest = 0xFF;
  req.wIndex = interface_idx;
  /* We assume this is a full speed device, use the maximum, 64 bytes */
  return uhci_control_transfer (address, (addr_t)&req, sizeof (req), 0, 0, 64);
}

void isochronous_transfer_test(void)
{
#if 0
  uint8_t data[20];
  uint8_t conf[1300];
  uint8_t iso1[1024];

  memset(data, 0, 20);
  USB_DEV_DESC *desc;
  USB_CFG_DESC *cfgd;
  UVC_IA_DESC *iad;
  USB_IF_DESC *vcifd;
  UVC_CSVC_IF_HDR_DESC *csvcifd;
  //USB_EPT_DESC *eptd;

  port_reset(0);
  port_reset(1);

  uhci_get_descriptor(0, TYPE_DEV_DESC, 0, 0, 18, (addr_t)data);
  desc = (USB_DEV_DESC*)data;
  print("The length of device descriptor is: 0x");
  putx(desc->bLength);
  putchar('\n');

  port_reset(0);
  port_reset(1);

  print("Setting new address for the device.\n");
  uhci_set_address(0, 1);

  memset(data, 0, 20);

  print("Now, getting device descriptor again.\n");
  uhci_get_descriptor(1, TYPE_DEV_DESC, 0, 0, 18, (addr_t)data);
  desc = (USB_DEV_DESC*)data;

#if 1
  print("Device Descriptor: \n");
  print("  bLength : ");
  putx(desc->bLength);
  putchar('\n');
  print("  bDescriptorType : ");
  putx(desc->bDescriptorType);
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
  print("  idVendor : ");
  putx(desc->idVendor);
  putchar('\n');
  print("  bNumConfigurations : ");
  putx(desc->bNumConfigurations);
  putchar('\n');
#endif

  delay(100);

  memset(data, 0, 20);

  print("Getting configuration descriptor.\n");
  uhci_get_descriptor(1, TYPE_CFG_DESC, 0, 0, 9, (addr_t)data);
  cfgd = (USB_CFG_DESC*)data;

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
  uhci_get_descriptor(1, TYPE_CFG_DESC, 0, 0, cfgd->wTotalLength, (addr_t)conf);
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

  delay(100);

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

  delay(100);

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

  delay(100);

  print("Set configuration to 1.\n");
  uhci_set_configuration(1, 1);

  print("New configuration is : ");
  putx(uhci_get_configuration(1));
  putchar('\n');

  delay(100);

#if 1
  memset(conf, 0, 1300);

  int status = uhci_get_descriptor(1, TYPE_CFG_DESC, 0, 0, 1000, (addr_t)conf);
  print("Status Code : ");
  putx(status);
  putchar('\n');

  uint32_t *dump = (uint32_t*)conf;
  int i = 0;

  for(i = 1; i < 200; i++) {
    putx(*dump);
    dump++;
    if(i%9 == 0) putchar('\n');
  }
#endif

  uhci_isochronous_transfer(1, 5, (addr_t)iso1, 1023, 1, DIR_IN, call_back);
}

static void call_back(addr_t buf) {
  print("Now, we are in the call back function.\n");
  int i = 0;

  for(i = 1; i < 256; i++) {
    putx((uint32_t)((uint32_t*)buf + i));
    if(i%5 == 0) putchar('\n');
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
