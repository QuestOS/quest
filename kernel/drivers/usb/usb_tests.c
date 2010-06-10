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

#define UMSC_CBW_SIGNATURE 0x43425355 /* "USBC" (little-endian) */
struct umsc_cbw {
  uint32 dCBWSignature;
  uint32 dCBWTag;
  uint32 dCBWDataTransferLength;
  union {
    uint8 raw;
    struct {
      uint8 _reserved:7;
      uint8 direction:1;
    };
  } bmCBWFlags;
  uint8 bCBWLUN:4;
  uint8 _reserved1:4;
  uint8 bCBWCBLength:5;
  uint8 _reserved2:3;
  uint8 CBWCB[16];  
} PACKED;
typedef struct umsc_cbw UMSC_CBW;

#define UMSC_CSW_SIGNATURE 0x53425355 /* "USBS" (little-endian) */
struct umsc_csw {
  uint32 dCSWSignature;
  uint32 dCSWTag;
  uint32 dCSWDataResidue;
  uint8  bCSWStatus;
} PACKED;
typedef struct umsc_csw UMSC_CSW;

sint
umsc_bo_reset (uint address, uint interface_idx)
{
  USB_DEV_REQ req;
  printf ("bmsc_reset (%d, %d)\n", address, interface_idx);
  memset (&req, 0, sizeof (req));
  req.bmRequestType = 0x21;
  req.bRequest = 0xFF;
  req.wIndex = interface_idx;
  return uhci_control_transfer (address, (addr_t)&req, sizeof (req), 0, 0);
}

extern uint8 glb_toggle;

static uint8 in_tog = 0, out_tog = 0;

sint
umsc_bulk_scsi (uint addr, uint ep_out, uint ep_in,
                uint8 cmd[16], uint dir, uint8* data, 
                uint data_len, uint maxpkt)
{
  UMSC_CBW cbw;
  UMSC_CSW csw;
  sint status, i, j;

  printf ("umsc_bulk_scsi cmd:");
  for (i=0;i<16;i++) printf (" %.02X", cmd[i]);
  printf ("\n");

  memset (&cbw, 0, sizeof (cbw));
  memset (&csw, 0, sizeof (csw));

  cbw.dCBWSignature = UMSC_CBW_SIGNATURE;
  cbw.dCBWDataTransferLength = data_len;
  cbw.bmCBWFlags.direction = dir;
  cbw.bCBWCBLength = 16;            /* cmd length */
  memcpy (cbw.CBWCB, cmd, 16);

  glb_toggle = out_tog;
  status = uhci_bulk_transfer (addr, ep_out, &cbw, 0x1f, maxpkt, DIR_OUT);
  out_tog = glb_toggle;

  printf ("status=%d\n", status);

  if (data_len > 0) {
    if (dir) { 
      glb_toggle = in_tog;
      status = uhci_bulk_transfer (addr, ep_in, data, data_len, maxpkt, DIR_IN);
      in_tog = glb_toggle;
    }
    else {
      glb_toggle = out_tog;
      status = uhci_bulk_transfer (addr, ep_out, data, data_len, maxpkt, DIR_OUT);
      out_tog = glb_toggle;
    }

    printf ("status=%d\n", status);

    if (status != 0) return status;

    for (j=0;j<8;j++) {
      sint k;
      for (k=0;k<8;k++)
        printf ("%.02x ", data[j*8+k]);
      printf ("\n");
    }

  }

  glb_toggle = in_tog;
  status = uhci_bulk_transfer (1, ep_in, (addr_t) &csw, 0x0d, maxpkt, DIR_IN);
  in_tog = glb_toggle;

  printf ("status=%d\n", status);

  if (status != 0) return status;

  printf ("csw sig=%p tag=%p res=%d status=%d\n",
          csw.dCSWSignature, csw.dCSWTag, csw.dCSWDataResidue, csw.bCSWStatus);

  return status;
}

void
control_transfer_test (void)
{
  uint sector_size, last_lba;
  sint status, j;
  uint8_t data[20];
  uint8_t conf[2048];
  uint8_t iso1[1024];
  //uint8_t iso2[1024];
  //uint8_t iso3[1024];
  //uint8_t iso4[1024];
  //uint8_t iso5[1024];
  //uint8_t iso6[1024];
  memset (data, 0, 20);
  USB_DEV_DESC *desc;
  USB_CFG_DESC *cfgd;
  UVC_IA_DESC *iad;
  USB_IF_DESC *vcifd, *ifd;
  UVC_CSVC_IF_HDR_DESC *csvcifd;
  USB_EPT_DESC *ep1, *ep2, *ep;
  uint8 ep_in, ep_out;
  UMSC_CBW cbw;
  //USB_EPT_DESC *eptd;

  port_reset (0);
  port_reset (1);

  //show_usb_regs();

  uhci_get_descriptor (0, TYPE_DEV_DESC, 0, 0, 18, (addr_t) data);
  desc = (USB_DEV_DESC *) data;
  print ("The length of device descriptor is: 0x");
  putx (desc->bLength);
  putchar ('\n');

  port_reset (0);
  port_reset (1);

  print ("Setting new address for the device.\n");
  uhci_set_address (0, 1);

  //show_usb_regs();

  memset (data, 0, 20);

  print ("Now, getting device descriptor again.\n");
  uhci_get_descriptor (1, TYPE_DEV_DESC, 0, 0, 18, (addr_t) data);
  desc = (USB_DEV_DESC *) data;

#if 1
  print ("Device Descriptor: \n");
  print ("  bLength : ");
  putx (desc->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (desc->bDescriptorType);
  putchar ('\n');
  print ("  bDeviceClass : ");
  putx (desc->bDeviceClass);
  putchar ('\n');
  print ("  bDeviceSubClass : ");
  putx (desc->bDeviceSubClass);
  putchar ('\n');
  print ("  bDeviceProtocol : ");
  putx (desc->bDeviceProtocol);
  putchar ('\n');
  print ("  idVendor : ");
  putx (desc->idVendor);
  putchar ('\n');
  print ("  idProduct : ");
  putx (desc->idProduct);
  putchar ('\n');
  print ("  bNumConfigurations : ");
  putx (desc->bNumConfigurations);
  putchar ('\n');
#endif

  delay (1000);
  delay (1000);
  delay (1000);

  memset (data, 0, 20);

  print ("Getting configuration descriptor.\n");
  uhci_get_descriptor (1, TYPE_CFG_DESC, 0, 0, 9, (addr_t) data);
  cfgd = (USB_CFG_DESC *) data;

#if 1
  print ("Configuration Descriptor: \n");
  print ("  bLength : ");
  putx (cfgd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (cfgd->bDescriptorType);
  putchar ('\n');
  print ("  wTotalLength : ");
  putx (cfgd->wTotalLength);
  putchar ('\n');
  print ("  bNumInterfaces : ");
  putx (cfgd->bNumInterfaces);
  putchar ('\n');
  print ("  bConfigurationValue : ");
  putx (cfgd->bConfigurationValue);
  putchar ('\n');
  print ("  iConfiguration : ");
  putx (cfgd->iConfiguration);
  putchar ('\n');
  print ("  bmAttributes : ");
  putx (cfgd->bmAttributes);
  putchar ('\n');
  print ("  MaxPower : ");
  putx (cfgd->MaxPower);
  putchar ('\n');
#endif

  memset (conf, 0, 1300);

  print ("Getting all descriptors.\n");
  uhci_get_descriptor (1, TYPE_CFG_DESC, 0, 0, cfgd->wTotalLength,
                       (addr_t) conf);
  ifd = (USB_IF_DESC *) (&conf[cfgd->bLength]);
  ep1 = (USB_EPT_DESC *) (&conf[cfgd->bLength + ifd->bLength]); 
  ep2 = (USB_EPT_DESC *) (&conf[cfgd->bLength + ifd->bLength + ep1->bLength]); 

  printf ("interface %d protocol=%x subclass=%x\n",
          ifd->bInterfaceNumber, 
          ifd->bInterfaceProtocol, ifd->bInterfaceSubClass);
  ep = ep1;
  printf ("endpoint length=%x desctype=%x endaddr=%x attr=%x maxpkt=%x interval=%x\n",
          ep->bLength, ep->bDescriptorType, ep->bEndpointAddress,
          ep->bmAttributes, ep->wMaxPacketSize, ep->bInterval);

  if (ep->bEndpointAddress & 0x80)
    ep_in = ep->bEndpointAddress & 0x7F;
  else
    ep_out = ep->bEndpointAddress & 0x7F;

  ep = ep2;
  printf ("endpoint length=%x desctype=%x endaddr=%x attr=%x maxpkt=%x interval=%x\n",
          ep->bLength, ep->bDescriptorType, ep->bEndpointAddress,
          ep->bmAttributes, ep->wMaxPacketSize, ep->bInterval);

  if (ep->bEndpointAddress & 0x80)
    ep_in = ep->bEndpointAddress & 0x7F;
  else
    ep_out = ep->bEndpointAddress & 0x7F;

  
  delay (1000);

  print ("Set configuration to 1.\n");
  uhci_set_configuration (1, 1);

  printf ("New configuration is : %x\n",
          uhci_get_configuration (1));

  delay (1000);


  {
    uint8 cmd[16] = {0x12,0,0,0,0x24,0,0,0,0,0,0,0};
    printf ("SENDING INQUIRY\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, 0x24, 0x40);
  }

  {
    uint8 cmd[16] = {0,0,0,0,0,0,0,0,0,0,0,0};
    printf ("SENDING TEST UNIT READY\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, 0, 0x40);
  }

  {
    uint8 cmd[16] = {0x03,0,0,0,0x24,0,0,0,0,0,0,0};
    printf ("SENDING REQUEST SENSE\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, 0x24, 0x40);
  }

  {
    uint8 cmd[16] = {0,0,0,0,0,0,0,0,0,0,0,0};
    printf ("SENDING TEST UNIT READY\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, 0, 0x40);
  }

  {
    uint8 cmd[16] = {0x03,0,0,0,0x24,0,0,0,0,0,0,0};
    printf ("SENDING REQUEST SENSE\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, 0x24, 0x40);
  }

  {
    uint8 cmd[16] = {0x25,0,0,0,0,0,0,0,0,0,0,0};
    printf ("SENDING READ CAPACITY\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, 0x8, 0x40);
    last_lba = conf[3] | conf[2] << 8 | conf[1] << 16 | conf[0] << 24;
    sector_size = conf[7] | conf[6] << 8 | conf[5] << 16 | conf[4] << 24;
    printf ("sector_size=0x%x last_lba=0x%x total_size=%d bytes\n",
            sector_size, last_lba, sector_size * (last_lba + 1));
  }

  {
    uint8 cmd[16] = { [0] = 0x28, [8] = 1 };
    printf ("SENDING READ (10)\n");
    umsc_bulk_scsi (1, ep_out, ep_in, cmd, 1, conf, sector_size, 0x40);
  }


#if 0
  iad = (UVC_IA_DESC *) (&conf[cfgd->bLength]);

  print ("Interface Association Descriptor: \n");
  print ("  bLength : ");
  putx (iad->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (iad->bDescriptorType);
  putchar ('\n');
  print ("  bFirstInterface : ");
  putx (iad->bFirstInterface);
  putchar ('\n');
  print ("  bInterfaceCount : ");
  putx (iad->bInterfaceCount);
  putchar ('\n');
  print ("  bFunctionClass : ");
  putx (iad->bFunctionClass);
  putchar ('\n');
  print ("  bFunctionSubClass : ");
  putx (iad->bFunctionSubClass);
  putchar ('\n');
  print ("  bFunctionProtocol : ");
  putx (iad->bFunctionProtocol);
  putchar ('\n');
  print ("  iFunction : ");
  putx (iad->iFunction);
  putchar ('\n');
#endif

  delay (1000);
  delay (1000);
  delay (1000);
  delay (1000);
  delay (1000);

#if 0
  vcifd = (USB_IF_DESC *) (&conf[cfgd->bLength + iad->bLength]);


  print ("VC Interface Descriptor : \n");
  print ("  bLength : ");
  putx (vcifd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (vcifd->bDescriptorType);
  putchar ('\n');
  print ("  bInterfaceNumber : ");
  putx (vcifd->bInterfaceNumber);
  putchar ('\n');
  print ("  bAlternateSetting : ");
  putx (vcifd->bAlternateSetting);
  putchar ('\n');
  print ("  bNumEndpoints : ");
  putx (vcifd->bNumEndpoints);
  putchar ('\n');
  print ("  bInterfaceClass : ");
  putx (vcifd->bInterfaceClass);
  putchar ('\n');
  print ("  bInterfaceSubClass : ");
  putx (vcifd->bInterfaceSubClass);
  putchar ('\n');
  print ("  bInterfaceProtocol : ");
  putx (vcifd->bInterfaceProtocol);
  putchar ('\n');
  print ("  iInterface : ");
  putx (vcifd->iInterface);
  putchar ('\n');
#endif

  delay (1000);
  delay (1000);
  delay (1000);
  delay (1000);
  delay (1000);

#if 0
  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength]);


  print ("Class-Specific VC Interface Header Descriptor : \n");
  print ("  bLength : ");
  putx (csvcifd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (csvcifd->bDescriptorType);
  putchar ('\n');
  print ("  bDescriptorSubType : ");
  putx (csvcifd->bDescriptorSubType);
  putchar ('\n');
  print ("  bcdUVC : ");
  putx (csvcifd->bcdUVC);
  putchar ('\n');
  print ("  wTotalLength : ");
  putx (csvcifd->wTotalLength);
  putchar ('\n');
  print ("  dwClockFrequency : ");
  putx (csvcifd->dwClockFrequency);
  putchar ('\n');
  print ("  bInCollection : ");
  putx (csvcifd->bInCollection);
  putchar ('\n');
  print ("  baInterface1 : ");
  putx (csvcifd->baInterface1);
  putchar ('\n');
#endif

#if 0
  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength +
               csvcifd->bLength]);


  print ("Class-Specific VC Interface Header Descriptor : \n");
  print ("  bLength : ");
  putx (csvcifd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (csvcifd->bDescriptorType);
  putchar ('\n');
  print ("  bDescriptorSubType : ");
  putx (csvcifd->bDescriptorSubType);
  putchar ('\n');
  print ("  wTotalLength : ");
  putx (csvcifd->wTotalLength);
  putchar ('\n');
  print ("  bInCollection : ");
  putx (csvcifd->bInCollection);
  putchar ('\n');
#endif

#if 0
  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength + 13 +
               csvcifd->bLength]);


  print ("Class-Specific VC Interface Header Descriptor : \n");
  print ("  bLength : ");
  putx (csvcifd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (csvcifd->bDescriptorType);
  putchar ('\n');
  print ("  bDescriptorSubType : ");
  putx (csvcifd->bDescriptorSubType);
  putchar ('\n');
  print ("  wTotalLength : ");
  putx (csvcifd->wTotalLength);
  putchar ('\n');
  print ("  bInCollection : ");
  putx (csvcifd->bInCollection);
  putchar ('\n');
#endif

#if 0
  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength + 13 + 18 +
               csvcifd->bLength]);


  print ("Class-Specific VC Interface Header Descriptor : \n");
  print ("  bLength : ");
  putx (csvcifd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (csvcifd->bDescriptorType);
  putchar ('\n');
  print ("  bDescriptorSubType : ");
  putx (csvcifd->bDescriptorSubType);
  putchar ('\n');
  print ("  wTotalLength : ");
  putx (csvcifd->wTotalLength);
  putchar ('\n');
  print ("  bInCollection : ");
  putx (csvcifd->bInCollection);
  putchar ('\n');
#endif

#if 0
  csvcifd =
    (UVC_CSVC_IF_HDR_DESC
     *) (&conf[cfgd->bLength + iad->bLength + vcifd->bLength + 13 + 18 + 11 +
               csvcifd->bLength]);


  print ("Class-Specific VC Interface Header Descriptor : \n");
  print ("  bLength : ");
  putx (csvcifd->bLength);
  putchar ('\n');
  print ("  bDescriptorType : ");
  putx (csvcifd->bDescriptorType);
  putchar ('\n');
  print ("  bDescriptorSubType : ");
  putx (csvcifd->bDescriptorSubType);
  putchar ('\n');
  print ("  wTotalLength : ");
  putx (csvcifd->wTotalLength);
  putchar ('\n');
  print ("  bInCollection : ");
  putx (csvcifd->bInCollection);
  putchar ('\n');
#endif


#if 1
  memset (conf, 0, 1300);

  status =
    uhci_get_descriptor (1, TYPE_CFG_DESC, 0, 0, 1000, (addr_t) conf);
  print ("Status Code : ");
  putx (status);
  putchar ('\n');

  uint32_t *dump = (uint32_t *) conf;
  int i = 0;

  for (i = 1; i < 200; i++) {
    putx (*dump);
    dump++;
    if (i % 9 == 0)
      putchar ('\n');
  }
#endif

  //uhci_set_interface(1, 0, 0);
  //uhci_set_interface(1, 0, 1);
  //uhci_set_interface(1, 0, 2);
  //uhci_set_interface(1, 0, 3);

#if 1
  uhci_isochronous_transfer (1, 5, (addr_t) iso1, 1023, 1, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso2, 1023, 10, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso3, 1023, 20, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso4, 1023, 30, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso5, 1023, 40, DIR_IN);
  //uhci_isochronous_transfer(1, 4, (addr_t)iso6, 1023, 50, DIR_IN);
#endif

#if 0
  print ("ISO on endpoint 7\n");
  uhci_isochronous_transfer (1, 7, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 7, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 7, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 7, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 7, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 7, (addr_t) iso6, 1023, 50, DIR_IN);
#endif

#if 0
  print ("ISO on endpoint 1\n");
  uhci_isochronous_transfer (1, 1, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 1, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 1, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 1, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 1, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 1, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 2\n");
  uhci_isochronous_transfer (1, 2, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 2, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 2, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 2, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 2, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 2, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 3\n");
  uhci_isochronous_transfer (1, 3, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 3, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 3, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 3, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 3, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 3, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 4\n");
  uhci_isochronous_transfer (1, 4, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 4, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 4, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 4, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 4, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 4, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 5\n");
  uhci_isochronous_transfer (1, 5, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 5, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 5, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 5, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 5, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 5, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 6\n");
  uhci_isochronous_transfer (1, 6, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 6, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 6, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 6, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 6, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 6, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 8\n");
  uhci_isochronous_transfer (1, 8, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 8, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 8, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 8, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 8, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 8, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 9\n");
  uhci_isochronous_transfer (1, 9, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 9, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 9, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 9, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 9, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 9, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 10\n");
  uhci_isochronous_transfer (1, 10, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 10, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 10, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 10, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 10, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 10, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 11\n");
  uhci_isochronous_transfer (1, 11, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 11, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 11, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 11, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 11, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 11, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 12\n");
  uhci_isochronous_transfer (1, 12, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 12, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 12, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 12, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 12, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 12, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 13\n");
  uhci_isochronous_transfer (1, 13, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 13, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 13, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 13, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 13, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 13, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 14\n");
  uhci_isochronous_transfer (1, 14, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 14, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 14, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 14, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 14, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 14, (addr_t) iso6, 1023, 50, DIR_IN);

  print ("ISO on endpoint 15\n");
  uhci_isochronous_transfer (1, 15, (addr_t) iso1, 1023, 1, DIR_IN);
  uhci_isochronous_transfer (1, 15, (addr_t) iso2, 1023, 10, DIR_IN);
  uhci_isochronous_transfer (1, 15, (addr_t) iso3, 1023, 20, DIR_IN);
  uhci_isochronous_transfer (1, 15, (addr_t) iso4, 1023, 30, DIR_IN);
  uhci_isochronous_transfer (1, 15, (addr_t) iso5, 1023, 40, DIR_IN);
  uhci_isochronous_transfer (1, 15, (addr_t) iso6, 1023, 50, DIR_IN);

#endif

#if 1
  delay (1000);
  delay (1000);
  delay (1000);

  i = 0;

  for (i = 1; i < 256; i++) {
    putx ((uint32_t) ((uint32_t *) iso1 + i));
    if (i % 5 == 0)
      putchar ('\n');
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
