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

/* USB Mass Storage Class driver */
#include "drivers/usb/usb.h"
#include "drivers/usb/uhci.h"
#include "util/printf.h"
#include "kernel.h"

#define USB_MASS_STORAGE_CLASS 0x8
#define UMSC_PROTOCOL 0x50

#define DEBUG_UMSC

#ifdef DEBUG_UMSC
#define DLOG(fmt,...) DLOG_PREFIX("umsc",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

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
umsc_bulk_scsi (uint addr, uint ep_out, uint ep_in,
                uint8 cmd[16], uint dir, uint8* data,
                uint data_len, uint maxpkt)
{
  UMSC_CBW cbw;
  UMSC_CSW csw;
  sint status;

  DLOG ("cmd: %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X %.02X",
        cmd[0], cmd[1], cmd[2], cmd[3],
        cmd[4], cmd[5], cmd[6], cmd[7],
        cmd[8], cmd[9], cmd[10], cmd[11],
        cmd[12], cmd[13], cmd[14], cmd[15]);

  memset (&cbw, 0, sizeof (cbw));
  memset (&csw, 0, sizeof (csw));

  cbw.dCBWSignature = UMSC_CBW_SIGNATURE;
  cbw.dCBWDataTransferLength = data_len;
  cbw.bmCBWFlags.direction = dir;
  cbw.bCBWCBLength = 16;            /* cmd length */
  memcpy (cbw.CBWCB, cmd, 16);

  status = uhci_bulk_transfer (addr, ep_out, &cbw, 0x1f, maxpkt, DIR_OUT);

  DLOG ("status=%d", status);

  if (data_len > 0) {
    if (dir) {
      status = uhci_bulk_transfer (addr, ep_in, data, data_len, maxpkt, DIR_IN);
    }
    else {
      status = uhci_bulk_transfer (addr, ep_out, data, data_len, maxpkt, DIR_OUT);
    }

    DLOG ("status=%d", status);

    if (status != 0) return status;

    DLOG ("data=%.02X %.02X %.02X %.02X", data[0], data[1], data[2], data[3]);

  }

  status = uhci_bulk_transfer (addr, ep_in, (addr_t) &csw, 0x0d, maxpkt, DIR_IN);

  DLOG ("status=%d", status);

  if (status != 0) return status;

  DLOG ("csw sig=%p tag=%p res=%d status=%d",
        csw.dCSWSignature, csw.dCSWTag, csw.dCSWDataResidue, csw.bCSWStatus);

  return status;
}

static bool
umsc_probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  uint i, addr = info->address, ep_in=0, ep_out=0;
  USB_EPT_DESC *ep;
  uint last_lba, sector_size, maxpkt=64;
  uint8 conf[512];

  if (ifd->bInterfaceClass != USB_MASS_STORAGE_CLASS ||
      ifd->bInterfaceProtocol != UMSC_PROTOCOL)
    return FALSE;

  ep = (USB_EPT_DESC *)(&ifd[1]);

  if (ep[0].wMaxPacketSize != ep[1].wMaxPacketSize) {
    DLOG ("endpoint packet sizes don't match!");
    return FALSE;
  }
  maxpkt = ep[0].wMaxPacketSize;
  for (i=0; i<ifd->bNumEndpoints; i++)
    if (ep[i].bEndpointAddress & 0x80)
      ep_in = ep[i].bEndpointAddress & 0x7F;
    else
      ep_out = ep[i].bEndpointAddress & 0x7F;

  if (!(ep_in && ep_out))
    return FALSE;

  DLOG ("detected device=%d ep_in=%d ep_out=%d maxpkt=%d",
        addr, ep_in, ep_out, maxpkt);

  uhci_set_configuration (addr, cfgd->bConfigurationValue);
  delay (50);


  {
    uint8 cmd[16] = {0x12,0,0,0,0x24,0,0,0,0,0,0,0};
    DLOG ("SENDING INQUIRY");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 0x24, maxpkt);
  }

  {
    uint8 cmd[16] = {0,0,0,0,0,0,0,0,0,0,0,0};
    DLOG ("SENDING TEST UNIT READY");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 0, maxpkt);
  }

  {
    uint8 cmd[16] = {0x03,0,0,0,0x24,0,0,0,0,0,0,0};
    DLOG ("SENDING REQUEST SENSE");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 0x24, maxpkt);
  }

  {
    uint8 cmd[16] = {0,0,0,0,0,0,0,0,0,0,0,0};
    DLOG ("SENDING TEST UNIT READY");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 0, maxpkt);
  }

  {
    uint8 cmd[16] = {0x03,0,0,0,0x24,0,0,0,0,0,0,0};
    DLOG ("SENDING REQUEST SENSE");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 0x24, maxpkt);
  }

  {
    uint8 cmd[16] = {0x25,0,0,0,0,0,0,0,0,0,0,0};
    DLOG ("SENDING READ CAPACITY");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 0x8, maxpkt);
    last_lba = conf[3] | conf[2] << 8 | conf[1] << 16 | conf[0] << 24;
    sector_size = conf[7] | conf[6] << 8 | conf[5] << 16 | conf[4] << 24;
    DLOG ("sector_size=0x%x last_lba=0x%x total_size=%d bytes",
            sector_size, last_lba, sector_size * (last_lba + 1));
  }

  {
    uint8 cmd[16] = { [0] = 0x28, [8] = 1 };
    DLOG ("SENDING READ (10)");
    umsc_bulk_scsi (addr, ep_out, ep_in, cmd, 1, conf, 512, maxpkt);
    DLOG ("read from sector 0: %.02X %.02X %.02X %.02X",
          conf[0], conf[1], conf[2], conf[3]);
  }

  return TRUE;
}

static USB_DRIVER umsc_driver = {
  .probe = umsc_probe
};

extern bool
usb_mass_storage_driver_init (void)
{
  return usb_register_driver (&umsc_driver);
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
