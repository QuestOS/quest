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

  status = uhci_bulk_transfer (1, ep_in, (addr_t) &csw, 0x0d, maxpkt, DIR_IN);

  DLOG ("status=%d", status);

  if (status != 0) return status;

  DLOG ("csw sig=%p tag=%p res=%d status=%d",
        csw.dCSWSignature, csw.dCSWTag, csw.dCSWDataResidue, csw.bCSWStatus);

  return status;
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
