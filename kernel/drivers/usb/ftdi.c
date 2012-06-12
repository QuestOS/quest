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
#include <drivers/usb/ftdi.h>
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_FTDI

#ifdef DEBUG_FTDI
#define DLOG(fmt,...) DLOG_PREFIX("ftdi",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* The Base Clock of the chip. It is either 12000000 or 48000000. */
#define FTDI_BASE_CLOCK    48000000

static USB_DEVICE_INFO ftdi_dev;
static uint8_t in_ept = 0, out_ept = 0;

bool usb_ftdi_driver_init (void);
void usb_ftdi_putc (char);
char usb_ftdi_getc (void);
int usb_ftdi_write (unsigned char *, uint32_t);
int usb_ftdi_read (unsigned char *, uint32_t);

/*
 * Reset FTDI chip. Always set port to 0? The Linux driver
 * says this is 0 for FT232/245. For now, we are supposed to
 * support only FT232.
 */
static int
ftdi_reset (USB_DEVICE_INFO * dev, uint16_t port)
{
  USB_DEV_REQ req;
  req.bmRequestType = 0x40;
  req.bRequest = USB_FTDI_RESET;
  /* wValue = 0  :  Reset
   * wValue = 1  :  Purge RX buffer
   * wValue = 2  :  Purge TX buffer
   */
  req.wValue = 0;
  req.wIndex = port;
  req.wLength = 0;

  return usb_control_transfer (dev, (addr_t) & req,
      sizeof (USB_DEV_REQ), 0, 0);
}

/* Get the divisor which is later used to set the baud rate */
/* This function is from the Linux FTDI driver */
static uint32_t
ftdi_baud_base_to_divisor (uint32_t baud, uint32_t base)
{
  static const unsigned char divfrac[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
  uint32_t divisor;
  /* divisor shifted 3 bits to the left */
  int divisor3 = base / 2 / baud;
  divisor = divisor3 >> 3;
  divisor |= (uint32_t)divfrac[divisor3 & 0x7] << 14;
  /* Deal with special cases for highest baud rates. */
  if (divisor == 1)
    divisor = 0;
  else if (divisor == 0x4001)
    divisor = 1;
  return divisor;
}

static int
ftdi_set_baud (USB_DEVICE_INFO * dev, uint32_t baud, uint16_t port)
{
  USB_DEV_REQ req;
  uint32_t divisor = ftdi_baud_base_to_divisor (baud, FTDI_BASE_CLOCK);

  req.bmRequestType = 0x40;
  req.bRequest = USB_FTDI_SET_BAUD_RATE;
  req.wValue = divisor;
  req.wIndex = port;
  req.wLength = 0;

  return usb_control_transfer (dev, (addr_t) & req,
      sizeof (USB_DEV_REQ), 0, 0);
}

/* Setting FTDI data characteristics */
static int
ftdi_set_data (
    USB_DEVICE_INFO * dev,
    uint8_t bits, /* Number of data bits */
    uint8_t parity, /* 0 = None, 1 = Odd, 2 = Even, 3 = Mark, 4 = Space */
    uint8_t stb, /* Stop Bits 0 = 1, 1 = 1.5, 2 = 2 */
    uint8_t tx, /* 1 = TX ON (break), 0 = TX OFF (normal state) */
    uint16_t port)
{
  USB_DEV_REQ req;
  uint16_t dc = (tx & 0x1) << 14 |
    (stb & 0x7) << 11 | (parity & 0x7) << 8 | bits;

  req.bmRequestType = 0x40;
  req.bRequest = USB_FTDI_SET_DATA;
  req.wValue = dc;
  req.wIndex = port;
  req.wLength = 0;

  return usb_control_transfer (dev, (addr_t) & req,
      sizeof (USB_DEV_REQ), 0, 0);
}

static uint16_t
ftdi_modem_status (USB_DEVICE_INFO * dev, uint16_t port)
{
  USB_DEV_REQ req;
  uint16_t status;

  req.bmRequestType = 0xC0;
  req.bRequest = USB_FTDI_GET_MODEM_STATUS;
  req.wValue = 0;
  req.wIndex = port;
  req.wLength = 2;

  if (usb_control_transfer (dev, (addr_t) & req,
        sizeof (USB_DEV_REQ), (addr_t) & status, req.wLength)) {
    DLOG ("Get modem status failed");
    return 0xFFFF;
  }

  return status;
}

static int
ftdi_set_flow_ctl (USB_DEVICE_INFO * dev, uint16_t port, uint8_t ctrl)
{
  USB_DEV_REQ req;

  req.bmRequestType = 0x40;
  req.bRequest = USB_FTDI_SET_FLOW_CTRL;
  req.wValue = 0;
  req.wIndex = (ctrl << 8) + port;
  req.wLength = 0;

  return usb_control_transfer (dev, (addr_t) & req,
      sizeof (USB_DEV_REQ), 0, 0);
}

/* Set the timeout interval. The default is 16 ms */
static int
ftdi_set_latency (USB_DEVICE_INFO * dev, uint16_t port, uint8_t latency)
{
  USB_DEV_REQ req;

  req.bmRequestType = 0x40;
  req.bRequest = USB_FTDI_SET_LATENCY_TIMER;
  req.wValue = latency; /* Latency in milliseconds */
  req.wIndex = port;
  req.wLength = 0;

  return usb_control_transfer (dev, (addr_t) & req,
      sizeof (USB_DEV_REQ), 0, 0);
}

static uint8_t
ftdi_get_latency (USB_DEVICE_INFO * dev, uint16_t port)
{
  USB_DEV_REQ req;
  uint8_t latency = 0;

  req.bmRequestType = 0xC0;
  req.bRequest = USB_FTDI_GET_LATENCY_TIMER;
  req.wValue = 0;
  req.wIndex = port;
  req.wLength = 1;

  if (usb_control_transfer (dev, (addr_t) & req,
      sizeof (USB_DEV_REQ), (addr_t) & latency, 1)) {
    DLOG ("Getting latency timer failed!");
    return 0;
  }

  return latency;
}

static bool
ftdi_init (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  uint8_t tmp[50];
  int i = 0;
  uint16_t mod_stat = 0;
  uint8_t latency = 0;
  USB_EPT_DESC *ftdiept;

  ftdi_dev = *(dev);
  memset (tmp, 0, 50);

  /* Parsing endpoints */
  DLOG ("Parsing FTDI chip endpoints ...");
  usb_get_descriptor (dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength,
      (addr_t)tmp);
  for (i = 0; i < cfg->wTotalLength; i += ftdiept->bLength) {
    ftdiept = (USB_EPT_DESC *) (tmp + i);
    if ((ftdiept->bDescriptorType == USB_TYPE_EPT_DESC) &&
        ((ftdiept->bmAttributes & 0x3) == 0x2)) {
      DLOG ("Found Bulk Endpoint");
      switch (ftdiept->bEndpointAddress & 0x80) {
        case 0x80 :
          in_ept = ftdiept->bEndpointAddress & 0xF;
          DLOG ("IN Endpoint. Address is: 0x%X", in_ept);
          break;

        case 0x00 :
          out_ept = ftdiept->bEndpointAddress & 0xF;
          DLOG ("OUT Endpoint. Address is: 0x%X", out_ept);
          break;

        default :
          break;
      }
    }
  }

  /* Reset the chip */
  DLOG ("Resetting FTDI chip ...");
  if (ftdi_reset (dev, 0)) {
    DLOG ("Chip reset failed!");
    return FALSE;
  }

  /* Set the baud rate for the serial port */
  DLOG ("Setting baud rate ...");
  if (ftdi_set_baud (dev, 38400, 0)) {
    DLOG ("Setting baud rate failed!");
    return FALSE;
  }

  /* Set data characteristics */
  DLOG ("Setting data characteristics ...");
  if (ftdi_set_data (dev, 8, 0, 0, 0, 0)) {
    DLOG ("Setting data characteristics failed!");
    return FALSE;
  }

  /* Set timeout interval */
  DLOG ("Setting latency timer ...");
  if (ftdi_set_latency (dev, 0, 16)) {
    DLOG ("Setting latency timer failed!");
    return FALSE;
  }

  /* Get timeout interval */
  DLOG ("Getting latency timer ...");
  if ((latency = ftdi_get_latency (dev, 0))) {
    DLOG ("Current timeout interval is: %d", latency);
  }

  /* Set flow control */
  DLOG ("Setting flow control ...");
  if (ftdi_set_flow_ctl (dev, 0, USB_FTDI_NO_FLOW_CTRL)) {
    DLOG ("Setting flow control failed!");
    return FALSE;
  }
#if 1
  /* Get modem status */
  DLOG ("Getting modem status ...");
  if ((mod_stat = ftdi_modem_status (dev, 0)) == 0xFFFF) {
    DLOG ("Getting modem status failed!");
    return FALSE;
  } else {
    DLOG ("Modem status is: 0x%X", mod_stat);
  }
#endif
  return TRUE;
}

static void
ftdi_test (void)
{
  DLOG ("FTDI Tests");
  char c = 0;

  usb_ftdi_putc ('!');

  while (c != 0xD) {
    c = usb_ftdi_getc ();
    DLOG ("Got character : %c", c);
    usb_ftdi_putc (c);
  }
}

static bool
ftdi_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  if (dev->devd.idVendor == 0x0403) {
    DLOG ("An FTDI chip is detected");

    if (dev->devd.idProduct != 0x6001) {
      DLOG ("FT232 not found. Product ID is: 0x%X",
          dev->devd.idProduct);
      return FALSE;
    }
  } else {
    return FALSE;
  }

  if (!ftdi_init(dev, cfg, ifd)) {
    DLOG("Initialization failed!");
    return FALSE;
  }

  DLOG ("FTDI Serial Converter configured");
  //ftdi_test();

  return TRUE;
}

char
usb_ftdi_getc (void)
{
  unsigned char buf[3];
  int act_len = 0;

  if ((act_len = usb_ftdi_read (buf, 3)) != 3) {
    DLOG ("usb_ftdi_read () failed. %d bytes returned.", act_len);
    return '\0';
  }

  return buf[2];
}

void
usb_ftdi_putc (char c)
{
  unsigned char buf[2];
  int count = 0, buf_size = 0;

#if 0
  /* This is not necessary for newer chips */
  /* First byte is reserved, it contains length and Port ID */
  /* B0 = 1, B1 = 0, B2..7 = Length of Message */
  buf[0] = (1 << 2) | 1;
  /* Actual data to be transfered */
  buf[1] = c;
  buf_size = 2;
#else
  buf[0] = c;
  buf_size = 1;
#endif

  if ((count = usb_ftdi_write (buf, buf_size)) != buf_size)
    DLOG ("usb_ftdi_putc failed, %d bytes sent", count);
  else
    DLOG ("usb_ftdi_putc done, %d bytes sent", count);
}

int
usb_ftdi_write (unsigned char * buf, uint32_t len)
{
  uint32_t act_len = 0;
  int status = 0;

  if ((status = usb_bulk_transfer (&ftdi_dev, out_ept, (addr_t) buf,
        len, 64, USB_DIR_OUT, &act_len)))
    DLOG ("Bulk write failed. Error Code: 0x%X", status);
  
  return act_len;
}

/*
 * First two bytes read contains modem status and line status.
 *
 * Byte 0: Modem Status
 *
 * Offset       Description
 * B0   Reserved - must be 1
 * B1   Reserved - must be 0
 * B2   Reserved - must be 0
 * B3   Reserved - must be 0
 * B4   Clear to Send (CTS)
 * B5   Data Set Ready (DSR)
 * B6   Ring Indicator (RI)
 * B7   Receive Line Signal Detect (RLSD)
 *
 * Byte 1: Line Status
 *
 * Offset       Description
 * B0   Data Ready (DR)
 * B1   Overrun Error (OE)
 * B2   Parity Error (PE)
 * B3   Framing Error (FE)
 * B4   Break Interrupt (BI)
 * B5   Transmitter Holding Register (THRE)
 * B6   Transmitter Empty (TEMT)
 * B7   Error in RCVR FIFO
 */
int
usb_ftdi_read (unsigned char * buf, uint32_t len)
{
  uint32_t act_len = 0;
  int status = 0;

  if ((status = usb_bulk_transfer (&ftdi_dev, in_ept, (addr_t) buf,
        len, 64, USB_DIR_IN, &act_len)))
    DLOG ("Bulk read failed. Error Code: 0x%X", status);
  
  return act_len;
}

static USB_DRIVER ftdi_driver = {
  .probe = ftdi_probe
};

bool
usb_ftdi_driver_init (void)
{
  return usb_register_driver (&ftdi_driver);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_ftdi_driver_init
};

//DEF_MODULE (usb___ftdi, "USB FTDI driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
