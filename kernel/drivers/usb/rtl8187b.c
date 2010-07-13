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

/* The Realtek 8187B is a common USB Wi-Fi chipset */

/* Based on the Linux driver:
 * Copyright 2007 Michael Wu <flamingice@sourmilk.net>
 * Copyright 2007 Andrea Merello <andreamrl@tiscali.it>
 *
 * Based on the r8187 driver, which is:
 * Copyright 2005 Andrea Merello <andreamrl@tiscali.it>, et al.
 *
 * The driver was extended to the RTL8187B in 2008 by:
 *      Herton Ronaldo Krzesinski <herton@mandriva.com.br>
 *	Hin-Tak Leung <htl10@users.sourceforge.net>
 *	Larry Finger <Larry.Finger@lwfinger.net>
 *
 * Magic delays and register offsets below are taken from the original
 * r8187 driver sources.  Thanks to Realtek for their support!
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <drivers/usb/usb.h>
#include <drivers/usb/uhci.h>
#include <drivers/net/ethernet.h>
#include <util/printf.h>
#include <kernel.h>
#include <drivers/eeprom/93cx6.h>
#include "rtl818x.h"

#define DEBUG_RTL8187B

#ifdef DEBUG_RTL8187B
#define DLOG(fmt,...) DLOG_PREFIX("rtl8187b",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static struct { uint16 v, p; char *s; } compat_list[] = {
  { 0x0846, 0x4260, "Netgear WG111v3 [RTL8187B]" },
  { 0xFFFF, 0xFFFF, NULL }
};

#define RTL8187_EEPROM_TXPWR_BASE	0x05
#define RTL8187_EEPROM_MAC_ADDR		0x07
#define RTL8187_EEPROM_TXPWR_CHAN_1	0x16	/* 3 channels */
#define RTL8187_EEPROM_TXPWR_CHAN_6	0x1B	/* 2 channels */
#define RTL8187_EEPROM_TXPWR_CHAN_4	0x3D	/* 2 channels */
#define RTL8187_EEPROM_SELECT_GPIO	0x3B

#define RTL8187_REQ_GET_REG 0x05
#define RTL8187_REQ_SET_REG 0x05
#define RTL8187_REQT_READ   0xC0
#define RTL8187_REQT_WRITE  0x40

#define RTL8187_MAX_RX		0x9C4

#define RFKILL_MASK_8187_89_97	0x2
#define RFKILL_MASK_8198	0x4

static USB_DEVICE_INFO *usbdev;
/* a map of the configuration info on the chipset */
static struct rtl818x_csr *map = (struct rtl818x_csr *)0xFF00;
static struct eeprom_93cx6 eeprom;
static uint8 ethaddr[ETH_ADDR_LEN];

#define udelay(x) tsc_delay_usec (x)

/* ************************************************** */

static int
control_msg (uint8 req, uint8 reqtype, uint16 val, uint16 index,
             uint8 *data, uint16 len)
{
  USB_DEV_REQ setup_req;
  setup_req.bmRequestType = reqtype;
  setup_req.bRequest = req;
  setup_req.wValue = val;
  setup_req.wIndex = index;
  setup_req.wLength = len;

  return usb_control_transfer (usbdev, (addr_t) &setup_req,
                               sizeof (USB_DEV_REQ), data, len);
}

#define iowrite_template(sz)                                            \
  static inline uint##sz                                                \
  ioread##sz##_idx (void* addr, uint8 idx)                              \
  {                                                                     \
    uint##sz val;                                                       \
    control_msg (RTL8187_REQ_GET_REG, RTL8187_REQT_READ,                \
                 (uint) addr, idx & 0x03, (uint8*)&val, sizeof (val));  \
    return val;                                                         \
  }                                                                     \
  static inline uint##sz                                                \
  ioread##sz (void* addr) { return ioread##sz##_idx (addr, 0); }        \
  static inline void                                                    \
  iowrite##sz##_idx (void* addr, uint##sz val, uint8 idx)               \
  {                                                                     \
    control_msg (RTL8187_REQ_SET_REG, RTL8187_REQT_WRITE,               \
                 (uint) addr, idx & 0x03, (uint8*)&val, sizeof (val));  \
  }                                                                     \
  static inline void                                                    \
  iowrite##sz (void* addr, uint##sz val)                                \
  { iowrite##sz##_idx (addr, val, 0); }

iowrite_template(8)
iowrite_template(16)
iowrite_template(32)

/* ************************************************** */

static void
eeprom_read (struct eeprom_93cx6 *eeprom)
{
  uint8 reg = ioread8 (&map->EEPROM_CMD);
  eeprom->reg_data_in = reg & RTL818X_EEPROM_CMD_WRITE;
  eeprom->reg_data_out = reg & RTL818X_EEPROM_CMD_READ;
  eeprom->reg_data_clock = reg & RTL818X_EEPROM_CMD_CK;
  eeprom->reg_chip_select = reg & RTL818X_EEPROM_CMD_CS;
}

static void
eeprom_write (struct eeprom_93cx6 *eeprom)
{
  uint8 reg = RTL818X_EEPROM_CMD_PROGRAM;
  if (eeprom->reg_data_in)
    reg |= RTL818X_EEPROM_CMD_WRITE;
  if (eeprom->reg_data_out)
    reg |= RTL818X_EEPROM_CMD_READ;
  if (eeprom->reg_data_clock)
    reg |= RTL818X_EEPROM_CMD_CK;
  if (eeprom->reg_chip_select)
    reg |= RTL818X_EEPROM_CMD_CS;

  iowrite8 (&map->EEPROM_CMD, reg);
  udelay (10);
}

/* ************************************************** */

static bool
init_hw (void)
{
  uint8 reg;

  iowrite8 (&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8 (&map->CONFIG3);

  DLOG ("CONFIG3=0x%.02X", reg);

  return TRUE;
}

static bool
probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  int i;
  for (i=0; compat_list[i].v != 0xFFFF; i++) {
    if (compat_list[i].v == info->devd.idVendor &&
        compat_list[i].p == info->devd.idProduct)
      break;
  }
  if (compat_list[i].v == 0xFFFF)
    return FALSE;
  DLOG ("Found USB device address=%d: %s", info->address, compat_list[i].s);

  if (usb_set_configuration (info, cfgd->bConfigurationValue) != 0) {
    DLOG ("set_configuration: failed");
    return FALSE;
  }

  usbdev = info;

  eeprom.register_read = eeprom_read;
  eeprom.register_write = eeprom_write;
  if (ioread32 (&map->RX_CONF) & (1 << 6))
    eeprom.width = PCI_EEPROM_WIDTH_93C66;
  else
    eeprom.width = PCI_EEPROM_WIDTH_93C46;

  iowrite8 (&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  udelay (10);
  eeprom_93cx6_multiread (&eeprom, RTL8187_EEPROM_MAC_ADDR,
                          (uint16 *) ethaddr, 3);

  DLOG ("ethaddr=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
        ethaddr[0], ethaddr[1], ethaddr[2],
        ethaddr[3], ethaddr[4], ethaddr[5]);

  return init_hw ();
}

/* ************************************************** */

static USB_DRIVER rtl8187b_usb_driver = {
  .probe = probe
};

extern bool
usb_rtl8187b_driver_init (void)
{
  return usb_register_driver (&rtl8187b_usb_driver);
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
