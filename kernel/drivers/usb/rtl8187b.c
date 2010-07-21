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
#include <drivers/net/ieee80211.h>
#include <drivers/net/mac80211.h>
#include <drivers/net/ieee80211_standard.h>
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

static const struct ieee80211_rate rtl818x_rates[] = {
  { .bitrate = 10, .hw_value = 0, },
  { .bitrate = 20, .hw_value = 1, },
  { .bitrate = 55, .hw_value = 2, },
  { .bitrate = 110, .hw_value = 3, },
  { .bitrate = 60, .hw_value = 4, },
  { .bitrate = 90, .hw_value = 5, },
  { .bitrate = 120, .hw_value = 6, },
  { .bitrate = 180, .hw_value = 7, },
  { .bitrate = 240, .hw_value = 8, },
  { .bitrate = 360, .hw_value = 9, },
  { .bitrate = 480, .hw_value = 10, },
  { .bitrate = 540, .hw_value = 11, },
};

static const struct ieee80211_channel rtl818x_channels[] = {
  { .center_freq = 2412 },
  { .center_freq = 2417 },
  { .center_freq = 2422 },
  { .center_freq = 2427 },
  { .center_freq = 2432 },
  { .center_freq = 2437 },
  { .center_freq = 2442 },
  { .center_freq = 2447 },
  { .center_freq = 2452 },
  { .center_freq = 2457 },
  { .center_freq = 2462 },
  { .center_freq = 2467 },
  { .center_freq = 2472 },
  { .center_freq = 2484 },
};

#define ETH_ALEN ETH_ADDR_LEN

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

#define RTL8187_RTL8225_ANAPARAM_ON	0xa0000a59
#define RTL8187_RTL8225_ANAPARAM2_ON	0x860c7312
#define RTL8187_RTL8225_ANAPARAM_OFF	0xa00beb59
#define RTL8187_RTL8225_ANAPARAM2_OFF	0x840dec11

#define RTL8187B_RTL8225_ANAPARAM_ON	0x45090658
#define RTL8187B_RTL8225_ANAPARAM2_ON	0x727f3f52
#define RTL8187B_RTL8225_ANAPARAM3_ON	0x00
#define RTL8187B_RTL8225_ANAPARAM_OFF	0x55480658
#define RTL8187B_RTL8225_ANAPARAM2_OFF	0x72003f50
#define RTL8187B_RTL8225_ANAPARAM3_OFF	0x00

static USB_DEVICE_INFO *usbdev;
/* a map of the configuration info on the chipset */
static struct rtl818x_csr *map = (struct rtl818x_csr *)0xFF00;
static struct eeprom_93cx6 eeprom;
static uint8 rfkill_mask;
static uint8 ethaddr[ETH_ADDR_LEN];
static bool is_rtl8187b;
static enum {
  RTL8187BvB, RTL8187BvD, RTL8187BvE
} hw_rev;
static struct ieee80211_channel channels[14];
static struct ieee80211_rate rates[12];
static const struct rtl818x_rf_ops *rf;
static uint8 slot_time, aifsn[4];

#define udelay(x) tsc_delay_usec (x)
#define msleep(x) sched_usleep (x*1000);

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

void
rtl8187_write_phy(u8 addr, u32 data)
{
  data <<= 8;
  data |= addr | 0x80;

  iowrite8(&map->PHY[3], (data >> 24) & 0xFF);
  iowrite8(&map->PHY[2], (data >> 16) & 0xFF);
  iowrite8(&map->PHY[1], (data >> 8) & 0xFF);
  iowrite8(&map->PHY[0], data & 0xFF);
}

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
is_radio_enabled (void)
{
  u8 gpio;
  gpio = ioread8 (&map->GPIO0);
  iowrite8 (&map->GPIO0, gpio & ~rfkill_mask);
  gpio = ioread8 (&map->GPIO1);

  return gpio & rfkill_mask;
}

static void
rfkill_init (void)
{
  bool enabled = is_radio_enabled ();
  DLOG ("wireless radio switch is %s",
        enabled ? "on" : "off");

  /* FIXME: what to do... */
  /* wiphy_rfkill_set_hwstate (hw->wiphy, !rfkill_off); */
}

/* ************************************************** */


static bool
add_iface (struct ieee80211_hw *dev,
           struct ieee80211_if_init_conf *conf)
{
  int i;
  iowrite8 (&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  for (i=0; i<ETH_ADDR_LEN; i++)
    iowrite8 (&map->MAC[i], ethaddr[i]);
  iowrite8 (&map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

  return TRUE;
}

static void
rem_iface (struct ieee80211_hw *dev,
           struct ieee80211_if_init_conf *conf)
{
  DLOG ("rem_iface");
}

static bool
config (struct ieee80211_hw *dev, u32 changed)
{
  struct ieee80211_conf *conf = &dev->conf;
  u32 reg;
  reg = ioread32 (&map->TX_CONF);
  /* Enable TX loopback during channel changes to avoid TX */
  iowrite32 (&map->TX_CONF, reg | RTL818X_TX_CONF_LOOPBACK_MAC);
  DLOG ("setting channel to freq=%d", conf->channel->center_freq);
  rf->set_chan (conf);
  msleep (10);
  iowrite32 (&map->TX_CONF, reg);

  iowrite16 (&map->ATIM_WND, 2);
  iowrite16 (&map->ATIMTR_INTERVAL, 100);
  iowrite16 (&map->BEACON_INTERVAL, 100);
  iowrite16 (&map->BEACON_INTERVAL_TIME, 100);

  return TRUE;
}

static __le32 *rtl8187b_ac_addr[4] = {
  (__le32 *) 0xFFF0, /* AC_VO */
  (__le32 *) 0xFFF4, /* AC_VI */
  (__le32 *) 0xFFFC, /* AC_BK */
  (__le32 *) 0xFFF8, /* AC_BE */
};
#define SIFS_TIME 0xa
#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

static void
conf_erp(bool use_short_slot,
         bool use_short_preamble)
{
  if (is_rtl8187b) {
    u8 difs, eifs;
    u16 ack_timeout;
    int queue;

    if (use_short_slot) {
      slot_time = 0x9;
      difs = 0x1c;
      eifs = 0x53;
    } else {
      slot_time = 0x14;
      difs = 0x32;
      eifs = 0x5b;
    }
    iowrite8(&map->SIFS, 0x22);
    iowrite8(&map->SLOT, slot_time);
    iowrite8(&map->DIFS, difs);

    /*
     * BRSR+1 on 8187B is in fact EIFS register
     * Value in units of 4 us
     */
    iowrite8((u8 *)&map->BRSR + 1, eifs);

    /*
     * For 8187B, CARRIER_SENSE_COUNTER is in fact ack timeout
     * register. In units of 4 us like eifs register
     * ack_timeout = ack duration + plcp + difs + preamble
     */
    ack_timeout = 112 + 48 + difs;
    if (use_short_preamble)
      ack_timeout += 72;
    else
      ack_timeout += 144;
    iowrite8(&map->CARRIER_SENSE_COUNTER,
             DIV_ROUND_UP(ack_timeout, 4));

    for (queue = 0; queue < 4; queue++)
      iowrite8((u8 *) rtl8187b_ac_addr[queue],
               aifsn[queue] * slot_time +
               SIFS_TIME);
  } else {
    iowrite8(&map->SIFS, 0x22);
    if (use_short_slot) {
      iowrite8(&map->SLOT, 0x9);
      iowrite8(&map->DIFS, 0x14);
      iowrite8(&map->EIFS, 91 - 0x14);
    } else {
      iowrite8(&map->SLOT, 0x14);
      iowrite8(&map->DIFS, 0x24);
      iowrite8(&map->EIFS, 91 - 0x24);
    }
  }
}

static void
bss_info_changed(struct ieee80211_hw *dev,
                 struct ieee80211_vif *vif,
                 struct ieee80211_bss_conf *info,
                 u32 changed)
{
  int i;
  u8 reg;

  if (changed & BSS_CHANGED_BSSID) {
    for (i = 0; i < ETH_ALEN; i++)
      iowrite8(&map->BSSID[i],
               info->bssid[i]);

    if (is_rtl8187b)
      reg = RTL818X_MSR_ENEDCA;
    else
      reg = 0;

    if (is_valid_ether_addr(info->bssid)) {
      reg |= RTL818X_MSR_INFRA;
      iowrite8(&map->MSR, reg);
    } else {
      reg |= RTL818X_MSR_NO_LINK;
      iowrite8(&map->MSR, reg);
    }

  }

  if (changed & (BSS_CHANGED_ERP_SLOT | BSS_CHANGED_ERP_PREAMBLE))
    conf_erp(info->use_short_slot,
             info->use_short_preamble);
}


static bool
conf_tx (struct ieee80211_hw *dev,
         u16 queue, const struct ieee80211_tx_queue_params *params)
{
  u8 cw_min, cw_max;

  if (queue > 3)
    return FALSE;

  cw_min = fls(params->cw_min);
  cw_max = fls(params->cw_max);

  if (is_rtl8187b) {
    aifsn[queue] = params->aifs;

    /*
     * This is the structure of AC_*_PARAM registers in 8187B:
     * - TXOP limit field, bit offset = 16
     * - ECWmax, bit offset = 12
     * - ECWmin, bit offset = 8
     * - AIFS, bit offset = 0
     */
    iowrite32(rtl8187b_ac_addr[queue],
              (params->txop << 16) | (cw_max << 12) |
              (cw_min << 8) | (params->aifs *
                               slot_time + SIFS_TIME));
  } else {
    if (queue != 0)
      return FALSE;

    iowrite8(&map->CW_VAL,
             cw_min | (cw_max << 4));
  }
  return TRUE;
}

static bool
cmd_reset (void)
{
  u8 reg;
  int i;

  reg = ioread8(&map->CMD);
  reg &= (1 << 1);
  reg |= RTL818X_CMD_RESET;
  iowrite8(&map->CMD, reg);

  i = 10;
  do {
    msleep(2);
    if (!(ioread8(&map->CMD) &
          RTL818X_CMD_RESET))
      break;
  } while (--i);

  if (!i) {
    DLOG("Reset timeout!");
    return FALSE;
  }

  /* reload registers from eeprom */
  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_LOAD);

  i = 10;
  do {
    msleep(4);
    if (!(ioread8(&map->EEPROM_CMD) &
          RTL818X_EEPROM_CMD_CONFIG))
      break;
  } while (--i);

  if (!i) {
    DLOG("eeprom reset timeout!");
    return FALSE;
  }

  DLOG ("reset success");
  return TRUE;
}

static const u8 rtl8187b_reg_table[][3] = {
  {0xF0, 0x32, 0}, {0xF1, 0x32, 0}, {0xF2, 0x00, 0}, {0xF3, 0x00, 0},
  {0xF4, 0x32, 0}, {0xF5, 0x43, 0}, {0xF6, 0x00, 0}, {0xF7, 0x00, 0},
  {0xF8, 0x46, 0}, {0xF9, 0xA4, 0}, {0xFA, 0x00, 0}, {0xFB, 0x00, 0},
  {0xFC, 0x96, 0}, {0xFD, 0xA4, 0}, {0xFE, 0x00, 0}, {0xFF, 0x00, 0},

  {0x58, 0x4B, 1}, {0x59, 0x00, 1}, {0x5A, 0x4B, 1}, {0x5B, 0x00, 1},
  {0x60, 0x4B, 1}, {0x61, 0x09, 1}, {0x62, 0x4B, 1}, {0x63, 0x09, 1},
  {0xCE, 0x0F, 1}, {0xCF, 0x00, 1}, {0xE0, 0xFF, 1}, {0xE1, 0x0F, 1},
  {0xE2, 0x00, 1}, {0xF0, 0x4E, 1}, {0xF1, 0x01, 1}, {0xF2, 0x02, 1},
  {0xF3, 0x03, 1}, {0xF4, 0x04, 1}, {0xF5, 0x05, 1}, {0xF6, 0x06, 1},
  {0xF7, 0x07, 1}, {0xF8, 0x08, 1},

  {0x4E, 0x00, 2}, {0x0C, 0x04, 2}, {0x21, 0x61, 2}, {0x22, 0x68, 2},
  {0x23, 0x6F, 2}, {0x24, 0x76, 2}, {0x25, 0x7D, 2}, {0x26, 0x84, 2},
  {0x27, 0x8D, 2}, {0x4D, 0x08, 2}, {0x50, 0x05, 2}, {0x51, 0xF5, 2},
  {0x52, 0x04, 2}, {0x53, 0xA0, 2}, {0x54, 0x1F, 2}, {0x55, 0x23, 2},
  {0x56, 0x45, 2}, {0x57, 0x67, 2}, {0x58, 0x08, 2}, {0x59, 0x08, 2},
  {0x5A, 0x08, 2}, {0x5B, 0x08, 2}, {0x60, 0x08, 2}, {0x61, 0x08, 2},
  {0x62, 0x08, 2}, {0x63, 0x08, 2}, {0x64, 0xCF, 2}, {0x72, 0x56, 2},
  {0x73, 0x9A, 2},

  {0x34, 0xF0, 0}, {0x35, 0x0F, 0}, {0x5B, 0x40, 0}, {0x84, 0x88, 0},
  {0x85, 0x24, 0}, {0x88, 0x54, 0}, {0x8B, 0xB8, 0}, {0x8C, 0x07, 0},
  {0x8D, 0x00, 0}, {0x94, 0x1B, 0}, {0x95, 0x12, 0}, {0x96, 0x00, 0},
  {0x97, 0x06, 0}, {0x9D, 0x1A, 0}, {0x9F, 0x10, 0}, {0xB4, 0x22, 0},
  {0xBE, 0x80, 0}, {0xDB, 0x00, 0}, {0xEE, 0x00, 0}, {0x4C, 0x00, 2},

  {0x9F, 0x00, 3}, {0x8C, 0x01, 0}, {0x8D, 0x10, 0}, {0x8E, 0x08, 0},
  {0x8F, 0x00, 0}
};

static bool
init_hw (void)
{
  uint8 reg;
  bool res;
  int i;

  DLOG ("init_hw");

  iowrite8 (&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8 (&map->CONFIG3);

  reg |= RTL818X_CONFIG3_ANAPARAM_WRITE | RTL818X_CONFIG3_GNT_SELECT;
  iowrite8(&map->CONFIG3, reg);
  iowrite32(&map->ANAPARAM2,
                    RTL8187B_RTL8225_ANAPARAM2_ON);
  iowrite32(&map->ANAPARAM,
                    RTL8187B_RTL8225_ANAPARAM_ON);
  iowrite8(&map->ANAPARAM3,
                   RTL8187B_RTL8225_ANAPARAM3_ON);

  iowrite8((u8 *)0xFF61, 0x10);
  reg = ioread8((u8 *)0xFF62);
  iowrite8((u8 *)0xFF62, reg & ~(1 << 5));
  iowrite8((u8 *)0xFF62, reg | (1 << 5));

  reg = ioread8(&map->CONFIG3);
  reg &= ~RTL818X_CONFIG3_ANAPARAM_WRITE;
  iowrite8(&map->CONFIG3, reg);

  iowrite8(&map->EEPROM_CMD,
                   RTL818X_EEPROM_CMD_NORMAL);

  res = cmd_reset();
  if (!res)
    return res;

  iowrite16((__le16 *)0xFF2D, 0x0FFF);
  reg = ioread8(&map->CW_CONF);
  reg |= RTL818X_CW_CONF_PERPACKET_RETRY_SHIFT;
  iowrite8(&map->CW_CONF, reg);
  reg = ioread8(&map->TX_AGC_CTL);
  reg |= RTL818X_TX_AGC_CTL_PERPACKET_GAIN_SHIFT |
    RTL818X_TX_AGC_CTL_PERPACKET_ANTSEL_SHIFT;
  iowrite8(&map->TX_AGC_CTL, reg);

  iowrite16_idx((__le16 *)0xFFE0, 0x0FFF, 1);

  iowrite16(&map->BEACON_INTERVAL, 100);
  iowrite16(&map->ATIM_WND, 2);
  iowrite16_idx((__le16 *)0xFFD4, 0xFFFF, 1);

  iowrite8(&map->EEPROM_CMD,
                   RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8(&map->CONFIG1);
  iowrite8(&map->CONFIG1, (reg & 0x3F) | 0x80);
  iowrite8(&map->EEPROM_CMD,
                   RTL818X_EEPROM_CMD_NORMAL);

  iowrite8(&map->WPA_CONF, 0);
  for (i = 0; i < ARRAY_SIZE(rtl8187b_reg_table); i++) {
    iowrite8_idx(        (u8 *)(u32)
                         (rtl8187b_reg_table[i][0] | 0xFF00),
                         rtl8187b_reg_table[i][1],
                         rtl8187b_reg_table[i][2]);
  }

  iowrite16(&map->TID_AC_MAP, 0xFA50);
  iowrite16(&map->INT_MIG, 0);

  iowrite32_idx((__le32 *)0xFFF0, 0, 1);
  iowrite32_idx((__le32 *)0xFFF4, 0, 1);
  iowrite8_idx((u8 *)0xFFF8, 0, 1);

  iowrite32(&map->RF_TIMING, 0x00004001);

  iowrite16_idx((__le16 *)0xFF72, 0x569A, 2);

  iowrite8(&map->EEPROM_CMD,
                   RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8(&map->CONFIG3);
  reg |= RTL818X_CONFIG3_ANAPARAM_WRITE;
  iowrite8(&map->CONFIG3, reg);
  iowrite8(&map->EEPROM_CMD,
                   RTL818X_EEPROM_CMD_NORMAL);

  iowrite16(&map->RFPinsOutput, 0x0480);
  iowrite16(&map->RFPinsSelect, 0x2488);
  iowrite16(&map->RFPinsEnable, 0x1FFF);
  msleep(100);

  rf->init();

  reg = RTL818X_CMD_TX_ENABLE | RTL818X_CMD_RX_ENABLE;
  iowrite8(&map->CMD, reg);
  iowrite16(&map->INT_MASK, 0xFFFF);

  iowrite8((u8 *)0xFE41, 0xF4);
  iowrite8((u8 *)0xFE40, 0x00);
  iowrite8((u8 *)0xFE42, 0x00);
  iowrite8((u8 *)0xFE42, 0x01);
  iowrite8((u8 *)0xFE40, 0x0F);
  iowrite8((u8 *)0xFE42, 0x00);
  iowrite8((u8 *)0xFE42, 0x01);

  reg = ioread8((u8 *)0xFFDB);
  iowrite8((u8 *)0xFFDB, reg | (1 << 2));
  iowrite16_idx((__le16 *)0xFF72, 0x59FA, 3);
  iowrite16_idx((__le16 *)0xFF74, 0x59D2, 3);
  iowrite16_idx((__le16 *)0xFF76, 0x59D2, 3);
  iowrite16_idx((__le16 *)0xFF78, 0x19FA, 3);
  iowrite16_idx((__le16 *)0xFF7A, 0x19FA, 3);
  iowrite16_idx((__le16 *)0xFF7C, 0x00D0, 3);
  iowrite8((u8 *)0xFF61, 0);
  iowrite8_idx((u8 *)0xFF80, 0x0F, 1);
  iowrite8_idx((u8 *)0xFF83, 0x03, 1);
  iowrite8((u8 *)0xFFDA, 0x10);
  iowrite8_idx((u8 *)0xFF4D, 0x08, 2);

  iowrite32(&map->HSSI_PARA, 0x0600321B);

  iowrite16_idx((__le16 *)0xFFEC, 0x0800, 1);

  slot_time = 0x9;
  aifsn[0] = 2; /* AIFSN[AC_VO] */
  aifsn[1] = 2; /* AIFSN[AC_VI] */
  aifsn[2] = 7; /* AIFSN[AC_BK] */
  aifsn[3] = 3; /* AIFSN[AC_BE] */
  iowrite8(&map->ACM_CONTROL, 0);

  /* ENEDCA flag must always be set, transmit issues? */
  iowrite8(&map->MSR, RTL818X_MSR_ENEDCA);

  return TRUE;
}

/* ************************************************** */

static void
debug_buf (char *prefix, uint8 *buf, u32 len)
{
  s32 i, j;

  for (i=0;i<len;i+=8) {
    com1_printf ("%s: ", prefix);
    for (j=0;j<8;j++) {
      if (i+j >= len) break;
      com1_printf ("%.02X ", buf[i+j]);
    }
    com1_printf ("\n");
  }
}

/* ************************************************** */

#define TX_MAXPKT 64
static u8 tx_endps[4] = { 6, 7, 5, 4 };
static u8 cur_tx_ep = 0;

struct rtl8187_tx_hdr {
  __le32 flags;
  __le16 rts_duration;
  __le16 len;
  __le32 retry;
} PACKED;

struct rtl8187b_tx_hdr {
  __le32 flags;
  __le16 rts_duration;
  __le16 len;
  __le32 unused_1;
  __le16 unused_2;
  __le16 tx_duration;
  __le32 unused_3;
  __le32 retry;
  __le32 unused_4[2];
} PACKED;

static int
_tx (uint8 *pkt, u32 len)
{
  u32 act_len;
  int ret;
  struct rtl8187b_tx_hdr hdr;
  static uint8 buf[2500];

  memset (&hdr, 0, sizeof (hdr));

  hdr.len = hdr.flags = len;
  hdr.flags |= RTL818X_TX_DESC_FLAG_NO_ENC;
  hdr.flags |= RTL818X_TX_DESC_FLAG_FS;
  hdr.flags |= RTL818X_TX_DESC_FLAG_LS;
  hdr.retry = 1 << 8;
  hdr.tx_duration = 0x0831;

  memcpy (buf, &hdr, sizeof (hdr));
  memcpy (buf+sizeof (hdr), pkt, len);

  len += sizeof (hdr);

  debug_buf ("rtl8187b: tx", buf, len);

  if (usb_bulk_transfer (usbdev, tx_endps[cur_tx_ep], &buf, len,
                         TX_MAXPKT, DIR_OUT, &act_len) == 0) {
    DLOG ("tx: sent %d bytes", act_len);
    ret=act_len;
  } else ret=0;

  cur_tx_ep++;
  cur_tx_ep &= 0x03;

  return ret;
}

static int
tx (struct ieee80211_hw *dev, sk_buff_t *skb)
{
  return _tx (skb->data, skb->len);
}

static void
tx_probe_resp (void)
{
  uint8 pkt[] = {
    0x50, 0x00, 0x3A, 0x01,             /* PROBE RESPONSE */
    0x00, 0x23, 0x4D, 0xAF, 0x00, 0x3C, /* DA */

    //0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    //0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E, /* SA */
    0xb6, 0x27, 0x07, 0xb6, 0x73, 0x7E, /* BSS ID */

    0x20, 0x00, 

    0x8E, 0x91, 0x42, 0x11, 0x67, 0x00, 0x00, 0x00, /* timestamp */
    0x64, 0x00,                                     /* interval */
    0x01, 0x04,                                     /* capability */

    0x00, 0x06, 0x6C, 0x65, 0x6E, 0x6F, 0x76, 0x63, /* SSID lenovc */

    0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24
  };
  _tx (pkt, sizeof (pkt));
}

static void
tx_beacon (void)
{
  u32 len;
#if 0
  uint8 pkt[] = {
    0x00, 0x01,                 /* MGMT AssocReq toDS */
    0x01, 0x05,
    0x00, 0x26, 0xCB, 0xD0, 0x77, 0x46, /* DA */
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E, /* SA */
    0x00, 0x26, 0xCB, 0xD0, 0x77, 0x46, /* BSS ID */
    0x00, 0x00,                         /* SeqCtrl */
    /* SSID BU Wireless Help */
    0x00, 0x10, 0x42, 0x55, 0x20, 0x57, 0x69, 0x72, 0x65,
    0x6C, 0x65, 0x73, 0x73, 0x20, 0x48, 0x65, 0x6C, 0x70,
    /* Supported Rates */
    0x01, 0x08, 0x82, 0x84, 0x8B, 0x0C, 0x12, 0x96, 0x18, 0x24
  };
#endif
#if 0
  uint8 pkt[] = {
    0x80, 0x00,                 /* MGMT Beacon */
    0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, /* DA */
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E, /* SA */
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E, /* BSS ID */
    0x00, 0x00,                         /* SeqCtrl */
    0x8E, 0x91, 0x42, 0x11, 0x67, 0x00, 0x00, 0x00, /* timestamp */
    0x64, 0x00,                                     /* interval */
    0x01, 0x04,                                     /* capability */
    /* SSID Test */
    0x00, 0x04, 0x54, 0x65, 0x73, 0x74,
    /* Supported Rates */
    0x01, 0x08, 0x82, 0x84, 0x8B, 0x0C, 0x12, 0x96, 0x18, 0x24
  };
#endif
#if 1
  uint8 pkt[] = {
    0x80, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    //0x00, 0x23, 0x4d, 0xaf, 0x00, 0x3c,
    //0xb6, 0x27, 0x07, 0xb6, 0x00, 0x3c,
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E, /* SA */
    0xb6, 0x27, 0x07, 0xb6, 0x73, 0x7E, /* BSS ID */
    0x40, 0x46,
    0x00, 0x62, 0xc5, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x64, 0x00, 
    0x22, 0x00, 
    0x00, 0x06, 0x6c, 0x65, 0x6e, 0x6f, 0x76, 0x63,
    0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
    0x03, 0x01, 0x01,
    0x06, 0x02, 0x00, 0x00,
    0x2a, 0x01, 0x00,
    0x32, 0x04, 0x30, 0x48, 0x60, 0x6c
  };
#endif

  len = sizeof (pkt);

  _tx (pkt, len);
}

static uint32 beacon_stack[1024] ALIGNED(0x1000);

static void
beacon_thread (void)
{
  DLOG ("beacon: hello from 0x%x", str ());
  for (;;) {
    msleep (100);
    tx_beacon ();
  }
}

/* ************************************************** */

/*
  15:48:57.615816 314us Probe Request (lenova) [1.0* 2.0* 5.5* 11.0* 6.0 9.0 12.0 
  18.0 Mbit]
        0x0000:  4000 3a01 ffff ffff ffff 0023 4daf 003c
        0x0010:  ffff ffff ffff 7000 0006 6c65 6e6f 7661
        0x0020:  0108 8284 8b96 0c12 1824 3204 3048 606c
*/

/*
rtl8187b: rx: act_len=72
rtl8187b: rx: 40 00 3A 01 FF FF FF FF 
rtl8187b: rx: FF FF 00 23 4D AF 00 3C 
rtl8187b: rx: FF FF FF FF FF FF 20 00 
rtl8187b: rx: 00 06 6C 65 6E 6F 76 63 
rtl8187b: rx: 01 08 82 84 8B 96 0C 12 
rtl8187b: rx: 18 24 32 04 30 48 60 6C 
rtl8187b: rx: 3E 87 B5 87 34 00 01 00 
rtl8187b: rx: 45 79 A5 0C 00 00 00 00 
rtl8187b: rx: 4B 90 FE 00 8E F6 EF 1C 
rtl8187b: HDR: flags=0x00010034 agc=0xFE rssi=0x90 mactime=0x00000000 0CA57945
rtl8187b: FC=0x0040 MGMT 0x40 SEQ=0x0020
rtl8187b: PROBE REQUEST
rtl8187b:     SA=00:23:4D:AF:00:3C
rtl8187b:   SSID lenovc
rtl8187b:   SUPPORTED-RATE 0x82
rtl8187b:   SUPPORTED-RATE 0x84
rtl8187b:   SUPPORTED-RATE 0x8B
rtl8187b:   SUPPORTED-RATE 0x96
rtl8187b:   SUPPORTED-RATE 0x0C
rtl8187b:   SUPPORTED-RATE 0x12
rtl8187b:   SUPPORTED-RATE 0x18
rtl8187b:   SUPPORTED-RATE 0x24
*/

static u32 rx_conf;

static uint32 rx_stack[1024] ALIGNED(0x1000);
static uint32 status_stack[1024] ALIGNED(0x1000);

#define RX_EPT (is_rtl8187b ? 3 : 1)
#define RX_MAXPKT 64

static void
debug_info_element (u8 **p_buf, s32 *p_len)
{
  uint8 ssid[IEEE80211_MAX_SSID_LEN+1];
  u8 *buf = *p_buf;
  s32 i;

  switch (buf[0]) {
  case WLAN_EID_SSID:
    memset (ssid, 0, IEEE80211_MAX_SSID_LEN+1);
    memcpy (ssid, &buf[2], buf[1]);
    DLOG ("  SSID %s", ssid);
    break;
  case WLAN_EID_SUPP_RATES:
    for (i=0; i<buf[1]; i++)
      DLOG ("  SUPPORTED-RATE 0x%.02X", buf[2+i]);
    break;
  case WLAN_EID_DS_PARAMS:
    DLOG ("  DS PARAMS CURRENT CHANNEL %d", buf[2]);
    break;
  case WLAN_EID_COUNTRY:
    DLOG ("  COUNTRY %c%c%c", buf[2], buf[3], buf[4]);
    break;
  default:
    DLOG ("  IE-ID %d LEN %d", buf[0], buf[1]);
    break;
  }
  *p_len -= (buf[1] + 2);
  *p_buf += (buf[1] + 2);
}

static void
debug_info_elements (u8 *buf, s32 len)
{
  uint8 id = 0;
  DLOG ("Info Elements: (%p, %d)", buf, len);
  while (len > 0) {
    if (buf[0] < id)
      /* elements come in ascending order by ID */
      break;
    id = buf[0];
    debug_info_element (&buf, &len);
  }
}

struct rtl8187_rx_hdr {
  __le32 flags;
  u8 noise;
  u8 signal;
  u8 agc;
  u8 reserved;
  __le64 mac_time;
} PACKED;

struct rtl8187b_rx_hdr {
  __le32 flags;
  __le64 mac_time;
  u8 sq;
  u8 rssi;
  u8 agc;
  u8 flags2;
  __le16 snr_long2end;
  s8 pwdb_g12;
  u8 fot;
} PACKED;

static void
debug_rx (u8 *buf, u32 len)
{
  u32 req = 0;
  struct ieee80211_hdr *h = (struct ieee80211_hdr *) buf;
  char *type;
  struct rtl8187b_rx_hdr *hdr =
    /* "footer" really */
    (struct rtl8187b_rx_hdr *) &buf[len - sizeof (struct rtl8187b_rx_hdr)];

  DLOG ("HDR: flags=0x%.08X agc=0x%X rssi=0x%X mactime=0x%.08X %.08X",
        hdr->flags, hdr->agc, hdr->rssi,
        (u32) (hdr->mac_time >> 32),
        (u32) hdr->mac_time);

  req += sizeof (struct ieee80211_hdr);
  if (len < req) return;

  if (ieee80211_is_mgmt (h->frame_control))
    type = "MGMT";
  else if (ieee80211_is_ctl (h->frame_control))
    type = "CTRL";
  else if (ieee80211_is_data (h->frame_control))
    type = "DATA";
  else
    type = "????";

  DLOG ("FC=0x%.04X %s 0x%x SEQ=0x%.04X", h->frame_control, type,
        h->frame_control & IEEE80211_FCTL_STYPE,
        h->seq_ctrl);

  if (h->seq_ctrl & 0x000F)
    /* don't bother with fragments beyond the first */
    return;

  if (ieee80211_is_beacon (h->frame_control)) {
    /* interpret beacon frame */
    struct ieee80211_mgmt *m = (struct ieee80211_mgmt *) buf;
    DLOG ("BEACON bssid=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
          m->bssid[0], m->bssid[1], m->bssid[2],
          m->bssid[3], m->bssid[4], m->bssid[5]);
    DLOG ("  time=0x%.08X %.08X interval=0x%.04X capability=0x%.04X %s %s",
          (u32) (m->u.beacon.timestamp >> 32),
          (u32) (m->u.beacon.timestamp >> 00),
          m->u.beacon.beacon_int,
          m->u.beacon.capab_info,
          m->u.beacon.capab_info & WLAN_CAPABILITY_ESS ? "ESS" : "",
          m->u.beacon.capab_info & WLAN_CAPABILITY_IBSS ? "IBSS" : "");
    debug_info_elements (m->u.beacon.variable,
                         len
                         /* minus FCS */
                         - 4
                         /* minus header */
                         - ((u32) m->u.beacon.variable - (u32) m));
  } else if (ieee80211_is_assoc_req (h->frame_control)) {
    struct ieee80211_mgmt *m = (struct ieee80211_mgmt *) buf;
    uint8 *v = m->u.assoc_req.variable;
    s32 l = len;
    DLOG ("ASSOC REQUEST capab=0x%.04X interval=0x%.04X",
          m->u.assoc_req.capab_info,
          m->u.assoc_req.listen_interval);
    //debug_info_element (&v, &l); /* SSID */
    //debug_info_element (&v, &l); /* Supported Rates */
  } else if (ieee80211_is_assoc_resp (h->frame_control)) {
    struct ieee80211_mgmt *m = (struct ieee80211_mgmt *) buf;
    uint8 *v = m->u.assoc_resp.variable;
    s32 l = len;
    DLOG ("ASSOC RESPONSE capab=0x%.04X status=0x%.04X aid=0x%.04X",
          m->u.assoc_resp.capab_info,
          m->u.assoc_resp.status_code,
          m->u.assoc_resp.aid);
    //debug_info_element (&v, &l); /* Supported Rates */
  } else if (ieee80211_is_probe_req (h->frame_control)) {
    struct ieee80211_mgmt *m = (struct ieee80211_mgmt *) buf;
    uint8 *v = m->u.probe_req.variable;
    s32 l = len;
    DLOG ("PROBE REQUEST");
    DLOG ("    SA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr2[0], h->addr2[1], h->addr2[2],
            h->addr2[3], h->addr2[4], h->addr2[5]);
    debug_info_element (&v, &l); /* SSID */
    debug_info_element (&v, &l); /* Supported Rates */
  } else if (ieee80211_is_probe_resp (h->frame_control)) {
    DLOG ("PROBE RESPONSE");
  } else if (ieee80211_is_data (h->frame_control)) {
    /* get some info out of a DATA frame */
    bool toDS, fromDS;
    toDS = ieee80211_has_tods (h->frame_control);
    fromDS = ieee80211_has_fromds (h->frame_control);
    if (!toDS && !fromDS) {
      DLOG ("  IBSS/DLS");
      DLOG ("    DA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr1[0], h->addr1[1], h->addr1[2],
            h->addr1[3], h->addr1[4], h->addr1[5]);
      DLOG ("    SA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr2[0], h->addr2[1], h->addr2[2],
            h->addr2[3], h->addr2[4], h->addr2[5]);
      DLOG ("    BSSID=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr3[0], h->addr3[1], h->addr3[2],
            h->addr3[3], h->addr3[4], h->addr3[5]);
    } else if (!toDS && fromDS) {
      DLOG ("  AP -> STA");
      DLOG ("    DA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr1[0], h->addr1[1], h->addr1[2],
            h->addr1[3], h->addr1[4], h->addr1[5]);
      DLOG ("    BSSID=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr2[0], h->addr2[1], h->addr2[2],
            h->addr2[3], h->addr2[4], h->addr2[5]);
      DLOG ("    SA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr3[0], h->addr3[1], h->addr3[2],
            h->addr3[3], h->addr3[4], h->addr3[5]);
    } else if (toDS && !fromDS) {
      DLOG ("  STA -> AP");
      DLOG ("    BSSID=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr1[0], h->addr1[1], h->addr1[2],
            h->addr1[3], h->addr1[4], h->addr1[5]);
      DLOG ("    SA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr2[0], h->addr2[1], h->addr2[2],
            h->addr2[3], h->addr2[4], h->addr2[5]);
      DLOG ("    DA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr3[0], h->addr3[1], h->addr3[2],
            h->addr3[3], h->addr3[4], h->addr3[5]);
    } else {
      DLOG ("  unspecified (WDS)");
      DLOG ("    RA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr1[0], h->addr1[1], h->addr1[2],
            h->addr1[3], h->addr1[4], h->addr1[5]);
      DLOG ("    TA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr2[0], h->addr2[1], h->addr2[2],
            h->addr2[3], h->addr2[4], h->addr2[5]);
      DLOG ("    DA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr3[0], h->addr3[1], h->addr3[2],
            h->addr3[3], h->addr3[4], h->addr3[5]);
      DLOG ("    SA=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            h->addr4[0], h->addr4[1], h->addr4[2],
            h->addr4[3], h->addr4[4], h->addr4[5]);
    }
  }
}

static void
rx_thread (void)
{
  u8 buf[RTL8187_MAX_RX];
  u32 act_len;
  int i;

  DLOG ("rx: hello from 0x%x", str ());
  for (;;) {
    if (usb_bulk_transfer (usbdev, RX_EPT, &buf, sizeof (buf),
                           RX_MAXPKT, DIR_IN, &act_len) == 0) {
      if (act_len > 0) {
#ifdef DEBUG_RTL8187B
        DLOG ("rx: act_len=%d", act_len);
        debug_buf ("rtl8187b: rx", buf, act_len);
        debug_rx (buf, act_len);
#endif
      }
    }
  }
}

#define STATUS_EPT 9
#define STATUS_MAXPKT 64

/*
 * Read from status buffer:
 *
 * bits [30:31] = cmd type:
 * - 0 indicates tx beacon interrupt
 * - 1 indicates tx close descriptor
 *
 * In the case of tx beacon interrupt:
 * [0:9] = Last Beacon CW
 * [10:29] = reserved
 * [30:31] = 00b
 * [32:63] = Last Beacon TSF
 *
 * If it's tx close descriptor:
 * [0:7] = Packet Retry Count
 * [8:14] = RTS Retry Count
 * [15] = TOK
 * [16:27] = Sequence No
 * [28] = LS
 * [29] = FS
 * [30:31] = 01b
 * [32:47] = unused (reserved?)
 * [48:63] = MAC Used Time
 */

static void
status_thread (void)
{
  u64 buf;
  u32 act_len, pkt_rc, seq_no;
  bool tok;

  DLOG ("status: hello from 0x%x", str ());
  for (;;) {
    if (usb_bulk_transfer (usbdev, STATUS_EPT, &buf, sizeof (buf),
                           STATUS_MAXPKT, DIR_IN, &act_len) == 0) {
      if (act_len > 0) {
        DLOG ("status: 0x%.08X %.08X",
              (u32) (buf >> 32), (u32) buf);
        if (((buf >> 30) & 0x3) == 1) {
          /* TX close descriptor */
          pkt_rc = buf & 0xFF;
          tok = (buf & (1 << 15) ? TRUE : FALSE);
          seq_no = (buf >> 16) && 0xFFF;
          DLOG ("status: TX close: pkt_rc=%d tok=%d seq_no=%d LS=%d FS=%d",
                pkt_rc, tok, seq_no,
                (buf & (1 << 28)) ? TRUE : FALSE,
                (buf & (1 << 29)) ? TRUE : FALSE);
        } else {
          /* TX beacon interrupt */
          DLOG ("status: TX beacon: last_CW=%d last_TSF=%d",
                buf & 0x3FF,
                buf >> 32);
        }
      } else
        DLOG ("status: act_len==0");
    }
  }
}

static void
init_urbs (void)
{
  start_kernel_thread ((u32) rx_thread, (u32) &rx_stack[1023]);
}

static void
init_status_urb (void)
{
  start_kernel_thread ((u32) status_thread, (u32) &status_stack[1023]);
}

static int
start (struct ieee80211_hw *hw)
{
  u32 reg;
  bool ret;

  ret = init_hw ();

  if (!ret)
    goto rtl8187_start_exit;

  //init_usb_anchor(&priv->anchored);

  if (is_rtl8187b) {
    reg = RTL818X_RX_CONF_MGMT |
      RTL818X_RX_CONF_DATA |
      RTL818X_RX_CONF_BROADCAST |
      RTL818X_RX_CONF_NICMAC |
      RTL818X_RX_CONF_BSSID |
      (7 << 13 /* RX FIFO threshold NONE */) |
      (7 << 10 /* MAX RX DMA */) |
      RTL818X_RX_CONF_RX_AUTORESETPHY |
      RTL818X_RX_CONF_ONLYERLPKT |
      RTL818X_RX_CONF_MULTICAST;
    rx_conf = reg;
    iowrite32(&map->RX_CONF, reg);

    iowrite32(&map->TX_CONF,
                      RTL818X_TX_CONF_HW_SEQNUM |
                      RTL818X_TX_CONF_DISREQQSIZE |
                      (7 << 8  /* short retry limit */) |
                      (7 << 0  /* long retry limit */) |
                      (7 << 21 /* MAX TX DMA */));
    init_urbs();
    init_status_urb();
    goto rtl8187_start_exit;
  }

  iowrite16(&map->INT_MASK, 0xFFFF);

  iowrite32(&map->MAR[0], ~0);
  iowrite32(&map->MAR[1], ~0);

  init_urbs();

  reg = RTL818X_RX_CONF_ONLYERLPKT |
    RTL818X_RX_CONF_RX_AUTORESETPHY |
    RTL818X_RX_CONF_BSSID |
    RTL818X_RX_CONF_MGMT |
    RTL818X_RX_CONF_DATA |
    (7 << 13 /* RX FIFO threshold NONE */) |
    (7 << 10 /* MAX RX DMA */) |
    RTL818X_RX_CONF_BROADCAST |
    RTL818X_RX_CONF_NICMAC;

  rx_conf = reg;
  iowrite32(&map->RX_CONF, reg);

  reg = ioread8(&map->CW_CONF);
  reg &= ~RTL818X_CW_CONF_PERPACKET_CW_SHIFT;
  reg |= RTL818X_CW_CONF_PERPACKET_RETRY_SHIFT;
  iowrite8(&map->CW_CONF, reg);

  reg = ioread8(&map->TX_AGC_CTL);
  reg &= ~RTL818X_TX_AGC_CTL_PERPACKET_GAIN_SHIFT;
  reg &= ~RTL818X_TX_AGC_CTL_PERPACKET_ANTSEL_SHIFT;
  reg &= ~RTL818X_TX_AGC_CTL_FEEDBACK_ANT;
  iowrite8(&map->TX_AGC_CTL, reg);

  reg  = RTL818X_TX_CONF_CW_MIN |
    (7 << 21 /* MAX TX DMA */) |
    RTL818X_TX_CONF_NO_ICV;
  iowrite32(&map->TX_CONF, reg);

  reg = ioread8(&map->CMD);
  reg |= RTL818X_CMD_TX_ENABLE;
  reg |= RTL818X_CMD_RX_ENABLE;
  iowrite8(&map->CMD, reg);
  //INIT_DELAYED_WORK(&priv->work, rtl8187_work);

 rtl8187_start_exit:
  return ret;
}

static void
stop (struct ieee80211_hw *hw)
{
  DLOG ("stop");
}

static uint16 txpwr_base;
static uint8 asic_rev, queues;
const struct rtl818x_rf_ops * rtl8187_detect_rf(void);

static struct ieee80211_ops rtl8187b_ops = {
  .start            = start,
  .stop             = stop,
  .config           = config,
  .add_interface    = add_iface,
  .remove_interface = rem_iface,
  .bss_info_changed = bss_info_changed,
  .conf_tx          = conf_tx,
  .tx               = tx
};

static bool
probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  struct ieee80211_hw *dev;
  char *chipname;
  int i;
  uint8 reg;
  uint16 txpwr, reg16;
  struct ieee80211_channel *channel;

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
  is_rtl8187b = TRUE;           /* assume for now */

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

  eeprom_93cx6_read (&eeprom, RTL8187_EEPROM_TXPWR_BASE, &txpwr_base);
  DLOG ("txpwr_base=0x%.04X", txpwr_base);

  reg = ioread8 (&map->PGSELECT) & ~1;
  iowrite8 (&map->PGSELECT, reg | 1);
  /* 0 means asic B-cut, we should use SW 3 wire
   * bit-by-bit banging for radio. 1 means we can use
   * USB specific request to write radio registers */
  asic_rev = ioread8 ((uint8*) 0xFFFE) & 0x3;
  iowrite8 (&map->PGSELECT, reg);
  iowrite8 (&map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

  DLOG ("asic_rev=0x%.02X", asic_rev);

  if (!is_rtl8187b) {
    u32 reg32;
    reg32 = ioread32(&map->TX_CONF);
    reg32 &= RTL818X_TX_CONF_HWVER_MASK;
    switch (reg32) {
    case RTL818X_TX_CONF_R8187vD_B:
      /* Some RTL8187B devices have a USB ID of 0x8187
       * detect them here */
      chipname = "RTL8187BvB(early)";
      is_rtl8187b = TRUE;
      hw_rev = RTL8187BvB;
      break;
    case RTL818X_TX_CONF_R8187vD:
      chipname = "RTL8187vD";
      break;
    default:
      chipname = "RTL8187vB (default)";
      break;
    }
  } else {

    switch (ioread8 ((u8*)0xFFE1)) {
    case RTL818X_R8187B_B:
      hw_rev = RTL8187BvB;
      chipname = "RTL8187BvB";
      break;
    case RTL818X_R8187B_D:
      hw_rev = RTL8187BvD;
      chipname = "RTL8187BvD";
      break;
    case RTL818X_R8187B_E:
      hw_rev = RTL8187BvE;
      chipname = "RTL8187BvE";
      break;
    default:
      hw_rev = RTL8187BvB;
      chipname = "RTL8187BvB (default)";
      break;
    }
  }
  DLOG ("hw_rev=%s", chipname);

  memcpy (channels, rtl818x_channels, sizeof (rtl818x_channels));
  memcpy (rates, rtl818x_rates, sizeof (rtl818x_rates));

  channel = channels;
  for (i=0; i<3; i++) {
    eeprom_93cx6_read (&eeprom, RTL8187_EEPROM_TXPWR_CHAN_1 + i,
                       &txpwr);
    (*channel++).hw_value = txpwr & 0xFF;
    (*channel++).hw_value = txpwr >> 8;
  }
  for (i=0; i<2; i++) {
    eeprom_93cx6_read (&eeprom, RTL8187_EEPROM_TXPWR_CHAN_4 + i,
                       &txpwr);
    (*channel++).hw_value = txpwr & 0xFF;
    (*channel++).hw_value = txpwr >> 8;
  }

  if (!is_rtl8187b) {
    for (i = 0; i < 2; i++) {
      eeprom_93cx6_read(&eeprom,
                        RTL8187_EEPROM_TXPWR_CHAN_6 + i,
                        &txpwr);
      (*channel++).hw_value = txpwr & 0xFF;
      (*channel++).hw_value = txpwr >> 8;
    }
  } else {
    eeprom_93cx6_read(&eeprom, RTL8187_EEPROM_TXPWR_CHAN_6,
                      &txpwr);
    (*channel++).hw_value = txpwr & 0xFF;

    eeprom_93cx6_read(&eeprom, 0x0A, &txpwr);
    (*channel++).hw_value = txpwr & 0xFF;

    eeprom_93cx6_read(&eeprom, 0x1C, &txpwr);
    (*channel++).hw_value = txpwr & 0xFF;
    (*channel++).hw_value = txpwr >> 8;
  }

  rfkill_mask = RFKILL_MASK_8187_89_97;
  eeprom_93cx6_read (&eeprom, RTL8187_EEPROM_SELECT_GPIO, &reg16);
  if (reg16 & 0xFF00)
    rfkill_mask = RFKILL_MASK_8198;
  DLOG ("rfkill_mask=0x%x", rfkill_mask);

  rf = rtl8187_detect_rf ();
  DLOG ("detected RF name: %s", rf->name);

  if (is_rtl8187b) queues = 4; else queues = 1;

  rfkill_init ();

  dev = ieee80211_alloc_hw (0, &rtl8187b_ops);

  if (!dev) return FALSE;

  if (!ieee80211_register_hw (dev))
    return FALSE;

  start_kernel_thread ((u32) beacon_thread, (u32) &beacon_stack[1023]);

  return TRUE;
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

/* ************************************************** */

/* Radio tuning for RTL8225 on RTL8187 */

static inline
void rtl8225_write_phy_ofdm(u8 addr, u32 data)
{
  rtl8187_write_phy (addr, data);
}

static inline
void rtl8225_write_phy_cck(u8 addr, u32 data)
{
  rtl8187_write_phy (addr, data | 0x10000);
}

/* used when asic_rev==0 */
static void
rtl8225_write_bitbang (u8 addr, u16 data)
{
  u16 reg80, reg84, reg82;
  u32 bangdata;
  int i;

  bangdata = (data << 4) | (addr & 0xf);

  reg80 = ioread16(&map->RFPinsOutput) & 0xfff3;
  reg82 = ioread16(&map->RFPinsEnable);

  iowrite16(&map->RFPinsEnable, reg82 | 0x7);

  reg84 = ioread16(&map->RFPinsSelect);
  iowrite16(&map->RFPinsSelect, reg84 | 0x7);
  udelay(10);

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  udelay(2);
  iowrite16(&map->RFPinsOutput, reg80);
  udelay(10);

  for (i = 15; i >= 0; i--) {
    u16 reg = reg80 | (bangdata & (1 << i)) >> i;

    if (i & 1)
      iowrite16(&map->RFPinsOutput, reg);

    iowrite16(&map->RFPinsOutput, reg | (1 << 1));
    iowrite16(&map->RFPinsOutput, reg | (1 << 1));

    if (!(i & 1))
      iowrite16(&map->RFPinsOutput, reg);
  }

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  udelay(10);

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  iowrite16(&map->RFPinsSelect, reg84);
}

static void
rtl8225_write_8051(u8 addr, __le16 data)
{
  u16 reg80, reg82, reg84;

  reg80 = ioread16(&map->RFPinsOutput);
  reg82 = ioread16(&map->RFPinsEnable);
  reg84 = ioread16(&map->RFPinsSelect);

  reg80 &= ~(0x3 << 2);
  reg84 &= ~0xF;

  iowrite16(&map->RFPinsEnable, reg82 | 0x0007);
  iowrite16(&map->RFPinsSelect, reg84 | 0x0007);
  udelay(10);

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  udelay(2);

  iowrite16(&map->RFPinsOutput, reg80);
  udelay(10);

  control_msg(RTL8187_REQ_SET_REG, RTL8187_REQT_WRITE,
              addr, 0x8225, (uint8*)&data, sizeof(data));

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  udelay(10);

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  iowrite16(&map->RFPinsSelect, reg84);
}

static void rtl8225_write(u8 addr, u16 data)
{
  if (asic_rev)
    rtl8225_write_8051(addr, data);
  else
    rtl8225_write_bitbang(addr, data);
}

static u16 rtl8225_read(u8 addr)
{
  u16 reg80, reg82, reg84, out;
  int i;

  reg80 = ioread16(&map->RFPinsOutput);
  reg82 = ioread16(&map->RFPinsEnable);
  reg84 = ioread16(&map->RFPinsSelect);

  reg80 &= ~0xF;

  iowrite16(&map->RFPinsEnable, reg82 | 0x000F);
  iowrite16(&map->RFPinsSelect, reg84 | 0x000F);

  iowrite16(&map->RFPinsOutput, reg80 | (1 << 2));
  udelay(4);
  iowrite16(&map->RFPinsOutput, reg80);
  udelay(5);

  for (i = 4; i >= 0; i--) {
    u16 reg = reg80 | ((addr >> i) & 1);

    if (!(i & 1)) {
      iowrite16(&map->RFPinsOutput, reg);
      udelay(1);
    }

    iowrite16(&map->RFPinsOutput,
                      reg | (1 << 1));
    udelay(2);
    iowrite16(&map->RFPinsOutput,
                      reg | (1 << 1));
    udelay(2);

    if (i & 1) {
      iowrite16(&map->RFPinsOutput, reg);
      udelay(1);
    }
  }

  iowrite16(&map->RFPinsOutput,
                    reg80 | (1 << 3) | (1 << 1));
  udelay(2);
  iowrite16(&map->RFPinsOutput,
                    reg80 | (1 << 3));
  udelay(2);
  iowrite16(&map->RFPinsOutput,
                    reg80 | (1 << 3));
  udelay(2);

  out = 0;
  for (i = 11; i >= 0; i--) {
    iowrite16(&map->RFPinsOutput,
                      reg80 | (1 << 3));
    udelay(1);
    iowrite16(&map->RFPinsOutput,
                      reg80 | (1 << 3) | (1 << 1));
    udelay(2);
    iowrite16(&map->RFPinsOutput,
                      reg80 | (1 << 3) | (1 << 1));
    udelay(2);
    iowrite16(&map->RFPinsOutput,
                      reg80 | (1 << 3) | (1 << 1));
    udelay(2);

    if (ioread16(&map->RFPinsInput) & (1 << 1))
      out |= 1 << i;

    iowrite16(&map->RFPinsOutput,
                      reg80 | (1 << 3));
    udelay(2);
  }

  iowrite16(&map->RFPinsOutput,
                    reg80 | (1 << 3) | (1 << 2));
  udelay(2);

  iowrite16(&map->RFPinsEnable, reg82);
  iowrite16(&map->RFPinsSelect, reg84);
  iowrite16(&map->RFPinsOutput, 0x03A0);

  return out;
}

static const u16 rtl8225bcd_rxgain[] = {
  0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0408, 0x0409,
  0x040a, 0x040b, 0x0502, 0x0503, 0x0504, 0x0505, 0x0540, 0x0541,
  0x0542, 0x0543, 0x0544, 0x0545, 0x0580, 0x0581, 0x0582, 0x0583,
  0x0584, 0x0585, 0x0588, 0x0589, 0x058a, 0x058b, 0x0643, 0x0644,
  0x0645, 0x0680, 0x0681, 0x0682, 0x0683, 0x0684, 0x0685, 0x0688,
  0x0689, 0x068a, 0x068b, 0x068c, 0x0742, 0x0743, 0x0744, 0x0745,
  0x0780, 0x0781, 0x0782, 0x0783, 0x0784, 0x0785, 0x0788, 0x0789,
  0x078a, 0x078b, 0x078c, 0x078d, 0x0790, 0x0791, 0x0792, 0x0793,
  0x0794, 0x0795, 0x0798, 0x0799, 0x079a, 0x079b, 0x079c, 0x079d,
  0x07a0, 0x07a1, 0x07a2, 0x07a3, 0x07a4, 0x07a5, 0x07a8, 0x07a9,
  0x07aa, 0x07ab, 0x07ac, 0x07ad, 0x07b0, 0x07b1, 0x07b2, 0x07b3,
  0x07b4, 0x07b5, 0x07b8, 0x07b9, 0x07ba, 0x07bb, 0x07bb
};

static const u8 rtl8225_agc[] = {
  0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e, 0x9e,
  0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98, 0x97, 0x96,
  0x95, 0x94, 0x93, 0x92, 0x91, 0x90, 0x8f, 0x8e,
  0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88, 0x87, 0x86,
  0x85, 0x84, 0x83, 0x82, 0x81, 0x80, 0x3f, 0x3e,
  0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38, 0x37, 0x36,
  0x35, 0x34, 0x33, 0x32, 0x31, 0x30, 0x2f, 0x2e,
  0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28, 0x27, 0x26,
  0x25, 0x24, 0x23, 0x22, 0x21, 0x20, 0x1f, 0x1e,
  0x1d, 0x1c, 0x1b, 0x1a, 0x19, 0x18, 0x17, 0x16,
  0x15, 0x14, 0x13, 0x12, 0x11, 0x10, 0x0f, 0x0e,
  0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06,
  0x05, 0x04, 0x03, 0x02, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
};

static const u8 rtl8225_gain[] = {
  0x23, 0x88, 0x7c, 0xa5,	/* -82dBm */
  0x23, 0x88, 0x7c, 0xb5,	/* -82dBm */
  0x23, 0x88, 0x7c, 0xc5,	/* -82dBm */
  0x33, 0x80, 0x79, 0xc5,	/* -78dBm */
  0x43, 0x78, 0x76, 0xc5,	/* -74dBm */
  0x53, 0x60, 0x73, 0xc5,	/* -70dBm */
  0x63, 0x58, 0x70, 0xc5,	/* -66dBm */
};

static const u8 rtl8225_threshold[] = {
  0x8d, 0x8d, 0x8d, 0x8d, 0x9d, 0xad, 0xbd
};

static const u8 rtl8225_tx_gain_cck_ofdm[] = {
  0x02, 0x06, 0x0e, 0x1e, 0x3e, 0x7e
};

static const u8 rtl8225_tx_power_cck[] = {
  0x18, 0x17, 0x15, 0x11, 0x0c, 0x08, 0x04, 0x02,
  0x1b, 0x1a, 0x17, 0x13, 0x0e, 0x09, 0x04, 0x02,
  0x1f, 0x1e, 0x1a, 0x15, 0x10, 0x0a, 0x05, 0x02,
  0x22, 0x21, 0x1d, 0x18, 0x11, 0x0b, 0x06, 0x02,
  0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03,
  0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03
};

static const u8 rtl8225_tx_power_cck_ch14[] = {
  0x18, 0x17, 0x15, 0x0c, 0x00, 0x00, 0x00, 0x00,
  0x1b, 0x1a, 0x17, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x1f, 0x1e, 0x1a, 0x0f, 0x00, 0x00, 0x00, 0x00,
  0x22, 0x21, 0x1d, 0x11, 0x00, 0x00, 0x00, 0x00,
  0x26, 0x25, 0x21, 0x13, 0x00, 0x00, 0x00, 0x00,
  0x2b, 0x2a, 0x25, 0x15, 0x00, 0x00, 0x00, 0x00
};

static const u8 rtl8225_tx_power_ofdm[] = {
  0x80, 0x90, 0xa2, 0xb5, 0xcb, 0xe4
};

static const u32 rtl8225_chan[] = {
  0x085c, 0x08dc, 0x095c, 0x09dc, 0x0a5c, 0x0adc, 0x0b5c,
  0x0bdc, 0x0c5c, 0x0cdc, 0x0d5c, 0x0ddc, 0x0e5c, 0x0f72
};

#define min(a,b) ((a) < (b) ? (a) : (b))

static void rtl8225_rf_set_tx_power(int channel)
{
  u8 cck_power, ofdm_power;
  const u8 *tmp;
  u32 reg;
  int i;

  cck_power = channels[channel - 1].hw_value & 0xF;
  ofdm_power = channels[channel - 1].hw_value >> 4;

  cck_power = min(cck_power, (u8)11);
  if (ofdm_power > (u8)15)
    ofdm_power = 25;
  else
    ofdm_power += 10;

  iowrite8(&map->TX_GAIN_CCK,
                   rtl8225_tx_gain_cck_ofdm[cck_power / 6] >> 1);

  if (channel == 14)
    tmp = &rtl8225_tx_power_cck_ch14[(cck_power % 6) * 8];
  else
    tmp = &rtl8225_tx_power_cck[(cck_power % 6) * 8];

  for (i = 0; i < 8; i++)
    rtl8225_write_phy_cck(0x44 + i, *tmp++);

  msleep(1); // FIXME: optional?

  /* anaparam2 on */
  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8(&map->CONFIG3);
  iowrite8(&map->CONFIG3,
                   reg | RTL818X_CONFIG3_ANAPARAM_WRITE);
  iowrite32(&map->ANAPARAM2,
                    RTL8187_RTL8225_ANAPARAM2_ON);
  iowrite8(&map->CONFIG3,
                   reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);
  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

  rtl8225_write_phy_ofdm(2, 0x42);
  rtl8225_write_phy_ofdm(6, 0x00);
  rtl8225_write_phy_ofdm(8, 0x00);

  iowrite8(&map->TX_GAIN_OFDM,
                   rtl8225_tx_gain_cck_ofdm[ofdm_power / 6] >> 1);

  tmp = &rtl8225_tx_power_ofdm[ofdm_power % 6];

  rtl8225_write_phy_ofdm(5, *tmp);
  rtl8225_write_phy_ofdm(7, *tmp);

  msleep(1);
}

static void
rtl8225_rf_init(void)
{
  int i;

  rtl8225_write(0x0, 0x067);
  rtl8225_write(0x1, 0xFE0);
  rtl8225_write(0x2, 0x44D);
  rtl8225_write(0x3, 0x441);
  rtl8225_write(0x4, 0x486);
  rtl8225_write(0x5, 0xBC0);
  rtl8225_write(0x6, 0xAE6);
  rtl8225_write(0x7, 0x82A);
  rtl8225_write(0x8, 0x01F);
  rtl8225_write(0x9, 0x334);
  rtl8225_write(0xA, 0xFD4);
  rtl8225_write(0xB, 0x391);
  rtl8225_write(0xC, 0x050);
  rtl8225_write(0xD, 0x6DB);
  rtl8225_write(0xE, 0x029);
  rtl8225_write(0xF, 0x914); msleep(100);

  rtl8225_write(0x2, 0xC4D); msleep(200);
  rtl8225_write(0x2, 0x44D); msleep(200);

  if (!(rtl8225_read(6) & (1 << 7))) {
    rtl8225_write(0x02, 0x0c4d);
    msleep(200);
    rtl8225_write(0x02, 0x044d);
    msleep(100);
    if (!(rtl8225_read(6) & (1 << 7)))
      DLOG("RF Calibration Failed! %x\n",
           rtl8225_read(6));
  }

  rtl8225_write(0x0, 0x127);

  for (i = 0; i < ARRAY_SIZE(rtl8225bcd_rxgain); i++) {
    rtl8225_write(0x1, i + 1);
    rtl8225_write(0x2, rtl8225bcd_rxgain[i]);
  }

  rtl8225_write(0x0, 0x027);
  rtl8225_write(0x0, 0x22F);

  for (i = 0; i < ARRAY_SIZE(rtl8225_agc); i++) {
    rtl8225_write_phy_ofdm(0xB, rtl8225_agc[i]);
    rtl8225_write_phy_ofdm(0xA, 0x80 + i);
  }

  msleep(1);

  rtl8225_write_phy_ofdm(0x00, 0x01);
  rtl8225_write_phy_ofdm(0x01, 0x02);
  rtl8225_write_phy_ofdm(0x02, 0x42);
  rtl8225_write_phy_ofdm(0x03, 0x00);
  rtl8225_write_phy_ofdm(0x04, 0x00);
  rtl8225_write_phy_ofdm(0x05, 0x00);
  rtl8225_write_phy_ofdm(0x06, 0x40);
  rtl8225_write_phy_ofdm(0x07, 0x00);
  rtl8225_write_phy_ofdm(0x08, 0x40);
  rtl8225_write_phy_ofdm(0x09, 0xfe);
  rtl8225_write_phy_ofdm(0x0a, 0x09);
  rtl8225_write_phy_ofdm(0x0b, 0x80);
  rtl8225_write_phy_ofdm(0x0c, 0x01);
  rtl8225_write_phy_ofdm(0x0e, 0xd3);
  rtl8225_write_phy_ofdm(0x0f, 0x38);
  rtl8225_write_phy_ofdm(0x10, 0x84);
  rtl8225_write_phy_ofdm(0x11, 0x06);
  rtl8225_write_phy_ofdm(0x12, 0x20);
  rtl8225_write_phy_ofdm(0x13, 0x20);
  rtl8225_write_phy_ofdm(0x14, 0x00);
  rtl8225_write_phy_ofdm(0x15, 0x40);
  rtl8225_write_phy_ofdm(0x16, 0x00);
  rtl8225_write_phy_ofdm(0x17, 0x40);
  rtl8225_write_phy_ofdm(0x18, 0xef);
  rtl8225_write_phy_ofdm(0x19, 0x19);
  rtl8225_write_phy_ofdm(0x1a, 0x20);
  rtl8225_write_phy_ofdm(0x1b, 0x76);
  rtl8225_write_phy_ofdm(0x1c, 0x04);
  rtl8225_write_phy_ofdm(0x1e, 0x95);
  rtl8225_write_phy_ofdm(0x1f, 0x75);
  rtl8225_write_phy_ofdm(0x20, 0x1f);
  rtl8225_write_phy_ofdm(0x21, 0x27);
  rtl8225_write_phy_ofdm(0x22, 0x16);
  rtl8225_write_phy_ofdm(0x24, 0x46);
  rtl8225_write_phy_ofdm(0x25, 0x20);
  rtl8225_write_phy_ofdm(0x26, 0x90);
  rtl8225_write_phy_ofdm(0x27, 0x88);

  rtl8225_write_phy_ofdm(0x0d, rtl8225_gain[2 * 4]);
  rtl8225_write_phy_ofdm(0x1b, rtl8225_gain[2 * 4 + 2]);
  rtl8225_write_phy_ofdm(0x1d, rtl8225_gain[2 * 4 + 3]);
  rtl8225_write_phy_ofdm(0x23, rtl8225_gain[2 * 4 + 1]);

  rtl8225_write_phy_cck(0x00, 0x98);
  rtl8225_write_phy_cck(0x03, 0x20);
  rtl8225_write_phy_cck(0x04, 0x7e);
  rtl8225_write_phy_cck(0x05, 0x12);
  rtl8225_write_phy_cck(0x06, 0xfc);
  rtl8225_write_phy_cck(0x07, 0x78);
  rtl8225_write_phy_cck(0x08, 0x2e);
  rtl8225_write_phy_cck(0x10, 0x9b);
  rtl8225_write_phy_cck(0x11, 0x88);
  rtl8225_write_phy_cck(0x12, 0x47);
  rtl8225_write_phy_cck(0x13, 0xd0);
  rtl8225_write_phy_cck(0x19, 0x00);
  rtl8225_write_phy_cck(0x1a, 0xa0);
  rtl8225_write_phy_cck(0x1b, 0x08);
  rtl8225_write_phy_cck(0x40, 0x86);
  rtl8225_write_phy_cck(0x41, 0x8d);
  rtl8225_write_phy_cck(0x42, 0x15);
  rtl8225_write_phy_cck(0x43, 0x18);
  rtl8225_write_phy_cck(0x44, 0x1f);
  rtl8225_write_phy_cck(0x45, 0x1e);
  rtl8225_write_phy_cck(0x46, 0x1a);
  rtl8225_write_phy_cck(0x47, 0x15);
  rtl8225_write_phy_cck(0x48, 0x10);
  rtl8225_write_phy_cck(0x49, 0x0a);
  rtl8225_write_phy_cck(0x4a, 0x05);
  rtl8225_write_phy_cck(0x4b, 0x02);
  rtl8225_write_phy_cck(0x4c, 0x05);

  iowrite8(&map->TESTR, 0x0D);

  rtl8225_rf_set_tx_power(1);

  /* RX antenna default to A */
  rtl8225_write_phy_cck(0x10, 0x9b);			/* B: 0xDB */
  rtl8225_write_phy_ofdm(0x26, 0x90);		/* B: 0x10 */

  iowrite8(&map->TX_ANTENNA, 0x03);	/* B: 0x00 */
  msleep(1);
  iowrite32((__le32 *)0xFF94, 0x3dc00002);

  /* set sensitivity */
  rtl8225_write(0x0c, 0x50);
  rtl8225_write_phy_ofdm(0x0d, rtl8225_gain[2 * 4]);
  rtl8225_write_phy_ofdm(0x1b, rtl8225_gain[2 * 4 + 2]);
  rtl8225_write_phy_ofdm(0x1d, rtl8225_gain[2 * 4 + 3]);
  rtl8225_write_phy_ofdm(0x23, rtl8225_gain[2 * 4 + 1]);
  rtl8225_write_phy_cck(0x41, rtl8225_threshold[2]);
}

static const u8 rtl8225z2_agc[] = {
  0x5e, 0x5e, 0x5e, 0x5e, 0x5d, 0x5b, 0x59, 0x57, 0x55, 0x53, 0x51, 0x4f,
  0x4d, 0x4b, 0x49, 0x47, 0x45, 0x43, 0x41, 0x3f, 0x3d, 0x3b, 0x39, 0x37,
  0x35, 0x33, 0x31, 0x2f, 0x2d, 0x2b, 0x29, 0x27, 0x25, 0x23, 0x21, 0x1f,
  0x1d, 0x1b, 0x19, 0x17, 0x15, 0x13, 0x11, 0x0f, 0x0d, 0x0b, 0x09, 0x07,
  0x05, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
  0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x26, 0x27, 0x27, 0x28,
  0x28, 0x29, 0x2a, 0x2a, 0x2a, 0x2b, 0x2b, 0x2b, 0x2c, 0x2c, 0x2c, 0x2d,
  0x2d, 0x2d, 0x2d, 0x2e, 0x2e, 0x2e, 0x2e, 0x2f, 0x2f, 0x2f, 0x30, 0x30,
  0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31,
  0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31, 0x31
};
static const u8 rtl8225z2_ofdm[] = {
  0x10, 0x0d, 0x01, 0x00, 0x14, 0xfb, 0xfb, 0x60,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x5c, 0x00, 0x00,
  0x40, 0x00, 0x40, 0x00, 0x00, 0x00, 0xa8, 0x26,
  0x32, 0x33, 0x07, 0xa5, 0x6f, 0x55, 0xc8, 0xb3,
  0x0a, 0xe1, 0x2C, 0x8a, 0x86, 0x83, 0x34, 0x0f,
  0x4f, 0x24, 0x6f, 0xc2, 0x6b, 0x40, 0x80, 0x00,
  0xc0, 0xc1, 0x58, 0xf1, 0x00, 0xe4, 0x90, 0x3e,
  0x6d, 0x3c, 0xfb, 0x07
};

static const u8 rtl8225z2_tx_power_cck_ch14[] = {
  0x36, 0x35, 0x2e, 0x1b, 0x00, 0x00, 0x00, 0x00,
  0x30, 0x2f, 0x29, 0x15, 0x00, 0x00, 0x00, 0x00,
  0x30, 0x2f, 0x29, 0x15, 0x00, 0x00, 0x00, 0x00,
  0x30, 0x2f, 0x29, 0x15, 0x00, 0x00, 0x00, 0x00
};

static const u8 rtl8225z2_tx_power_cck[] = {
  0x36, 0x35, 0x2e, 0x25, 0x1c, 0x12, 0x09, 0x04,
  0x30, 0x2f, 0x29, 0x21, 0x19, 0x10, 0x08, 0x03,
  0x2b, 0x2a, 0x25, 0x1e, 0x16, 0x0e, 0x07, 0x03,
  0x26, 0x25, 0x21, 0x1b, 0x14, 0x0d, 0x06, 0x03
};

static const u8 rtl8225z2_tx_power_ofdm[] = {
  0x42, 0x00, 0x40, 0x00, 0x40
};

static const u8 rtl8225z2_tx_gain_cck_ofdm[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
  0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11,
  0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
  0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23
};

static void rtl8225z2_rf_set_tx_power(int channel)
{
  u8 cck_power, ofdm_power;
  const u8 *tmp;
  u32 reg;
  int i;

  cck_power = channels[channel - 1].hw_value & 0xF;
  ofdm_power = channels[channel - 1].hw_value >> 4;

  cck_power = min(cck_power, (u8)15);
  cck_power += txpwr_base & 0xF;
  cck_power = min(cck_power, (u8)35);

  if (ofdm_power > (u8)15)
    ofdm_power = 25;
  else
    ofdm_power += 10;
  ofdm_power += txpwr_base >> 4;
  ofdm_power = min(ofdm_power, (u8)35);

  if (channel == 14)
    tmp = rtl8225z2_tx_power_cck_ch14;
  else
    tmp = rtl8225z2_tx_power_cck;

  for (i = 0; i < 8; i++)
    rtl8225_write_phy_cck(0x44 + i, *tmp++);

  iowrite8(&map->TX_GAIN_CCK,
                   rtl8225z2_tx_gain_cck_ofdm[cck_power]);
  msleep(1);

  /* anaparam2 on */
  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8(&map->CONFIG3);
  iowrite8(&map->CONFIG3,
                   reg | RTL818X_CONFIG3_ANAPARAM_WRITE);
  iowrite32(&map->ANAPARAM2,
                    RTL8187_RTL8225_ANAPARAM2_ON);
  iowrite8(&map->CONFIG3,
                   reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);
  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);

  rtl8225_write_phy_ofdm(2, 0x42);
  rtl8225_write_phy_ofdm(5, 0x00);
  rtl8225_write_phy_ofdm(6, 0x40);
  rtl8225_write_phy_ofdm(7, 0x00);
  rtl8225_write_phy_ofdm(8, 0x40);

  iowrite8(&map->TX_GAIN_OFDM,
                   rtl8225z2_tx_gain_cck_ofdm[ofdm_power]);
  msleep(1);
}

static void rtl8225z2_b_rf_set_tx_power(int channel)
{
  u8 cck_power, ofdm_power;
  const u8 *tmp;
  int i;

  cck_power = channels[channel - 1].hw_value & 0xF;
  ofdm_power = channels[channel - 1].hw_value >> 4;

  if (cck_power > 15)
    cck_power = (hw_rev == RTL8187BvB) ? 15 : 22;
  else
    cck_power += (hw_rev == RTL8187BvB) ? 0 : 7;
  cck_power += txpwr_base & 0xF;
  cck_power = min(cck_power, (u8)35);

  if (ofdm_power > 15)
    ofdm_power = (hw_rev == RTL8187BvB) ? 17 : 25;
  else
    ofdm_power += (hw_rev == RTL8187BvB) ? 2 : 10;
  ofdm_power += (txpwr_base >> 4) & 0xF;
  ofdm_power = min(ofdm_power, (u8)35);

  if (channel == 14)
    tmp = rtl8225z2_tx_power_cck_ch14;
  else
    tmp = rtl8225z2_tx_power_cck;

  if (hw_rev == RTL8187BvB) {
    if (cck_power <= 6)
      ; /* do nothing */
    else if (cck_power <= 11)
      tmp += 8;
    else
      tmp += 16;
  } else {
    if (cck_power <= 5)
      ; /* do nothing */
    else if (cck_power <= 11)
      tmp += 8;
    else if (cck_power <= 17)
      tmp += 16;
    else
      tmp += 24;
  }

  for (i = 0; i < 8; i++)
    rtl8225_write_phy_cck(0x44 + i, *tmp++);

  iowrite8(&map->TX_GAIN_CCK,
                   rtl8225z2_tx_gain_cck_ofdm[cck_power] << 1);
  msleep(1);

  iowrite8(&map->TX_GAIN_OFDM,
                   rtl8225z2_tx_gain_cck_ofdm[ofdm_power] << 1);
  if (hw_rev == RTL8187BvB) {
    if (ofdm_power <= 11) {
      rtl8225_write_phy_ofdm(0x87, 0x60);
      rtl8225_write_phy_ofdm(0x89, 0x60);
    } else {
      rtl8225_write_phy_ofdm(0x87, 0x5c);
      rtl8225_write_phy_ofdm(0x89, 0x5c);
    }
  } else {
    if (ofdm_power <= 11) {
      rtl8225_write_phy_ofdm(0x87, 0x5c);
      rtl8225_write_phy_ofdm(0x89, 0x5c);
    } else if (ofdm_power <= 17) {
      rtl8225_write_phy_ofdm(0x87, 0x54);
      rtl8225_write_phy_ofdm(0x89, 0x54);
    } else {
      rtl8225_write_phy_ofdm(0x87, 0x50);
      rtl8225_write_phy_ofdm(0x89, 0x50);
    }
  }
  msleep(1);
}

static const u16 rtl8225z2_rxgain[] = {
  0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0405, 0x0408, 0x0409,
  0x040a, 0x040b, 0x0502, 0x0503, 0x0504, 0x0505, 0x0540, 0x0541,
  0x0542, 0x0543, 0x0544, 0x0545, 0x0580, 0x0581, 0x0582, 0x0583,
  0x0584, 0x0585, 0x0588, 0x0589, 0x058a, 0x058b, 0x0643, 0x0644,
  0x0645, 0x0680, 0x0681, 0x0682, 0x0683, 0x0684, 0x0685, 0x0688,
  0x0689, 0x068a, 0x068b, 0x068c, 0x0742, 0x0743, 0x0744, 0x0745,
  0x0780, 0x0781, 0x0782, 0x0783, 0x0784, 0x0785, 0x0788, 0x0789,
  0x078a, 0x078b, 0x078c, 0x078d, 0x0790, 0x0791, 0x0792, 0x0793,
  0x0794, 0x0795, 0x0798, 0x0799, 0x079a, 0x079b, 0x079c, 0x079d,
  0x07a0, 0x07a1, 0x07a2, 0x07a3, 0x07a4, 0x07a5, 0x07a8, 0x07a9,
  0x03aa, 0x03ab, 0x03ac, 0x03ad, 0x03b0, 0x03b1, 0x03b2, 0x03b3,
  0x03b4, 0x03b5, 0x03b8, 0x03b9, 0x03ba, 0x03bb, 0x03bb
};

static const u8 rtl8225z2_gain_bg[] = {
  0x23, 0x15, 0xa5, /* -82-1dBm */
  0x23, 0x15, 0xb5, /* -82-2dBm */
  0x23, 0x15, 0xc5, /* -82-3dBm */
  0x33, 0x15, 0xc5, /* -78dBm */
  0x43, 0x15, 0xc5, /* -74dBm */
  0x53, 0x15, 0xc5, /* -70dBm */
  0x63, 0x15, 0xc5  /* -66dBm */
};

static void
rtl8225z2_rf_init(void)
{
  int i;

  rtl8225_write(0x0, 0x2BF);
  rtl8225_write(0x1, 0xEE0);
  rtl8225_write(0x2, 0x44D);
  rtl8225_write(0x3, 0x441);
  rtl8225_write(0x4, 0x8C3);
  rtl8225_write(0x5, 0xC72);
  rtl8225_write(0x6, 0x0E6);
  rtl8225_write(0x7, 0x82A);
  rtl8225_write(0x8, 0x03F);
  rtl8225_write(0x9, 0x335);
  rtl8225_write(0xa, 0x9D4);
  rtl8225_write(0xb, 0x7BB);
  rtl8225_write(0xc, 0x850);
  rtl8225_write(0xd, 0xCDF);
  rtl8225_write(0xe, 0x02B);
  rtl8225_write(0xf, 0x114);
  msleep(100);

  rtl8225_write(0x0, 0x1B7);

  for (i = 0; i < ARRAY_SIZE(rtl8225z2_rxgain); i++) {
    rtl8225_write(0x1, i + 1);
    rtl8225_write(0x2, rtl8225z2_rxgain[i]);
  }

  rtl8225_write(0x3, 0x080);
  rtl8225_write(0x5, 0x004);
  rtl8225_write(0x0, 0x0B7);
  rtl8225_write(0x2, 0xc4D);

  msleep(200);
  rtl8225_write(0x2, 0x44D);
  msleep(100);

  if (!(rtl8225_read(6) & (1 << 7))) {
    rtl8225_write(0x02, 0x0C4D);
    msleep(200);
    rtl8225_write(0x02, 0x044D);
    msleep(100);
    if (!(rtl8225_read(6) & (1 << 7)))
      DLOG("RF Calibration Failed! %x\n",
           rtl8225_read(6));
  }

  msleep(200);

  rtl8225_write(0x0, 0x2BF);

  for (i = 0; i < ARRAY_SIZE(rtl8225_agc); i++) {
    rtl8225_write_phy_ofdm(0xB, rtl8225_agc[i]);
    rtl8225_write_phy_ofdm(0xA, 0x80 + i);
  }

  msleep(1);

  rtl8225_write_phy_ofdm(0x00, 0x01);
  rtl8225_write_phy_ofdm(0x01, 0x02);
  rtl8225_write_phy_ofdm(0x02, 0x42);
  rtl8225_write_phy_ofdm(0x03, 0x00);
  rtl8225_write_phy_ofdm(0x04, 0x00);
  rtl8225_write_phy_ofdm(0x05, 0x00);
  rtl8225_write_phy_ofdm(0x06, 0x40);
  rtl8225_write_phy_ofdm(0x07, 0x00);
  rtl8225_write_phy_ofdm(0x08, 0x40);
  rtl8225_write_phy_ofdm(0x09, 0xfe);
  rtl8225_write_phy_ofdm(0x0a, 0x08);
  rtl8225_write_phy_ofdm(0x0b, 0x80);
  rtl8225_write_phy_ofdm(0x0c, 0x01);
  rtl8225_write_phy_ofdm(0x0d, 0x43);
  rtl8225_write_phy_ofdm(0x0e, 0xd3);
  rtl8225_write_phy_ofdm(0x0f, 0x38);
  rtl8225_write_phy_ofdm(0x10, 0x84);
  rtl8225_write_phy_ofdm(0x11, 0x07);
  rtl8225_write_phy_ofdm(0x12, 0x20);
  rtl8225_write_phy_ofdm(0x13, 0x20);
  rtl8225_write_phy_ofdm(0x14, 0x00);
  rtl8225_write_phy_ofdm(0x15, 0x40);
  rtl8225_write_phy_ofdm(0x16, 0x00);
  rtl8225_write_phy_ofdm(0x17, 0x40);
  rtl8225_write_phy_ofdm(0x18, 0xef);
  rtl8225_write_phy_ofdm(0x19, 0x19);
  rtl8225_write_phy_ofdm(0x1a, 0x20);
  rtl8225_write_phy_ofdm(0x1b, 0x15);
  rtl8225_write_phy_ofdm(0x1c, 0x04);
  rtl8225_write_phy_ofdm(0x1d, 0xc5);
  rtl8225_write_phy_ofdm(0x1e, 0x95);
  rtl8225_write_phy_ofdm(0x1f, 0x75);
  rtl8225_write_phy_ofdm(0x20, 0x1f);
  rtl8225_write_phy_ofdm(0x21, 0x17);
  rtl8225_write_phy_ofdm(0x22, 0x16);
  rtl8225_write_phy_ofdm(0x23, 0x80);
  rtl8225_write_phy_ofdm(0x24, 0x46);
  rtl8225_write_phy_ofdm(0x25, 0x00);
  rtl8225_write_phy_ofdm(0x26, 0x90);
  rtl8225_write_phy_ofdm(0x27, 0x88);

  rtl8225_write_phy_ofdm(0x0b, rtl8225z2_gain_bg[4 * 3]);
  rtl8225_write_phy_ofdm(0x1b, rtl8225z2_gain_bg[4 * 3 + 1]);
  rtl8225_write_phy_ofdm(0x1d, rtl8225z2_gain_bg[4 * 3 + 2]);
  rtl8225_write_phy_ofdm(0x21, 0x37);

  rtl8225_write_phy_cck(0x00, 0x98);
  rtl8225_write_phy_cck(0x03, 0x20);
  rtl8225_write_phy_cck(0x04, 0x7e);
  rtl8225_write_phy_cck(0x05, 0x12);
  rtl8225_write_phy_cck(0x06, 0xfc);
  rtl8225_write_phy_cck(0x07, 0x78);
  rtl8225_write_phy_cck(0x08, 0x2e);
  rtl8225_write_phy_cck(0x10, 0x9b);
  rtl8225_write_phy_cck(0x11, 0x88);
  rtl8225_write_phy_cck(0x12, 0x47);
  rtl8225_write_phy_cck(0x13, 0xd0);
  rtl8225_write_phy_cck(0x19, 0x00);
  rtl8225_write_phy_cck(0x1a, 0xa0);
  rtl8225_write_phy_cck(0x1b, 0x08);
  rtl8225_write_phy_cck(0x40, 0x86);
  rtl8225_write_phy_cck(0x41, 0x8d);
  rtl8225_write_phy_cck(0x42, 0x15);
  rtl8225_write_phy_cck(0x43, 0x18);
  rtl8225_write_phy_cck(0x44, 0x36);
  rtl8225_write_phy_cck(0x45, 0x35);
  rtl8225_write_phy_cck(0x46, 0x2e);
  rtl8225_write_phy_cck(0x47, 0x25);
  rtl8225_write_phy_cck(0x48, 0x1c);
  rtl8225_write_phy_cck(0x49, 0x12);
  rtl8225_write_phy_cck(0x4a, 0x09);
  rtl8225_write_phy_cck(0x4b, 0x04);
  rtl8225_write_phy_cck(0x4c, 0x05);

  iowrite8((u8 *)0xFF5B, 0x0D); msleep(1);

  rtl8225z2_rf_set_tx_power(1);

  /* RX antenna default to A */
  rtl8225_write_phy_cck(0x10, 0x9b);			/* B: 0xDB */
  rtl8225_write_phy_ofdm(0x26, 0x90);		/* B: 0x10 */

  iowrite8(&map->TX_ANTENNA, 0x03);	/* B: 0x00 */
  msleep(1);
  iowrite32((__le32 *)0xFF94, 0x3dc00002);
}

static void
rtl8225z2_b_rf_init(void)
{
  int i;

  DLOG ("rtl8225z2_b_rf_init");

  rtl8225_write(0x0, 0x0B7);
  rtl8225_write(0x1, 0xEE0);
  rtl8225_write(0x2, 0x44D);
  rtl8225_write(0x3, 0x441);
  rtl8225_write(0x4, 0x8C3);
  rtl8225_write(0x5, 0xC72);
  rtl8225_write(0x6, 0x0E6);
  rtl8225_write(0x7, 0x82A);
  rtl8225_write(0x8, 0x03F);
  rtl8225_write(0x9, 0x335);
  rtl8225_write(0xa, 0x9D4);
  rtl8225_write(0xb, 0x7BB);
  rtl8225_write(0xc, 0x850);
  rtl8225_write(0xd, 0xCDF);
  rtl8225_write(0xe, 0x02B);
  rtl8225_write(0xf, 0x114);

  rtl8225_write(0x0, 0x1B7);

#if 1
  DLOG ("writing rxgain");
  for (i = 0; i < ARRAY_SIZE(rtl8225z2_rxgain); i++) {
    DLOG ("rxgain[%d]", i);
    rtl8225_write(0x1, i + 1);
    rtl8225_write(0x2, rtl8225z2_rxgain[i]);
  }
#endif

  rtl8225_write(0x3, 0x080);
  rtl8225_write(0x5, 0x004);
  rtl8225_write(0x0, 0x0B7);

  rtl8225_write(0x2, 0xC4D);

  rtl8225_write(0x2, 0x44D);
  rtl8225_write(0x0, 0x2BF);

  iowrite8(&map->TX_GAIN_CCK, 0x03);
  iowrite8(&map->TX_GAIN_OFDM, 0x07);
  iowrite8(&map->TX_ANTENNA, 0x03);

#if 1
  DLOG ("writing agc");
  rtl8225_write_phy_ofdm(0x80, 0x12);
  for (i = 0; i < ARRAY_SIZE(rtl8225z2_agc); i++) {
    rtl8225_write_phy_ofdm(0xF, rtl8225z2_agc[i]);
    rtl8225_write_phy_ofdm(0xE, 0x80 + i);
    rtl8225_write_phy_ofdm(0xE, 0);
  }
  rtl8225_write_phy_ofdm(0x80, 0x10);

  DLOG ("writing ofdm");
  for (i = 0; i < ARRAY_SIZE(rtl8225z2_ofdm); i++)
    rtl8225_write_phy_ofdm(i, rtl8225z2_ofdm[i]);

  rtl8225_write_phy_ofdm(0x97, 0x46);
  rtl8225_write_phy_ofdm(0xa4, 0xb6);
  rtl8225_write_phy_ofdm(0x85, 0xfc);
  rtl8225_write_phy_cck(0xc1, 0x88);
#endif

  DLOG ("rf init finished");
}

static void rtl8225_rf_stop(void)
{
  u8 reg;

  rtl8225_write(0x4, 0x1f);

  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_CONFIG);
  reg = ioread8(&map->CONFIG3);
  iowrite8(&map->CONFIG3, reg | RTL818X_CONFIG3_ANAPARAM_WRITE);
  if (!is_rtl8187b) {
    iowrite32(&map->ANAPARAM2,
                      RTL8187_RTL8225_ANAPARAM2_OFF);
    iowrite32(&map->ANAPARAM,
                      RTL8187_RTL8225_ANAPARAM_OFF);
  } else {
    iowrite32(&map->ANAPARAM2,
                      RTL8187B_RTL8225_ANAPARAM2_OFF);
    iowrite32(&map->ANAPARAM,
                      RTL8187B_RTL8225_ANAPARAM_OFF);
    iowrite8(&map->ANAPARAM3,
                     RTL8187B_RTL8225_ANAPARAM3_OFF);
  }
  iowrite8(&map->CONFIG3, reg & ~RTL818X_CONFIG3_ANAPARAM_WRITE);
  iowrite8(&map->EEPROM_CMD, RTL818X_EEPROM_CMD_NORMAL);
}

static void
rtl8225_rf_set_channel(struct ieee80211_conf *conf)
{
  int chan = ieee80211_frequency_to_channel(conf->channel->center_freq);

  if (rf->init == rtl8225_rf_init)
    rtl8225_rf_set_tx_power(chan);
  else if (rf->init == rtl8225z2_rf_init)
    rtl8225z2_rf_set_tx_power(chan);
  else
    rtl8225z2_b_rf_set_tx_power(chan);

  rtl8225_write(0x7, rtl8225_chan[chan - 1]);
  msleep(10);
}

static const struct rtl818x_rf_ops rtl8225_ops = {
  .name		= "rtl8225",
  .init		= rtl8225_rf_init,
  .stop		= rtl8225_rf_stop,
  .set_chan	= rtl8225_rf_set_channel
};

static const struct rtl818x_rf_ops rtl8225z2_ops = {
  .name		= "rtl8225z2",
  .init		= rtl8225z2_rf_init,
  .stop		= rtl8225_rf_stop,
  .set_chan	= rtl8225_rf_set_channel
};

static const struct rtl818x_rf_ops rtl8225z2_b_ops = {
  .name		= "rtl8225z2",
  .init		= rtl8225z2_b_rf_init,
  .stop		= rtl8225_rf_stop,
  .set_chan	= rtl8225_rf_set_channel
};

const struct rtl818x_rf_ops *
rtl8187_detect_rf(void)
{
  u16 reg8, reg9;

  if (!is_rtl8187b) {
    rtl8225_write(0, 0x1B7);

    reg8 = rtl8225_read(8);
    reg9 = rtl8225_read(9);

    rtl8225_write(0, 0x0B7);

    if (reg8 != 0x588 || reg9 != 0x700)
      return &rtl8225_ops;

    return &rtl8225z2_ops;
  } else
    return &rtl8225z2_b_ops;
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
