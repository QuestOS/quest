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
#include <drivers/net/ethernet.h>
#include <util/printf.h>
#include <kernel.h>
#include "sched/sched.h"

//#define DEBUG_ASIX

#ifdef DEBUG_ASIX
#define DLOG(fmt,...) DLOG_PREFIX("asix",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static uint8 ethaddr[ETH_ADDR_LEN];
static uint status_ept, status_maxpkt;
static uint data_ept_in, data_ept_out, data_maxpkt;

static USB_DEVICE_INFO *ethusbdev;
static ethernet_device usbnet_ethdev;

#define SOFTWARE_MII 0x06
#define PHY_READ_REG 0x07
#define PHY_WRITE_REG 0x08
#define MII_STATUS 0x09
#define HARDWARE_MII 0x0A
#define RX_CTRL_READ 0x0F
#define RX_CTRL_WRITE 0x10
#define READ_IPG 0x11
#define WRITE_IPG 0x12
#define GET_NODE_ID 0x13
#define ETH_PHY_ID 0x19
#define MEDIUM_STATUS 0x1A
#define MEDIUM_MODE 0x1B
#define GPIO_WRITE 0x1F
#define SW_RESET 0x20
#define SW_PHY_SELECT 0x22

#define SWRESET_CLEAR 0x00
#define SWRESET_RR    0x01
#define SWRESET_RT    0x02
#define SWRESET_PRTE  0x04
#define SWRESET_PRL   0x08
#define SWRESET_BZ    0x10
#define SWRESET_IPRL  0x20
#define SWRESET_IPPD  0x40

#define MII_BMCR 0x00
#define MII_BMSR 0x01
#define MII_PHYSID1 0x02
#define MII_PHYSID2 0x03
#define MII_ADVERTISE 0x04

#define BMCR_RESET 0x8000
#define BMCR_ANRESTART 0x0200
#define BMCR_ANENABLE 0x1000
#define BMCR_LOOPBACK 0x4000

#define ADVERTISE_CSMA 0x0001
#define ADVERTISE_10HALF  0x0020
#define ADVERTISE_100HALF 0x0080
#define ADVERTISE_10FULL 0x0040
#define ADVERTISE_100FULL 0x100
#define ADVERTISE_ALL (ADVERTISE_10HALF | ADVERTISE_10FULL | \
                       ADVERTISE_100HALF | ADVERTISE_100FULL)

#define MEDIUM_PF   0x0080
#define MEDIUM_JFE  0x0040
#define MEDIUM_TFC  0x0020
#define MEDIUM_RFC  0x0010
#define MEDIUM_ENCK 0x0008
#define MEDIUM_AC   0x0004
#define MEDIUM_FD   0x0002
#define MEDIUM_GM   0x0001
#define MEDIUM_SM   0x1000
#define MEDIUM_SBP  0x0800
#define MEDIUM_PS   0x0200
#define MEDIUM_RE   0x0100

#define AX88772_MEDIUM_DEFAULT  \
        (MEDIUM_FD | MEDIUM_RFC | \
         MEDIUM_TFC | MEDIUM_PS | \
         MEDIUM_AC | MEDIUM_RE )

/* GPIO 0 .. 2 toggles */
#define GPIO_GPO0EN   0x01    /* GPIO0 Output enable */
#define GPIO_GPO_0    0x02    /* GPIO0 Output value */
#define GPIO_GPO1EN   0x04    /* GPIO1 Output enable */
#define GPIO_GPO_1    0x08    /* GPIO1 Output value */
#define GPIO_GPO2EN   0x10    /* GPIO2 Output enable */
#define GPIO_GPO_2    0x20    /* GPIO2 Output value */
#define GPIO_RESERVED 0x40    /* Reserved */
#define GPIO_RSE      0x80    /* Reload serial EEPROM */

#define AX88772_IPG0_DEFAULT 0x15
#define AX88772_IPG1_DEFAULT 0x0C
#define AX88772_IPG2_DEFAULT 0x12

static bool
send_cmd (bool input, uint8 cmd, uint16 val, uint16 index,
          uint16 len, uint8 *buf)
{
  USB_DEV_REQ setup_req;

  setup_req.bmRequestType = (input ? 0x80 : 0x00) | 0x40;
  setup_req.bRequest = cmd;
  setup_req.wValue = val;
  setup_req.wIndex = index;
  setup_req.wLength = len;

  {
    uint8 *ptr = (uint8 *) &setup_req;
    DLOG ("send_cmd : %.02X%.02X_%.02X%.02X_%.02X%.02X_%.02X%.02X",
          ptr[0], ptr[1], ptr[2], ptr[3],
          ptr[4], ptr[5], ptr[6], ptr[7]);
  }

  return usb_control_transfer (ethusbdev, &setup_req, sizeof (USB_DEV_REQ),
                               buf, len) == 0;
}

static inline bool
sw_mii (void)
{
  return send_cmd (FALSE, SOFTWARE_MII, 0, 0, 0, NULL);
}

static inline bool
hw_mii (void)
{
  return send_cmd (FALSE, HARDWARE_MII, 0, 0, 0, NULL);
}

static inline bool
mdio_write (uint16 phy_id, uint16 loc, uint16 val)
{
  bool ret;
  sw_mii ();
  ret = send_cmd (FALSE, PHY_WRITE_REG, phy_id, loc, 2, (uint8 *) &val);
  hw_mii ();
  return ret;
}

static inline uint16
mdio_read (uint16 phy_id, uint16 loc)
{
  uint16 val;
  sw_mii ();
  if (!send_cmd (TRUE, PHY_READ_REG, phy_id, loc, 2, (uint8 *) &val))
    val = 0;
  hw_mii ();
  return val;
}

static uint16
read_rx_ctrl (void)
{
  uint16 rx = 0;
  if (send_cmd (TRUE, RX_CTRL_READ, 0, 0, 2, (uint8 *) &rx))
    return rx;
  else
    return 0;
}

static bool
write_rx_ctrl (uint16 rx)
{
  return send_cmd (FALSE, RX_CTRL_WRITE, rx, 0, 0, NULL);
}

static bool
reset (void)
{
  uint32 phy_id, phy_reg1, phy_reg2;
  uint16 bmcr, medium;

  /* setup GPIO */
  if (!send_cmd (FALSE, GPIO_WRITE, GPIO_RSE | GPIO_GPO_2 | GPIO_GPO2EN,
                 0, 0, NULL))
    goto abort;
  delay (5);

  /* setup embedded or external PHY */
  if (!send_cmd (TRUE, ETH_PHY_ID, 0, 0, 2, (uint8 *)&phy_id))
    goto abort;

  if ((phy_id & 0xE0) == 0xE0) {
    /* lower byte is unsupported PHY */
    phy_id >>= 8;
    if ((phy_id & 0xE0) == 0xE0) {
      DLOG ("no supported PHY");
      goto abort;
    }
  }

  phy_id &= 0x1F;               /* mask ID bits */

  DLOG ("phy_id=0x%x", phy_id);

  if (!send_cmd (FALSE, SW_PHY_SELECT,
                 (phy_id & 0x1F) == 0x10 ? 1 : 0,
                 0, 0, NULL))
    goto abort;

  DLOG ("sending SW reset");
  /* reset card */
  if (!send_cmd (FALSE, SW_RESET, SWRESET_IPPD | SWRESET_PRL, 0, 0, NULL))
    goto abort;
  delay (150);
  if (!send_cmd (FALSE, SW_RESET, SWRESET_CLEAR, 0, 0, NULL))
    goto abort;
  delay (150);
  if (!send_cmd (FALSE, SW_RESET,
                 (phy_id & 0x1F) == 0x10 ? SWRESET_IPRL : SWRESET_PRTE,
                 0, 0, NULL))
    goto abort;
  delay (150);

  /* check RX CTRL */
  DLOG ("RXCTRL=0x%x", read_rx_ctrl ());
  if (!write_rx_ctrl (0x0))
    goto abort;
  DLOG ("wrote 0x0 -- RXCTRL=0x%x", read_rx_ctrl ());

  /* get ethernet address */
  memset (ethaddr, 0, ETH_ADDR_LEN);
  if (!send_cmd (TRUE, GET_NODE_ID, 0, 0, ETH_ADDR_LEN, ethaddr))
    goto abort;

  DLOG ("ethaddr=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
        ethaddr[0], ethaddr[1], ethaddr[2], ethaddr[3], ethaddr[4], ethaddr[5]);

  /* get PHY IDENTIFIER (vendor, model) from MII registers */
  phy_reg1 = mdio_read (phy_id, MII_PHYSID1);
  phy_reg2 = mdio_read (phy_id, MII_PHYSID2);

  DLOG ("MII said PHY IDENTIFIER=0x%x",
        ((phy_reg1 & 0xffff) << 16) | (phy_reg2 & 0xffff));


  /* reset card again */
  DLOG ("resending SW reset");
  if (!send_cmd (FALSE, SW_RESET, SWRESET_PRL, 0, 0, NULL))
    goto abort;
  delay (150);
  if (!send_cmd (FALSE, SW_RESET, SWRESET_IPRL | SWRESET_PRL, 0, 0, NULL))
    goto abort;
  delay (150);

  /* init MII */
  mdio_write (phy_id, MII_BMCR, BMCR_RESET);
  mdio_write (phy_id, MII_ADVERTISE, ADVERTISE_ALL | ADVERTISE_CSMA);

  /* autonegotiation */
  bmcr = mdio_read (phy_id, MII_BMCR);
  DLOG ("enabling autonegotiation.  BMCR=0x%x", bmcr);
  if (bmcr & BMCR_ANENABLE) {
    bmcr |= BMCR_ANRESTART;
    mdio_write (phy_id, MII_BMCR, bmcr);
  } else
    goto abort;

  DLOG ("setting medium mode=0x%x", AX88772_MEDIUM_DEFAULT);
  /* setup medium mode */
  if (!send_cmd (FALSE, MEDIUM_MODE, AX88772_MEDIUM_DEFAULT, 0, 0, NULL))
    goto abort;

  /* interpacket gap */
  DLOG ("setting IPG");
  if (!send_cmd (FALSE, WRITE_IPG,
                 AX88772_IPG0_DEFAULT | (AX88772_IPG1_DEFAULT << 8),
                 AX88772_IPG2_DEFAULT, 0, NULL))
    goto abort;

  DLOG ("accepting broadcasts, starting operation");
  /* Accept Broadcasts and Start Operation */
  if (!write_rx_ctrl (0x88))       /* AB | SO */
    goto abort;

  DLOG ("RXCTRL=0x%x at end of reset", read_rx_ctrl ());

  if (!send_cmd (TRUE, MEDIUM_STATUS, 0, 0, 2, (uint8 *) &medium))
    goto abort;
  DLOG ("medium status=0x%x at end of reset", medium);

  bmcr = mdio_read (phy_id, MII_BMCR);
  DLOG ("BMCR=0x%x at end of reset", bmcr);

  DLOG ("BMSR=0x%x at end of reset", mdio_read (phy_id, MII_BMSR));

  return TRUE;
 abort:
  DLOG ("reset failed");
  return FALSE;
}

static bool
get_hwaddr (uint8 addr[ETH_ADDR_LEN])
{
  int i;
  for (i=0; i<ETH_ADDR_LEN; i++)
    addr[i] = ethaddr[i];
  return TRUE;
}

static sint
transmit (uint8* buf, sint len)
{
  static uint8 frame[MAX_FRAME_SIZE];
  sint ret;
  uint32 act_len;
  uint32 prop_len = ((len ^ 0x0000ffff) << 16) | len;

  memcpy (frame, &prop_len, 4);
  memcpy (frame+4, buf, len);

#ifdef DEBUG_ASIX_DATA
  DLOG ("transmitting data len=%d: %.02X %.02X %.02X %.02X", len,
        frame[0], frame[1], frame[2], frame[3]);
  DLOG ("                         %.02X %.02X %.02X %.02X",
        frame[4], frame[5], frame[6], frame[7]);
  DLOG ("                         %.02X %.02X %.02X %.02X",
        frame[8], frame[9], frame[10], frame[11]);
  DLOG ("                         %.02X %.02X %.02X %.02X",
        frame[12], frame[13], frame[14], frame[15]);
#endif

  if (usb_bulk_transfer (ethusbdev, data_ept_out, frame, len+4,
                         data_maxpkt, DIR_OUT, &act_len) == 0)
    ret = act_len;
  else
    ret = 0;
  DLOG ("transmitted %d bytes", act_len);
  return ret;
}

static void
poll (void)
{
  uint8 buffer[1600];
  uint32 act_len;
  if (usb_bulk_transfer (ethusbdev, data_ept_in, buffer, 1514,
                         data_maxpkt, DIR_IN, &act_len) == 0) {
    if (act_len > 0) {
#ifdef DEBUG_ASIX_DATA
      DLOG ("receiving data len=%.04d: %.02X %.02X %.02X %.02X",
            act_len, buffer[0], buffer[1], buffer[2], buffer[3]);
      DLOG ("                         %.02X %.02X %.02X %.02X",
            buffer[4], buffer[5], buffer[6], buffer[7]);
      DLOG ("                         %.02X %.02X %.02X %.02X",
            buffer[8], buffer[9], buffer[10], buffer[11]);
      DLOG ("                         %.02X %.02X %.02X %.02X",
            buffer[12], buffer[13], buffer[14], buffer[15]);
#endif
      usbnet_ethdev.recv_func (&usbnet_ethdev, buffer+4, act_len-4);
    }
  }
}

static void
irq_loop (void)
{
  uint32 tick = 0;
  DLOG ("irq_loop pid=0x%x", str ());
  for (;;) {
    poll ();
    DLOG ("iteration %d", tick);
    tick++;
  }
}

static void
status_loop (void)
{
  uint32 tick = 0, act_len;
  uint32 status[2];
  DLOG ("status_loop pid=0x%x", str ());
  for (;;) {
    if (usb_bulk_transfer (ethusbdev, status_ept, (uint8 *)&status, 8,
                           status_maxpkt, DIR_IN, &act_len) == 0) {
      if (act_len > 0) {
        DLOG ("status update 0x%.08X %.08X", status[1], status[0]);
        if (status[0] & 0x10000)
          DLOG ("  primary link UP");
        if (status[0] & 0x20000)
          DLOG ("  secondary link UP");
        if (status[0] & 0x40000)
          DLOG ("  Bulk Out Ethernet Frame Length Error");
      }
    }
    DLOG ("status iteration %d", tick);
    tick++;
    delay (100);
  }
}


static uint32 irq_stack[1024] ALIGNED(0x1000);
static task_id irq_pid;

static uint32 status_stack[1024] ALIGNED(0x1000);
static task_id status_pid;

static bool
probe (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  uint i;
  USB_EPT_DESC *eptd;

  DLOG ("examining 0x%x 0x%x 0x%x",
        ifd->bInterfaceClass,
        info->devd.idVendor,
        info->devd.idProduct);

  if (!(ifd->bInterfaceClass == 0xFF &&
        ifd->bInterfaceSubClass == 0xFF &&
        ifd->bInterfaceProtocol == 0x00 &&
        /* ASIX */
        info->devd.idVendor == 0x0b95 &&
        /* Cables-to-go ASIX 88772 */
        info->devd.idProduct == 0x772a))
    return FALSE;

  /* expect endpoints to follow interface */
  eptd = (USB_EPT_DESC *) &ifd[1];

  status_ept = data_ept_in = data_ept_out = 0;

  /* 3 endpoints: 1 interrupt, 2 bulk */
  for (i=0; i<3; i++) {
    if ((eptd->bmAttributes & 0x3) == 0x3) {
      /* interrupt endpoint */
      status_ept = eptd->bEndpointAddress & 0x7F;
      status_maxpkt = eptd->wMaxPacketSize;
    } else if ((eptd->bmAttributes & 0x3) == 0x2) {
      /* bulk endpoint */
      data_maxpkt = eptd->wMaxPacketSize;
      if (eptd->bEndpointAddress & 0x80)
        data_ept_in = eptd->bEndpointAddress & 0x7F;
      else
        data_ept_out = eptd->bEndpointAddress & 0x7F;
    }
    eptd = &eptd[1];
  }

  DLOG ("status_ept=%d data_ept_in=%d data_ept_out=%d data_maxpkt=%d",
        status_ept, data_ept_in, data_ept_out, data_maxpkt);

  if (!status_ept || !data_ept_in || !data_ept_out)
    return FALSE;

  if (usb_set_configuration (info, cfgd->bConfigurationValue) != 0) {
    DLOG ("set_configuration: failed");
    return FALSE;
  }

  ethusbdev = info;

  if (!reset ())
    return FALSE;

  /* Register network device with net subsystem */
  usbnet_ethdev.recv_func = NULL;
  usbnet_ethdev.send_func = transmit;
  usbnet_ethdev.get_hwaddr_func = get_hwaddr;
  usbnet_ethdev.poll_func = poll;

  if (!net_register_device (&usbnet_ethdev)) {
    DLOG ("registration failed");
    return FALSE;
  }

  irq_pid = start_kernel_thread ((uint) irq_loop, (uint) &irq_stack[1023]);
#if 0
  status_pid = start_kernel_thread ((uint) status_loop,
                                    (uint) &status_stack[1023]);
#endif

  return TRUE;
}

static USB_DRIVER asix_driver = {
  .probe = probe
};

extern bool
usb_asix_driver_init (void)
{
  DLOG ("init");
  return usb_register_driver (&asix_driver);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_asix_driver_init
};

DEF_MODULE (usb___asix, "USB asix network driver", &mod_ops, {"usb", "net___ethernet"});


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
