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

#define DEBUG_ASIX

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
#define GET_NODE_ID 0x13
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
  /* reset card */
  if (!send_cmd (FALSE, SW_RESET, SWRESET_IPPD | SWRESET_PRL, 0, 0, NULL))
    goto abort;
  delay (150);
  if (!send_cmd (FALSE, SW_RESET, SWRESET_CLEAR, 0, 0, NULL))
    goto abort;
  delay (150);
  if (!send_cmd (FALSE, SW_RESET, SWRESET_IPRL, 0, 0, NULL))
    goto abort;
  delay (150);

  /* check RX CTRL */
  DLOG ("RXCTRL=0x%x", read_rx_ctrl ());

  /* Accept Broadcasts and Start Operation */
  if (!write_rx_ctrl (0x88))       /* AB | SO */
    goto abort;

  DLOG ("after write RXCTRL=0x%x", read_rx_ctrl ());

  /* get ethernet address */
  memset (ethaddr, 0, ETH_ADDR_LEN);
  return send_cmd (TRUE, GET_NODE_ID, 0, 0, ETH_ADDR_LEN, ethaddr);

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
  sint ret;
  uint32 act_len;

  DLOG ("transmitting data len=%d: %.02X %.02X %.02X %.02X", len,
        buf[0], buf[1], buf[2], buf[3]);
  if (usb_bulk_transfer (ethusbdev, data_ept_out, buf, len,
                         data_maxpkt, DIR_OUT, &act_len) == 0)
    ret = len;
  else
    ret = 0;

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
      DLOG ("receiving data len=%d: %.02X %.02X %.02X %.02X",
            act_len, buffer[0], buffer[1], buffer[2], buffer[3]);
      usbnet_ethdev.recv_func (&usbnet_ethdev, buffer, act_len);
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

static uint32 irq_stack[1024] ALIGNED(0x1000);
static task_id irq_pid;

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

  DLOG ("ethaddr=%.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
        ethaddr[0], ethaddr[1], ethaddr[2], ethaddr[3], ethaddr[4], ethaddr[5]);

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



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
