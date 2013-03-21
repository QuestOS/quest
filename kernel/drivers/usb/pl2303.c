/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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
#include <drivers/usb/pl2303.h>
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>

#define DEBUG_PL2303

#ifdef DEBUG_PL2303
#define DLOG(fmt,...) DLOG_PREFIX("pl2303",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static USB_DEVICE_INFO pl2303_dev;
static uint8_t in_ept = 0, out_ept = 0;
bool pl2303_initialized = FALSE;
static bool write_urb_initialized = FALSE;
static struct urb* pl2303_urb = NULL;

#define COM_BUFFER_SIZE 200
static char com_buffer[COM_BUFFER_SIZE];
static char com_buffer2[COM_BUFFER_SIZE];
static int com_buffer_end = 0;

int usb_pl2303_write (char *, uint32_t);
void usb_pl2303_putc (char);
int usb_pl2303_read (unsigned char *, uint32_t);
char usb_pl2303_getc (void);

/* Flow control setting */
static int
pl2303_set_control (USB_DEVICE_INFO * dev, uint8_t on)
{
  USB_DEV_REQ req;
  uint8_t control = 0;

  if (on)
    control |= (USB_PL2303_CONTROL_DTR | USB_PL2303_CONTROL_RTS);
  else
    control &= ~(USB_PL2303_CONTROL_DTR | USB_PL2303_CONTROL_RTS);

  /* req.bmRequestType = 0x21; */
  /* req.bRequest = USB_PL2303_SET_CONTROL; */
  /* req.wValue = control; */
  /* req.wIndex = 0; */
  /* req.wLength = 0; */

  return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), USB_PL2303_SET_CONTROL, 0x21,
                         control, 0, & req, 0, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
  //return usb_control_transfer (dev, (addr_t) & req,
  //    sizeof (USB_DEV_REQ), 0, 0);
}

static PL2303_CONFIG
pl2303_get_line (USB_DEVICE_INFO * dev)
{
  PL2303_CONFIG data;
  memset (&data, 0, sizeof (PL2303_CONFIG));

  if(usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), USB_PL2303_GET_LINE, 0xA1, 0, 0, &data,
                     7, USB_DEFAULT_CONTROL_MSG_TIMEOUT) < 0) {
  //if (usb_control_transfer (dev, (addr_t) & req, sizeof (USB_DEV_REQ),
  //    (addr_t) & data, req.wLength)) {
    DLOG ("pl2303_get_line failed!");
    memset (&data, 0, sizeof (PL2303_CONFIG));
  }

  return data;
}

static int
pl2303_set_line (
    USB_DEVICE_INFO * dev,
    uint32_t baud,
    uint8_t bits, /* Number of data bits */
    uint8_t parity, /* 0 = None, 1 = Odd, 2 = Even, 3 = Mark, 4 = Space */
    uint8_t stb) /* Stop Bits 0 = 1, 1 = 1.5, 2 = 2 */
{
  PL2303_CONFIG data;
  memset (&data, 0, sizeof (PL2303_CONFIG));

  data = pl2303_get_line (dev);
  
  if (data.baud_rate) {
    DLOG ("Current baud rate: %d", data.baud_rate);
    DLOG ("Current Stop Bits (0=1,1=1.5,2=2): %d", data.stop_bits);
    DLOG ("Current Parity (0=none,1=odd,2=even,3=mark,4=space): %d",
        data.parity);
    DLOG ("Current Data Bits: %d", data.data_bits);
  }

  data.baud_rate = baud;
  data.stop_bits = stb;
  data.parity = parity;
  data.data_bits = bits;

  /* req.bmRequestType = 0x21; */
  /* req.bRequest = USB_PL2303_SET_LINE; */
  /* req.wValue = 0; */
  /* req.wIndex = 0; */
  /* req.wLength = 7; */

  //return usb_control_transfer (dev, (addr_t) & req,
  //    sizeof (USB_DEV_REQ), (addr_t) & data, req.wLength);
  return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), USB_PL2303_SET_LINE, 0x21, 0, 0, &data, 7,
                         USB_DEFAULT_CONTROL_MSG_TIMEOUT);
}

static bool
pl2303_init (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  uint8_t tmp[70];
  int i = 0;
  USB_EPT_DESC *pl2303ept;
  PL2303_CONFIG conf;
  memset (&conf, 0, sizeof (PL2303_CONFIG));
  
  memcpy(&pl2303_dev, dev, sizeof(USB_DEVICE_INFO));
  memset (tmp, 0, 70);

  /* Parsing endpoints */
  DLOG ("Parsing PL2303 chip endpoints ...");
  usb_get_descriptor (dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength,
      (addr_t)tmp);
  for (i = 0; i < cfg->wTotalLength; i += pl2303ept->bLength) {
    pl2303ept = (USB_EPT_DESC *) (tmp + i);
    if ((pl2303ept->bDescriptorType == USB_TYPE_EPT_DESC) &&
        ((pl2303ept->bmAttributes & 0x3) == 0x2)) {
      DLOG ("Found Bulk Endpoint");
      switch (pl2303ept->bEndpointAddress & 0x80) {
        case 0x80 :
          in_ept = pl2303ept->bEndpointAddress & 0xF;
          pl2303_dev.ep_in[in_ept].desc = *pl2303ept;
          print_ept_desc_info(pl2303ept);
          DLOG ("IN Endpoint. Address is: 0x%X", in_ept);
          break;

        case 0x00 :
          out_ept = pl2303ept->bEndpointAddress & 0xF;
          pl2303_dev.ep_out[out_ept].desc = *pl2303ept;
          print_ept_desc_info(pl2303ept);
          DLOG ("OUT Endpoint. Address is: 0x%X", out_ept);
          break;

        default :
          break;
      }
    }
  }

  /* Serial port configuration */
  DLOG ("Configuring serial interface (38400, 8N1)...");
  if (pl2303_set_line (dev, 38400, 8, 0, 0)) {
    DLOG ("Serial interface configuration failed");
    return FALSE;
  }

  DLOG ("Checking configuration ...");
  conf = pl2303_get_line (dev);
  
  DLOG ("Current baud rate: %d", conf.baud_rate);
  DLOG ("Current Stop Bits (0=1,1=1.5,2=2): %d", conf.stop_bits);
  DLOG ("Current Parity (0=none,1=odd,2=even,3=mark,4=space): %d",
      conf.parity);
  DLOG ("Current Data Bits: %d", conf.data_bits);

#if 1
  /* Setting flow control */
  DLOG ("Setting flow control ...");
  if (pl2303_set_control (dev, 1)) {
    DLOG ("Setting flow control failed");
    return FALSE;
  }
#endif

  

  return TRUE;
}

SQUELCH_UNUSED
static void
test ()
{
  DLOG ("PL2303 Test\n");
#if 0
  char c = 0;
  while (c != 0xD) {
    c = usb_pl2303_getc ();
    DLOG ("Got character : %c", c);
    usb_pl2303_putc (c);
  }
#else
  usb_pl2303_putc ('Q');
  usb_pl2303_putc ('u');
  usb_pl2303_putc ('e');
  usb_pl2303_putc ('s');
  usb_pl2303_putc ('t');
  usb_pl2303_putc ('\r');
  usb_pl2303_putc ('\n');
#endif
}

static bool
pl2303_probe (USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  uint pipe;
  if (dev->devd.idVendor == 0x067B) {
    DLOG ("Prolific Technology, Inc. device is detected");

    if (dev->devd.idProduct != 0x2303) {
      DLOG ("Device attached is not PL2303. Product ID is: 0x%X",
          dev->devd.idProduct);
      return FALSE;
    }
  } else {
    return FALSE;
  }

  if(pl2303_initialized) {
    return TRUE;
  }

  if (!pl2303_init(dev, cfg, ifd)) {
    DLOG("Initialization failed!");
    return FALSE;
  }

  DLOG ("PL2303 Serial Converter configured");

  pl2303_urb = usb_alloc_urb(0, 0);
  
  pipe = usb_create_pipe(&pl2303_dev, &pl2303_dev.ep_out[out_ept].desc);
  usb_fill_rt_bulk_urb(pl2303_urb, &pl2303_dev, pipe, com_buffer2, COM_BUFFER_SIZE,
                       NULL, NULL, 4, 0);



  //test();
  pl2303_initialized = TRUE;

  return TRUE;
}

void
usb_pl2303_putc (char c)
{
  if (usb_pl2303_write (&c, 1) != 1) {
    DLOG ("PL2302 write failed\n");
  }
}


/* Can't report an errors in this function as it will then be
   recursively calling itself */
int
usb_pl2303_write (char * buf, uint32_t len)
{
  int act_len = 0;
  uint32_t original_len = len;
  int status;
  static volatile bool in_pl2303_write = FALSE;

  if(!pl2303_initialized) return -1;

  if(in_pl2303_write) return len;

  in_pl2303_write = TRUE;
  
  
  if(mp_enabled) {
    
    if(!write_urb_initialized) {
      int res;
    resubmit:
      res = usb_submit_urb(pl2303_urb, 0);
      if(res == -2) {
        goto resubmit;
      }
      else if(res == -1) {
        DLOG ("Failed to submit pl2303 urb\n");
        panic("Failed to submit pl2303 urb");
      }
      write_urb_initialized = TRUE;
    }
    
    usb_rt_free_write_resources(pl2303_urb);

    if(com_buffer_end) {
      /* Push remaining bytes */
      status = usb_rt_push_data(pl2303_urb, com_buffer, com_buffer_end, 0, 0, 0);
      if(status < 0) {
        DLOG("Failed to push data");
        panic("Failed to push data");
      }
      else if(status != com_buffer_end) {
        DLOG("Failed to push all data, status = %d, line = %d", status, __LINE__);
        panic("Failed to push data");
      }
      com_buffer_end = 0;
    }
    
    status = usb_rt_push_data(pl2303_urb, buf, len, 0, 0, 0);
    if(status < 0) {
      DLOG("Failed to push data");
      panic("Failed to push data");
    }
    else if(status != len) {
      DLOG("Failed to push all data, status = %d, line = %d", status, __LINE__);
      panic("Failed to push data");
    }
  }
  else {
    //in_pl2303_write = FALSE;
    //return len;
    while(len) {
      if(len + com_buffer_end >= COM_BUFFER_SIZE) {
        memcpy(&com_buffer[com_buffer_end], buf, COM_BUFFER_SIZE - com_buffer_end);
        status = usb_bulk_msg(&pl2303_dev, usb_sndbulkpipe(&pl2303_dev, out_ept), com_buffer,
                              COM_BUFFER_SIZE, &act_len, USB_DEFAULT_BULK_MSG_TIMEOUT);

        if(status < 0) {
          DLOG("Failed to send data");
          panic("Failed to send data");
        }
        if(act_len != COM_BUFFER_SIZE) {
          DLOG("Failed to send all data");
          panic("Failed to send all data");
        }

        len = len - (COM_BUFFER_SIZE - com_buffer_end);
        com_buffer_end = 0;
      }
      else {
        memcpy(&com_buffer[com_buffer_end], buf, len);
        com_buffer_end += len;
        len = 0;
      }
    }
  }
  in_pl2303_write = FALSE;
  return original_len;
}

char
usb_pl2303_getc (void)
{
  unsigned char buf[3];
  int act_len = 0;
  
  if ((act_len = usb_pl2303_read (buf, 1)) != 1) {
    DLOG ("PL2302 read failed\n");
    return '\0';
  }

  return buf[0];
}

int
usb_pl2303_read (unsigned char * buf, uint32_t len)
{
  int act_len = 0;
  int status = 0;

  if(!pl2303_initialized) return -1;
  
  status = usb_bulk_msg(&pl2303_dev, usb_rcvbulkpipe(&pl2303_dev, in_ept), buf,
                        len, &act_len, USB_DEFAULT_BULK_MSG_TIMEOUT);

  if (status  < 0) DLOG ("Bulk read failed. Error Code: 0x%X", status);

  return act_len;
}

static USB_DRIVER pl2303_driver = {
  .probe = pl2303_probe
};

bool
usb_pl2303_driver_init (void)
{
  return usb_register_driver (&pl2303_driver);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_pl2303_driver_init
};

DEF_MODULE (usb___pl2303, "USB pl2303 serial port driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
