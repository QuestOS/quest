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
#include <util/printf.h>
#include <kernel.h>
#include <sched/sched.h>

#define DEBUG_SIDEWINDER2

#ifdef DEBUG_SIDEWINDER2
#define DLOG(fmt,...) DLOG_PREFIX("sidewinder2-joystick",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif




typedef struct sidewinder2_joystick_dev{
  struct urb* urb;
  USB_DEVICE_INFO *dev;
  USB_EPT_DESC int_ep;
  uint buffer_size;
  char* buffer;
  char data[6];
  bool new_data;
} sidewinder2_joystick_dev_t;


static void usb_sidewinder2_callback(struct urb* urb)
{
  sidewinder2_joystick_dev_t* sidewinder2 = urb->dev->device_priv;
  int bytes_freed;
  int new_bytes = usb_rt_update_data(urb, 6);
  static int next_bytes = 0;
  static int bytes_available = 0;
  uint8 new[6];

  bytes_available += new_bytes;
  memset(new, 0, sizeof(new));
  memcpy(new, &sidewinder2->buffer[next_bytes], new_bytes);
  if(!(new[0] == 0 && new[1] == 0 && new[2] == 0 && new[3] == 0 && new[4] == 0 && new[5] == 176)) {
    sidewinder2->new_data = TRUE;
    memcpy(sidewinder2->data, new, 6);
  }
  next_bytes += new_bytes;
  if(next_bytes > sidewinder2->buffer_size) {
    next_bytes -= sidewinder2->buffer_size;
  }
  
  bytes_freed = usb_rt_free_data(urb, bytes_available);
  bytes_available = bytes_available - bytes_freed;
}

static bool init_sidewinder2_joystick_dev(sidewinder2_joystick_dev_t* dev, USB_DEVICE_INFO *usb_dev,
                                          int buffer_size)
{
  int res;
  uint pipe;

  dev->dev = usb_dev;
  dev->buffer_size = buffer_size;
  dev->buffer = kmalloc(buffer_size);

  if(dev->buffer == NULL) return FALSE;

  dev->new_data = FALSE;
  DLOG("dev->int_ep.bEndpointAddress = 0x%X", dev->int_ep.bEndpointAddress);
  dev->dev = usb_dev;
  usb_dev->ep_in[dev->int_ep.bEndpointAddress & 0xF].desc = dev->int_ep;

  dev->urb = usb_alloc_urb(0, 0);
  if(dev->urb == NULL) {
    DLOG("Failed to allocate urb");
    kfree(dev->buffer);
    return FALSE;
  }

  pipe = usb_create_pipe(dev->dev, &dev->int_ep);

  usb_fill_rt_int_urb(dev->urb, dev->dev, pipe, dev->buffer,
                      dev->buffer_size,
                      usb_sidewinder2_callback, NULL,
                      dev->int_ep.bInterval,
                      dev->int_ep.wMaxPacketSize);
  while(1) {
    res = usb_submit_urb(dev->urb, 0);
    DLOG("res = %d", res);
    if(res < 0) {
        if(res == -1) {
          /* Straight out failure */
          kfree(dev->buffer);
          return FALSE;
          break;
        }
        else {
          /* Try to submit again later */
          sched_usleep(500 * 1000);
        }
    }
    else {
      DLOG("Submitted urb for sidewinder2");
      break;
    }
  }
  
  return TRUE;
}

static USB_DRIVER device_sidewinder2_joystick;

static bool
sidewinder2_probe(USB_DEVICE_INFO *dev, USB_CFG_DESC *cfg, USB_IF_DESC *ifd)
{
  sidewinder2_joystick_dev_t* sidewinder2 = NULL;
  USB_EPT_DESC* int_ep;
  addr_t temp;
  if(dev->devd.idVendor != 0x45E ||
     dev->devd.idProduct != 0x38) return FALSE;

  if(dev->device_priv != NULL) return TRUE;

  temp = kmalloc(cfg->wTotalLength);
  if(temp == NULL) {
    DLOG("Failed to allocate temp");
    return TRUE;
  }
  
  usb_get_descriptor(dev, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength, temp);

  print_all_descriptors(temp, cfg->wTotalLength);
  DLOG("ifd->bAlternateSetting = %d", ifd->bAlternateSetting);
  int_ep = get_next_desc(USB_TYPE_EPT_DESC, temp, cfg->wTotalLength);

  if(int_ep == NULL || (usb_get_endpoint_transfer_type(int_ep) != PIPE_INTERRUPT)) {
    DLOG("Failed to find interrupt endpoint");
    kfree(temp);
    return FALSE;
  }
  
  print_ept_desc_info(int_ep);

  
  sidewinder2 = kmalloc(sizeof(sidewinder2_joystick_dev_t));
  if(sidewinder2 == NULL) {
    DLOG("Failed to allocate sidewinder2 struct");
    return FALSE;
  }
  
  sidewinder2->int_ep = *int_ep;
  
  DLOG("sidewinder2->int_ep.bEndpointAddress = 0x%X", sidewinder2->int_ep.bEndpointAddress);
  dev->device_priv = sidewinder2;
  
  kfree(temp);
  DLOG("cfg->bConfigurationValue = 0x%X", cfg->bConfigurationValue);
  if(usb_set_configuration (dev, cfg->bConfigurationValue) < 0) {
    DLOG("Failed to set configuration");
    return FALSE;
  }

  if(usb_register_device(dev, &device_sidewinder2_joystick, "joystick") < 0) {
    panic("Failed to register joystick");
    return FALSE;
  }
  
  return TRUE;
}

static int sidewinder2_close(USB_DEVICE_INFO* device, int dev_num)
{
  DLOG("sidewinder2_close is broken");
  panic("sidewinder2_close is broken");
}

static int sidewinder2_open(USB_DEVICE_INFO* device, int dev_num)
{
  return init_sidewinder2_joystick_dev(device->device_priv, device,
                                       ((sidewinder2_joystick_dev_t*)device->device_priv)->int_ep.wMaxPacketSize * 10) ? 0 : -1;
}

static int sidewinder2_read(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
  sidewinder2_joystick_dev_t* sidewinder2 = device->device_priv;
  if(!sidewinder2->new_data) return 0;
  if(data_len < 6) return 0;
  sidewinder2->new_data = FALSE;
  memcpy(buf, sidewinder2->data, 6);
  return 6;
}


#include "module/header.h"

USB_DRIVER_INIT(sidewinder2_joystick, "USB Sidewinder 2 Joystick Driver", sidewinder2_probe,
                sidewinder2_open, sidewinder2_close, sidewinder2_read, NULL)



/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
