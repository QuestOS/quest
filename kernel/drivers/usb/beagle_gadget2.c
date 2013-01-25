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

#include <arch/i386.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/beagle_gadget2.h>
#include <util/printf.h>
#include <kernel.h>
#include <sched/sched.h>
#include <arch/i386-div64.h>
#include <arch/i386.h>

#define DEBUG_GADGET2
//#define DEBUG_GADGET2_VERBOSE

#ifdef DEBUG_GADGET2
#define DLOG(fmt,...) DLOG_PREFIX("gadget2",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_GADGET2_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("gadget2",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt, ...) ;
#endif

#define DLOG_INT(val) DLOG(#val " = %d", (int)(val));


static bool gadget2_probe (USB_DEVICE_INFO *device, USB_CFG_DESC *cfg,
                          USB_IF_DESC *desc);

#define MAX_GADGET2_DEVICES 5
static int current_gadget2_dev_count = 0;
static gadget2_device_info_t gadget2_devices[MAX_GADGET2_DEVICES];

static void initialise_gadget2_dev_info(gadget2_device_info_t* dev)
{
  memset(dev, 0, sizeof(*dev));
}

static int gadget2_open(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
  gadget2_sub_device_info_t* gadget2_dev = get_gadget2_sub_dev_info(device, dev_num);
  struct usb_host_endpoint *ep = usb_pipe_endpoint(device, gadget2_dev->pipe);
  int num_packets = (1024 * 8) >> (ep->desc.bInterval - 1);
  int result;
  uint64_t current_tsc;
  DLOG_INT(dev_num);
  DLOG("buf = 0x%p", buf);
  memset(buf, '-', data_len);
  
  gadget2_dev->next_to_read = gadget2_dev->data_available = 0;
  gadget2_dev->buffer_size = data_len;
  if(usb_pipetype(gadget2_dev->pipe) == PIPE_ISOCHRONOUS) {
    if(data_len <
       ((num_packets * gadget2_dev->transaction_size) +
        sizeof(struct urb) + (sizeof(usb_iso_packet_descriptor_t) * num_packets)) ) {
      DLOG("Not enough memory passed to gadget2_open, need %d",
           ((num_packets * gadget2_dev->transaction_size) +
            sizeof(struct urb) + (sizeof(usb_iso_packet_descriptor_t) * num_packets)));
      return -1;
    }
    gadget2_dev->urb = (struct urb*)buf;
    buf += (sizeof(struct urb) + (sizeof(usb_iso_packet_descriptor_t) * num_packets));
    usb_init_urb(gadget2_dev->urb);
    gadget2_dev->num_packets = num_packets;
  }
  else {
    gadget2_dev->urb = usb_alloc_urb(0, 0);
    if(gadget2_dev->urb == NULL) {
      DLOG("Failed to alloc URB");
      return -1;
    }
  }

  if(gadget2_dev->urb == NULL) {
    DLOG("Failed to allocate urb");
    return -1;
  }
  
  switch(usb_pipetype(gadget2_dev->pipe)) {
  case PIPE_ISOCHRONOUS:
    usb_fill_rt_iso_urb(gadget2_dev->urb, device, gadget2_dev->pipe, buf,
                        NULL, NULL,
                        ep->desc.bInterval, num_packets,
                        gadget2_dev->transaction_size, 0);
    break;
  case PIPE_INTERRUPT:
    if(gadget2_dev->buffer_size > (1024*400)) {
      gadget2_dev->buffer_size = 1024*400;
    }
    usb_fill_rt_int_urb(gadget2_dev->urb, device, gadget2_dev->pipe, buf,
                        gadget2_dev->buffer_size, NULL, NULL, ep->desc.bInterval, 0);
    break;
  case PIPE_BULK:
    if(gadget2_dev->buffer_size > (1024*400)) {
      gadget2_dev->buffer_size = 1024*400;
    }
    usb_fill_rt_bulk_urb(gadget2_dev->urb, device, gadget2_dev->pipe, buf,
                         gadget2_dev->buffer_size, NULL, NULL, 1, 0);
    break;
  default: // PIPE_CONTROL:
    DLOG("Gadget2 EP should never be control");
    panic("Gadget2 EP should never be control");
  }
  
  if((result = usb_submit_urb(gadget2_dev->urb, 0)) < 0) {
    //DLOG("Failed to submit rt urb result = %d", result);
    return result;
  }
  RDTSC(current_tsc);
  gadget2_dev->start_time = current_tsc;
  return 0;
}

static int gadget2_close(USB_DEVICE_INFO* device, int dev_num)
{
  return -1;
}

#define MAX_TRIP_TIMES 500
static unsigned long long trip_times[MAX_TRIP_TIMES];
static int num_trip_times = 0;


static int gadget2_read(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
#define BYTES_TO_READ (1024 * 800)
  gadget2_sub_device_info_t* gadget2_dev = get_gadget2_sub_dev_info(device, dev_num);
  struct urb* urb = gadget2_dev->urb;
  char* data_buf = urb->transfer_buffer;
  
  if(urb) {
    switch(usb_pipetype(gadget2_dev->pipe)) {
    case PIPE_INTERRUPT:
      DLOG("gadget2 interrupt right not implemented");
      panic("gadget2 interrupt right not implemented");

    case PIPE_ISOCHRONOUS:
      {
        int i, j;
        usb_iso_packet_descriptor_t* packets = urb->iso_frame_desc;
        int new_packets = usb_rt_iso_update_packets(urb, gadget2_dev->num_packets);
        uint packet_count_mask = urb->number_of_packets - 1;
        unsigned long long current_tsc;
        gadget2_dev->data_available += new_packets;
        
        int bytes_read = 0;
        //DLOG("%d:new packets = %d", dev_num, new_packets);
        if(new_packets < 0) {
          DLOG("new packets = %d", new_packets);
          panic("new packets < 0");
        }
        
        for(i = 0; i < new_packets; ++i) {
          int pac_num = (gadget2_dev->next_to_read + i) & packet_count_mask;
          if(packets[pac_num].actual_length > 0) {
            memcpy(&buf[bytes_read], &data_buf[packets[pac_num].offset],
                   packets[pac_num].actual_length);
            bytes_read += packets[pac_num].actual_length;
            if(dev_num == 0) {
              RDTSC(current_tsc);
              trip_times[num_trip_times++] = current_tsc;
              if(num_trip_times == MAX_TRIP_TIMES) {
                for(j = 0; j < MAX_TRIP_TIMES; ++j) {
                  DLOG("trip_times[%d] = 0x%llX", j, trip_times[j]);
                }
                for(j = 1; j < MAX_TRIP_TIMES; ++j) {
                  DLOG("diff[%d] = 0x%llX", j-1, trip_times[j] - trip_times[j-1]);
                }
                while(1);
              }
            }
          }
        }
        
        gadget2_dev->next_to_read = (gadget2_dev->next_to_read + new_packets) & packet_count_mask;
        if(gadget2_dev->data_available > 0) {
          int packets_freed = usb_rt_iso_free_packets(urb, gadget2_dev->data_available);
          if(packets_freed > 0) {
            gadget2_dev->data_available -= packets_freed;
          }
        }

        return bytes_read;
      }

    case PIPE_BULK:
      {
        DLOG("gadget2 read for bulk not implemented");
        panic("gadget2 read for bulk not implemented");
      }
    default: // case PIPE_CONTROL:
      {
        DLOG("Unsupported type in read");
        return -1;
      }
    }
  }
  else {
    DLOG("Urb not created need to call open first");
    return -1;
  }
}

static int gadget2_write(USB_DEVICE_INFO* device, int dev_num, char* buf,
                        int data_len)
{
  gadget2_sub_device_info_t* gadget2_dev = get_gadget2_sub_dev_info(device, dev_num);
  struct urb* urb = gadget2_dev->urb;
  int result;

  
  usb_rt_free_write_resources(gadget2_dev->urb);

  if(urb) {
    switch(usb_pipetype(gadget2_dev->pipe)) {
    case PIPE_INTERRUPT:
      {
        DLOG ("Interrupt not support in gadget2 write");
        panic("Interrupt not support in gadget2 write");
      }

    case PIPE_ISOCHRONOUS:
      {
        result = usb_rt_push_data(urb, buf, data_len, 0);
        
        return result;
      }
    case PIPE_BULK:
      {
        DLOG ("Bulk not support in gadget2 write");
        panic("Bulk not support in gadget2 write");
      }
    default: // case PIPE_CONTROL:
      {
        DLOG("Unsupported pipe type");
        return -1;
      }
    }
  }
    else {
    DLOG("Urb is not set call open first");
    return -1;
  }
}

static USB_DRIVER gadget2_driver = {
  .probe = gadget2_probe,
  .open  = gadget2_open,
  .close = gadget2_close,
  .read  = gadget2_read,
  .write = gadget2_write
};



static bool gadget2_probe (USB_DEVICE_INFO *device, USB_CFG_DESC *cfg,
                          USB_IF_DESC *desc)
{
  static char temp[30];
  USB_IF_DESC* interface;
  USB_EPT_DESC* ept;
  gadget2_device_info_t* gadget2_dev;
  int i;
  int dev_num;
  
  if(device->devd.idVendor != 0xabc5 ||
     device->devd.idProduct != 0xabc7) {
    return FALSE;
  }

  usb_set_configuration(device, cfg->bConfigurationValue);

  if(device->device_priv == NULL) {
    if(current_gadget2_dev_count == MAX_GADGET2_DEVICES) {
      DLOG("Too many gadget2 devices");
      return TRUE;
    }
    device->device_priv = &gadget2_devices[current_gadget2_dev_count++];
    initialise_gadget2_dev_info(device->device_priv);
  }

  gadget2_dev = get_gadget2_dev_info(device);
  if(gadget2_dev->initialised) {
    return TRUE;
  }

  gadget2_dev->initialised = TRUE;
  
  DLOG("Descriptor total length = %d", cfg->wTotalLength);
  DLOG("Struct size is %d", sizeof(USB_CFG_DESC));
  DLOG("Number of interfaces = %d", cfg->bNumInterfaces);
  if(usb_get_descriptor(device, USB_TYPE_CFG_DESC, 0, 0, cfg->wTotalLength,
                        (addr_t)temp)
     < 0) {
    DLOG("Failed to get full descriptor");
    return FALSE;
  }

  DLOG("Got full descriptor");
  DLOG("cfg->bLength = %d", cfg->bLength);
  DLOG("sizeof(USB_CFG_DESC) = %d", sizeof(USB_CFG_DESC));
  
  interface = (USB_IF_DESC*)(((uint8_t*)temp) + cfg->bLength);
  
  while(interface->bDescriptorType != USB_TYPE_IF_DESC) {
    interface = (USB_IF_DESC*)(((uint8_t*)interface) + interface->bLength);
  }
  
  DLOG("interface length = %d, Descriptor Type = %d, ", interface->bLength,
       interface->bDescriptorType);

  gadget2_dev->num_sub_devs = interface->bNumEndpoints;

  ept = (USB_EPT_DESC*)(((uint8_t*)interface) + interface->bLength);

  for(i = 0; i < gadget2_dev->num_sub_devs; ++i) {

    gadget2_sub_device_info_t* sub_dev = &gadget2_dev->sub_devs[i];
    
    while(ept->bDescriptorType != USB_TYPE_EPT_DESC) {
      ept = (USB_EPT_DESC*)(((uint8_t*)ept) + ept->bLength);
    }

    DLOG("Endpoint address = %d, attributes = 0x%X, "
         "maxPacketSize = %d, interval = %d",
         ept->bEndpointAddress, ept->bmAttributes, ept->wMaxPacketSize,
         ept->bInterval);

    sub_dev->pipe = usb_create_pipe(device, ept);

    DLOG("sub_dev->pipe = 0x%X", sub_dev->pipe);

    DLOG("Pipe is %s", usb_is_endpoint_in(ept) ? "Input" : "Output");

    sub_dev->transaction_size = usb_payload_size(device, ept);
    DLOG("sub_dev->transaction_size = %d", sub_dev->transaction_size);
  

    if(usb_pipein(sub_dev->pipe)) {
      device->ep_in[usb_pipeendpoint(sub_dev->pipe)].desc = *ept;
    }
    else {
      device->ep_out[usb_pipeendpoint(sub_dev->pipe)].desc = *ept;
    }

    dev_num = usb_register_device(device, &gadget2_driver);

    if(dev_num < 0) {
      DLOG("Failed to register device");
      panic("Failed to register device");
    }
    
    if(i == 0) {
      gadget2_dev->first_dev_num = dev_num;
    }

    
    ept = (USB_EPT_DESC*)(((uint8_t*)ept) + ept->bLength);
  }
  
  return TRUE;
}

bool usb_gadget2_driver_init(void)
{
  return usb_register_driver(&gadget2_driver);
}



#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_gadget2_driver_init
};

DEF_MODULE (usb___beaglegadget2, "USB gadget2 driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
