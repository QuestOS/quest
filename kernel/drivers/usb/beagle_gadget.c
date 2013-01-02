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
#include <drivers/usb/beagle_gadget.h>
#include <util/printf.h>
#include <kernel.h>
#include <sched/sched.h>
#include <arch/i386-div64.h>
#include <arch/i386.h>

#define DEBUG_GADGET
//#define DEBUG_GADGET_VERBOSE

#ifdef DEBUG_GADGET
#define DLOG(fmt,...) DLOG_PREFIX("gadget",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_GADGET_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("gadget",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt, ...) ;
#endif

#define DLOG_INT(val) DLOG(#val " = %d", (int)(val));


static bool gadget_probe (USB_DEVICE_INFO *device, USB_CFG_DESC *cfg,
                          USB_IF_DESC *desc);

#define MAX_GADGET_DEVICES 5
static int current_gadget_dev_count = 0;
static gadget_device_info_t gadget_devices[MAX_GADGET_DEVICES];

static void initialise_gadget_dev_info(gadget_device_info_t* dev)
{
  memset(dev, 0, sizeof(*dev));
}

static int gadget_open(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
  gadget_sub_device_info_t* gadget_dev = get_gadget_sub_dev_info(device, dev_num);
  struct usb_host_endpoint *ep = usb_pipe_endpoint(device, gadget_dev->pipe);
  int num_packets = (1024 * 8) >> (ep->desc.bInterval - 1);
  int result;
  uint64_t current_tsc;
  
  memset(buf, '-', data_len);
  
  gadget_dev->next_to_read = gadget_dev->data_available = 0;
  gadget_dev->buffer_size = data_len;
  if(usb_pipetype(gadget_dev->pipe) == PIPE_ISOCHRONOUS) {
    if(data_len <
       ((num_packets * gadget_dev->transaction_size) +
        sizeof(struct urb) + (sizeof(usb_iso_packet_descriptor_t) * num_packets)) ) {
      DLOG("Not enough memory passed to gadget_open, need %d",
           ((num_packets * gadget_dev->transaction_size) +
            sizeof(struct urb) + (sizeof(usb_iso_packet_descriptor_t) * num_packets)));
      return -1;
    }
    gadget_dev->urb = (struct urb*)buf;
    buf += (sizeof(struct urb) + (sizeof(usb_iso_packet_descriptor_t) * num_packets));
    usb_init_urb(gadget_dev->urb);
    gadget_dev->num_packets = num_packets;
  }
  else {
    gadget_dev->urb = usb_alloc_urb(0, 0);
    if(gadget_dev->urb == NULL) {
      DLOG("Failed to alloc URB");
      return -1;
    }
  }

  //DLOG("dev_num = %d urb = 0x%p", dev_num, gadget_dev->urb);

  if(gadget_dev->urb == NULL) {
    DLOG("Failed to allocate urb");
    return -1;
  }
  
  switch(usb_pipetype(gadget_dev->pipe)) {
  case PIPE_ISOCHRONOUS:
    usb_fill_rt_iso_urb(gadget_dev->urb, device, gadget_dev->pipe, buf,
                        ep->desc.bInterval, num_packets,
                        gadget_dev->transaction_size);
    break;
  case PIPE_INTERRUPT:
    if(gadget_dev->buffer_size > (1024*400)) {
      gadget_dev->buffer_size = 1024*400*2;
    }
    usb_fill_rt_int_urb(gadget_dev->urb, device, gadget_dev->pipe, buf,
                        gadget_dev->buffer_size, ep->desc.bInterval);
    break;
  case PIPE_BULK:
    if(gadget_dev->buffer_size > (1024*400)) {
      gadget_dev->buffer_size = 1024*400*2;
    }
    usb_fill_rt_bulk_urb(gadget_dev->urb, device, gadget_dev->pipe, buf,
                        gadget_dev->buffer_size, 1);
    break;
  default: // PIPE_CONTROL:
    DLOG("Gadget EP should never be control");
    panic("Gadget EP should never be control");
  }
  
  if((result = usb_submit_urb(gadget_dev->urb, 0)) < 0) {
    //DLOG("Failed to submit rt urb result = %d", result);
    return result;
  }
  RDTSC(current_tsc);
  gadget_dev->start_time = current_tsc;
  return 0;
}

static int gadget_close(USB_DEVICE_INFO* device, int dev_num)
{
  return -1;
}

static int gadget_read(USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len)
{
#define BYTES_TO_READ (1024 * 800)
  static unsigned int total_bytes_read = 0;
  gadget_sub_device_info_t* gadget_dev = get_gadget_sub_dev_info(device, dev_num);
  struct urb* urb = gadget_dev->urb;
  char* data_buf;
  int bytes_freed;

  data_buf = urb->transfer_buffer;
  
  if(urb) {
    switch(usb_pipetype(gadget_dev->pipe)) {
    case PIPE_INTERRUPT:
      {
        int new_bytes = usb_rt_int_update_data(urb, BYTES_TO_READ);
        
        if(new_bytes < 0) {
          DLOG("new bytes < 0");
          panic("new bytes < 0");
        }
        if(new_bytes > 0) {
          gadget_dev->data_available += new_bytes;
          gadget_dev->total_bytes_read += new_bytes;

          gadget_dev->next_to_read += new_bytes;
          if(gadget_dev->next_to_read > gadget_dev->buffer_size) {
            gadget_dev->next_to_read -= gadget_dev->buffer_size;
          }
        }

        if(buf[0] == 'e') {
          uint64_t current_tsc;
          uint64_t diff_tsc;
          uint64_t bytes_read = gadget_dev->total_bytes_read;
          
          RDTSC(current_tsc);
          
          diff_tsc = current_tsc - gadget_dev->start_time;
          
          DLOG("\tdev: %d", dev_num);
          DLOG("\ttime diff =  %llX", diff_tsc);
          DLOG("\tbytes_read = %llX", bytes_read);
          
        }

        if(gadget_dev->data_available > 0) {
          bytes_freed = usb_rt_int_free_data(urb, gadget_dev->data_available);
          if(bytes_freed > 0) {
            gadget_dev->data_available -= bytes_freed;
          }
        }

        return new_bytes;
      }

    case PIPE_ISOCHRONOUS:
      {
        int i;
        usb_iso_packet_descriptor_t* packets = urb->iso_frame_desc;
        int new_packets = usb_rt_iso_update_packets(urb, gadget_dev->num_packets);
        uint packet_count_mask = urb->number_of_packets - 1;
        gadget_dev->data_available += new_packets;
        gadget_dev->total_packets += new_packets;
        if(new_packets < 0) {
          DLOG("new packets = %d", new_packets);
          panic("new packets < 0");
        }

        //DLOG("%d: new packets = %d", dev_num, new_packets);
        
        for(i = 0; i < new_packets; ++i) {
          int new_bytes = packets[(gadget_dev->next_to_read + i) & packet_count_mask].actual_length;
          //if(new_bytes < 100) {
          //DLOG("%d: new_bytes = %d", dev_num, new_bytes);
          //}
          gadget_dev->total_bytes_read += new_bytes;
            
          
        }


        if(buf[0] == 'e') {
          uint64_t current_tsc;
          uint64_t diff_tsc;
          uint64_t bytes_read = gadget_dev->total_bytes_read;

          RDTSC(current_tsc);
          
          diff_tsc = current_tsc - gadget_dev->start_time;
          //DLOG("current %llu", current_tsc);
          //DLOG("start   %llu", gadget_dev->start_time);
          DLOG("\tdev: %d", dev_num);
          DLOG("\ttime diff =  %llX", diff_tsc);
          DLOG("\tbytes_read = %llX", bytes_read);

        }
        
        //DLOG("gadget_dev->total_packets = %llu", gadget_dev->total_packets);
        //DLOG("gadget_dev->total_bytes_read = %llu", gadget_dev->total_bytes_read);
        //DLOG("average bytes per packet = %llu", div64_64(gadget_dev->total_bytes_read, gadget_dev->total_packets));

        gadget_dev->next_to_read = (gadget_dev->next_to_read + new_packets) & packet_count_mask;
        if(gadget_dev->data_available > 0) {
          int packets_freed = usb_rt_iso_free_packets(urb, gadget_dev->data_available);
          if(packets_freed > 0) {
            gadget_dev->data_available -= packets_freed;
          }
        }

        return 0;
      }

    case PIPE_BULK:
      {
        int new_bytes;
        new_bytes = usb_rt_bulk_update_data(urb, BYTES_TO_READ);
        
        if(new_bytes < 0) {
          DLOG("new bytes < 0");
          panic("new bytes < 0");
        }
        if(new_bytes > 0) {
          gadget_dev->data_available += new_bytes;
          gadget_dev->total_bytes_read += new_bytes;
          gadget_dev->next_to_read += new_bytes;
          if(gadget_dev->next_to_read > gadget_dev->buffer_size) {
            gadget_dev->next_to_read -= gadget_dev->buffer_size;
          }
        }

        if(buf[0] == 'e') {
          uint64_t current_tsc;
          uint64_t diff_tsc;
          uint64_t bytes_read = gadget_dev->total_bytes_read;

          RDTSC(current_tsc);
          
          diff_tsc = current_tsc - gadget_dev->start_time;
          
          DLOG("\tdev: %d", dev_num);
          DLOG("\ttime diff =  %llX", diff_tsc);
          DLOG("\tbytes_read = %llX", bytes_read);

        }
        
        if(gadget_dev->data_available > 0) {
          bytes_freed = usb_rt_bulk_free_data(urb, gadget_dev->data_available);
          if(bytes_freed > 0) {
            gadget_dev->data_available -= bytes_freed;
          }
        }

        return new_bytes;
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

#define PUSH_LOTS_DATA
#define SET_DATA

static int gadget_write(USB_DEVICE_INFO* device, int dev_num, char* buf,
                        int data_len)
{
  static int rotating_counter = 0;
  static int write_call_counter = 0;
  uint64_t time;
  gadget_sub_device_info_t* gadget_dev = get_gadget_sub_dev_info(device, dev_num);
  struct urb* urb = gadget_dev->urb;
  int result;
#ifdef SET_DATA
  int memset_counter = 0;
#endif
  DLOG_INT(write_call_counter);
  write_call_counter++;

#ifdef SET_DATA
  while(memset_counter < data_len) {
    int data_this_time = (data_len - memset_counter) > 512 ?
      512 : (data_len - memset_counter);
    
    memset(&buf[memset_counter], 'a' + rotating_counter, data_this_time);
    memset_counter += data_this_time;
    if(++rotating_counter == 10) {
      rotating_counter = 0;
    }
  }
  
#endif
  if(urb) {
    switch(usb_pipetype(gadget_dev->pipe)) {
    case PIPE_INTERRUPT:
      {
#if 1
#ifdef PUSH_LOTS_DATA
        result = usb_rt_int_push_data(urb, buf, data_len);
#else
        result = usb_rt_int_push_data(urb, buf, gadget_dev->transaction_size);
#endif
#else
        result = usb_rt_int_push_data(urb, "1", 1);
        DLOG("result = %d", result);
        while(1);
#endif
        if(result > 0) {
          gadget_dev->total_bytes_written += result;
          RDTSC(time);
          DLOG("%d: total_bytes_written = %u, 0x%llX", dev_num,
               gadget_dev->total_bytes_written, time - gadget_dev->start_time);
        }
        if(result < 0) {
          DLOG("result = %d", result);
          panic("result < 0");
        }
        
        DLOG("result = %d", result);
        return result;
      }

    case PIPE_ISOCHRONOUS:
      {
#if 1
#ifdef PUSH_LOTS_DATA
        result = usb_rt_iso_push_data(urb, buf, data_len);
#else
        result = usb_rt_iso_push_data(urb, buf, gadget_dev->transaction_size);
#endif
#else
        result = usb_rt_iso_push_data(urb, "3", 1);
        DLOG("result = %d", result);
#endif
        
        return result;
      }
    case PIPE_BULK:
      {
#if 1
#ifdef PUSH_LOTS_DATA
        result = usb_rt_bulk_push_data(urb, buf, data_len);
#else
        result = usb_rt_bulk_push_data(urb, buf, gadget_dev->transaction_size);
#endif
#else
        result = usb_rt_bulk_push_data(urb, "3", 1);
        
#endif
        DLOG("result = %d", result);
        return result;
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

static USB_DRIVER gadget_driver = {
  .probe = gadget_probe,
  .open  = gadget_open,
  .close = gadget_close,
  .read  = gadget_read,
  .write = gadget_write
};



static bool gadget_probe (USB_DEVICE_INFO *device, USB_CFG_DESC *cfg,
                          USB_IF_DESC *desc)
{
  static char temp[30];
  USB_IF_DESC* interface;
  USB_EPT_DESC* ept;
  gadget_device_info_t* gadget_dev;
  int i;
  int dev_num;
  DLOG("gadget_probe called");
  if(device->devd.idVendor != 0xabc4 ||
     device->devd.idProduct != 0xabc7) {
    return FALSE;
  }

  usb_set_configuration(device, cfg->bConfigurationValue);

  if(device->device_priv == NULL) {
    if(current_gadget_dev_count == MAX_GADGET_DEVICES) {
      DLOG("Too many gadget devices");
      return TRUE;
    }
    device->device_priv = &gadget_devices[current_gadget_dev_count++];
    initialise_gadget_dev_info(device->device_priv);
  }

  gadget_dev = get_gadget_dev_info(device);
  if(gadget_dev->initialised) {
    return TRUE;
  }

  gadget_dev->initialised = TRUE;
  
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

  gadget_dev->num_sub_devs = interface->bNumEndpoints;

  ept = (USB_EPT_DESC*)(((uint8_t*)interface) + interface->bLength);

  for(i = 0; i < gadget_dev->num_sub_devs; ++i) {

    gadget_sub_device_info_t* sub_dev = &gadget_dev->sub_devs[i];
    
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

    dev_num = usb_register_device(device, &gadget_driver);

    if(dev_num < 0) {
      DLOG("Failed to register device");
      panic("Failed to register device");
    }

    DLOG("Device number = %d", dev_num);
    
    if(i == 0) {
      gadget_dev->first_dev_num = dev_num;
    }

    
    ept = (USB_EPT_DESC*)(((uint8_t*)ept) + ept->bLength);
  }
  
  return TRUE;
}

bool usb_gadget_driver_init(void)
{
  return usb_register_driver(&gadget_driver);
}



#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_gadget_driver_init
};

DEF_MODULE (usb___beaglegadget, "USB gadget driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
