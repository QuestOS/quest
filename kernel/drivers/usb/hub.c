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

/* USB Hub driver */
#include <smp/apic.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/hub.h>
#include <util/printf.h>
#include <kernel.h>
#include <sched/sched.h>

#define USB_HUB_CLASS 0x9

//#define DEBUG_USB_HUB
//#define DEBUG_USB_HUB_VERBOSE


#ifdef DEBUG_USB_HUB
#define DLOG(fmt,...) DLOG_PREFIX("usb-hub",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_USB_HUB_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("usb-hub",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif

#define HUB_PORT_STAT_POWER 0x0100
#define HUB_PORT_RESET 4
#define HUB_PORT_POWER 8
#define HUB_PORT_C_RESET 20
#define HUB_PORT_C_CONNECTION 16
 

#define MAX_NUM_HUBS 20
hub_info_t hub_infos[MAX_NUM_HUBS];
static uint num_hub_infos = 0;


#define HUB_HOTPLUG_STACK_SIZE 1024
u32 usb_hotplug_stack[HUB_HOTPLUG_STACK_SIZE] ALIGNED(0x1000);


static void enumerate_port(hub_info_t* hub_info, int port);

static uint16
hub_port_status (USB_DEVICE_INFO* info, uint port)
{
  sint status;
  uint8 data[4];

  /* We assume this is a full speed device, use the maximum, 64 bytes */
  status = usb_control_msg(info, usb_rcvctrlpipe(info, 0), USB_GET_STATUS, 0xA3,
                           0, port, data, 4, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
  DLOG ("GET_PORT_STATUS: status=%d port status: %.04X",
        status, *((uint16 *)data));

  return *((uint16 *)data);
}

static bool
hub_set_port_feature (USB_DEVICE_INFO* info, uint port, uint feature)
{
  sint status;
  status = usb_control_msg(info, usb_sndctrlpipe(info, 0), USB_SET_FEATURE,
                           0x23, feature, port, NULL, 0, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
  DLOG ("SET_PORT_FEATURE: status=%d", status);

  return status == 0;
}

static bool
hub_clr_port_feature (USB_DEVICE_INFO* info, uint port, uint feature)
{
  sint status;

  status = usb_control_msg(info, usb_sndctrlpipe(info, 0), USB_CLEAR_FEATURE,
                           0x23, feature, port, NULL, 0, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
  DLOGV("CLEAR_PORT_FEATURE: status=%d", status);

  return status == 0;
}


static bool
hub_clr_port_feature (USB_DEVICE_INFO* info, uint port, uint feature);

/* -- EM -- Right now this is polling would obviously better to have a
   signal mechanism where the interrupt handler for the usb completion
   signals to the thread to wake up and check for a status change */

/* This is the thread of control that is responsible for hot plugable USB devices */

void setup_hub_device_status_urb(hub_info_t* hub_info)
{
  int res;
  USB_DEVICE_INFO* dev;
  dev = hub_info->dev;
  hub_info->next_byte_to_read = 0;
  hub_info->urb = usb_alloc_urb(0,0);
  /* If we failed to allocate an urb nothing we can do but not use
     this hub */
  if(hub_info->urb != NULL) {
    uint pipe = usb_create_pipe(dev, &hub_info->status_change_endpoint);
    if(usb_pipein(pipe)) {
      dev->ep_in[usb_pipeendpoint(pipe)].desc = hub_info->status_change_endpoint;
      DLOG("dev->ep_in[usb_pipeendpoint(pipe)].desc.wMaxPacketSize = %d",
           dev->ep_in[usb_pipeendpoint(pipe)].desc.wMaxPacketSize);
    }
    else {
      dev->ep_out[usb_pipeendpoint(pipe)].desc = hub_info->status_change_endpoint;
      DLOG("dev->ep_out[usb_pipeendpoint(pipe)].desc.wMaxPacketSize = %d",
           dev->ep_out[usb_pipeendpoint(pipe)].desc.wMaxPacketSize);
    }
    memset(hub_info->status_change_buffer, 0, STATUS_CHANGE_BUFFER_SIZE);
    usb_fill_rt_int_urb(hub_info->urb, dev, pipe, &hub_info->status_change_buffer,
                        STATUS_CHANGE_BUFFER_SIZE, NULL, NULL,
                        hub_info->status_change_endpoint.bInterval, 0);
    while(1) {
      res = usb_submit_urb(hub_info->urb, 0);
      DLOG("res = %d", res);
      if(res < 0) {
        if(res == -1) {
          /* Straight out failure */
          /* -- EM -- Normally just give up for this hub but panic for now */
          DLOG("Failed to submit urb for hub");
          panic("Failed to submit urb for hub");
          break;
        }
        else {
          /* Try to submit again later */
          sched_usleep(500 * 1000);
        }
      }
      else {
        DLOG("Submitted urb for hub");
        break;
      }
    }
  }
  else {
    /* -- EM -- Remove this later */
    DLOG("Failed to allocate urb for hub");
    panic("Failed to allocate urb for hub");
  }
}


static void poll_for_port_change(hub_info_t* hub_info)
{
  static bool done = FALSE;
  struct urb* urb = hub_info->urb;
  int new_bytes = usb_rt_update_data(urb, STATUS_CHANGE_BUFFER_SIZE);
  DLOG("new_bytes = %d", new_bytes);
  
  //if(done) return;
  
  if(new_bytes > 0) {
    hub_info->bytes_available += new_bytes;

    while(hub_info->bytes_available >= STATUS_CHANGE_BUFFER_SIZE) {
      int bytes_freed;
      int i;
      uint32_t port_map = *((uint32_t*)hub_info->status_change_buffer);
      DLOGV("port_map = 0x%X", port_map);
      for(i = 1; i <= hub_info->hub_descriptor.bNbrPorts; ++i) {
        if(port_map & (1 << i)) {
          /* Change on this port */
          /* Right now we only support connecting a device not removing it */
          DLOGV("Device on port %d", i);
          if((hub_info->device_bitmap & (1 << i)) == 0) {
            /* If we have not seen the device already */
            DLOG("calling enumerate for port %d", i);
            
            enumerate_port(hub_info, i);
            done = TRUE;
            //return;
          }
          /* Clear device status bit */
          hub_clr_port_feature (hub_info->dev, i, HUB_PORT_C_CONNECTION);
        }
      }
      
      
      bytes_freed = usb_rt_free_data(urb, new_bytes);
      if(bytes_freed < 0) {
        DLOG("int_free_data returned < 0");
        panic("int_free_data returned < 0");
      }
      hub_info->bytes_available -= bytes_freed;
    }
  }
}
  
static void hub_hot_plugable_thread()
{
  /* First create a real-time urb for all the hubs we have an submit
     it */
  int i;

  for(i = 0; i < num_hub_infos; ++i) {
    setup_hub_device_status_urb(&hub_infos[i]);
  }
  
  while(1) {
    sched_usleep(10 * 1000 * 1000 );
    for(i = 0; i < num_hub_infos; ++i) {
      poll_for_port_change(&hub_infos[i]);
    }
  }
}





static uint8_t conf[3000];

static void enumerate_port(hub_info_t* hub_info, int port)
{
  int attached_dev_speed;
  uint16 port_status;
  USB_HUB_DESC* hubd = &hub_info->hub_descriptor;
  USB_DEVICE_INFO* info = hub_info->dev;
  
  hub_info->device_bitmap |= (1 << port);
  hub_set_port_feature (info, port, HUB_PORT_RESET);
  delay (10);
  hub_port_status (info, port);
  hub_clr_port_feature (info, port, HUB_PORT_C_RESET);
  port_status = hub_port_status (info, port);
  delay (2*hubd->bPwrOn2PwrGood);
  switch((port_status >> 9) & 0x3) {
  case 1:
  case 3:
    attached_dev_speed = USB_SPEED_LOW;
    break;
  case 0:
    attached_dev_speed = USB_SPEED_FULL;
    break;
  case 2:
    attached_dev_speed = USB_SPEED_HIGH;
    break;
  }
  DLOG("status = %X, masked status = %X, attached_dev_speed = %d",
       port_status, (port_status >> 9) & 0x3, attached_dev_speed);
  usb_enumerate(info->hcd, attached_dev_speed, info->address, port);
  
}

static bool
probe_hub (USB_DEVICE_INFO* info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  sint status, i;
  hub_info_t* hub_info;
  USB_EPT_DESC* endpoint;

  if (ifd->bInterfaceClass != USB_HUB_CLASS)
    return FALSE;

  if(num_hub_infos >= MAX_NUM_HUBS) {
    return FALSE;
  }
  
  /* it's a hub, set the configuration */
  if(usb_set_configuration (info, cfgd->bConfigurationValue) < 0) {
    return FALSE;
  }

  hub_info = &hub_infos[num_hub_infos++];
  
  usb_get_descriptor(info, USB_TYPE_CFG_DESC, 0, 0, cfgd->wTotalLength, (addr_t)conf);
  //print_all_descriptors(conf, cfgd->wTotalLength);
  
  endpoint = get_next_desc(USB_TYPE_EPT_DESC, conf, cfgd->wTotalLength);
  
  if(endpoint == NULL) {
    kfree(hub_info);
    return FALSE;
  }
  print_ept_desc_info(endpoint);
  
  memcpy(&hub_info->status_change_endpoint, endpoint, sizeof(USB_EPT_DESC));

  memset (&hub_info->hub_descriptor, 0, sizeof (hub_info->hub_descriptor));

  DLOG ("Probing hub @ %d", info->address);
  delay (100);
  /* We assume this is a full speed device, use the maximum, 64 bytes */

  status = usb_control_msg(info, usb_rcvctrlpipe(info, 0), USB_GET_DESCRIPTOR, 0xA0,
                           0x29 << 8, 0, &hub_info->hub_descriptor, sizeof(hub_info->hub_descriptor),
                           USB_DEFAULT_CONTROL_MSG_TIMEOUT);
  DLOG ("GET_HUB_DESCRIPTOR: status=%d len=%d nbrports=%d delay=%d",
        status, hub_info->hub_descriptor.bDescLength, hub_info->hub_descriptor.bNbrPorts,
        hub_info->hub_descriptor.bPwrOn2PwrGood);

  if (status < 0) {
    kfree(hub_info);
    return FALSE;
  }
  hub_info->dev = info;
  hub_info->device_bitmap = 0;
  for (i=1; i<=hub_info->hub_descriptor.bNbrPorts; i++) {
      DLOG("Working on port %d", i);
    /* power-on port if necessary */
    while (!((status = hub_port_status (info, i)) & HUB_PORT_STAT_POWER)) {
      hub_set_port_feature (info, i, HUB_PORT_POWER);
      delay (2*hub_info->hub_descriptor.bPwrOn2PwrGood);
    }
    if (status & 1) {
      /* potential device on port */
      enumerate_port(hub_info, i);
    }
  }

  
  info->device_priv = hub_info;

  
  
  return TRUE;
}

static USB_DRIVER hub_driver = {
  .probe = probe_hub
};

extern bool
usb_hub_driver_init (void)
{
#if 0
  task_id t= start_kernel_thread((u32)hub_hot_plugable_thread,
                       (u32) &usb_hotplug_stack[HUB_HOTPLUG_STACK_SIZE - 1],
                       "USB Hotplug");
  DLOG("hub task id = 0x%X", t);
#endif
  return usb_register_driver (&hub_driver);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_hub_driver_init
};

DEF_MODULE (usb___hub, "USB hub driver", &mod_ops, {"usb"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
