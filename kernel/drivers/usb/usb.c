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
#include <arch/i386.h>
#include <util/printf.h>
#include <kernel.h>
#include "sched/sched.h"
#include <mem/pow2.h>

//#define DEBUG_USB

#ifdef DEBUG_USB
#define DLOG(fmt,...) DLOG_PREFIX("USB", fmt, ##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define MAX_USB_HCD 10
usb_hcd_t* usb_hcd_list[MAX_USB_HCD];
uint32_t usb_hcd_list_next = 0;

#define USB_MAX_DEVICE_DRIVERS 32
static USB_DRIVER drivers[USB_MAX_DEVICE_DRIVERS];
static uint num_drivers = 0;

#define USB_CORE_MAX_DEVICES 32
static USB_DEVICE_INFO* devices[USB_CORE_MAX_DEVICES];
static uint num_devices = 0;

int usb_syscall_handler(int device_id, int operation, char* buf, int data_len)
{
  int result;
  USB_DEVICE_INFO* device = usb_get_device(device_id);

  DLOG("entering %s", __FUNCTION__);
  if(device == NULL) {
    DLOG("Unknown device id");
    return -1;
  }
  
  switch(operation) {
  case USB_USER_READ:

    result = device->driver->read(device, buf, data_len);
    break;
    
  case USB_USER_WRITE:
    
    result = device->driver->write(device, buf, data_len);
    break;

  default:
    DLOG("Unknown usb user operation %d", operation);
    return -1;
  }

  DLOG("Leaving %s result = %d", __FUNCTION__, result);
  return result;
}

bool
initialise_usb_hcd(usb_hcd_t* usb_hcd, uint32_t usb_hc_type,
                   usb_reset_root_ports_func reset_root_ports,
                   usb_post_enumeration_func post_enumeration,
                   usb_submit_urb_func usb_submit_urb,
                   usb_kill_urb_func usb_kill_urb)
{
  usb_hcd->usb_hc_type = usb_hc_type;
  usb_hcd->reset_root_ports = reset_root_ports;
  usb_hcd->post_enumeration = post_enumeration;
  usb_hcd->usb_submit_urb   = usb_submit_urb;
  usb_hcd->usb_kill_urb     = usb_kill_urb;
  usb_hcd->next_address = 1;
  return TRUE;
}

bool
add_usb_hcd(usb_hcd_t* usb_hcd)
{
  if(usb_hcd_list_next < MAX_USB_HCD) {
    usb_hcd_list[usb_hcd_list_next++] = usb_hcd;
    return TRUE;
  }
  return FALSE;
}

bool usb_register_device(USB_DEVICE_INFO* device, USB_DRIVER* driver)
{
  if(num_devices < USB_CORE_MAX_DEVICES) {
    device->driver = driver;
    devices[num_devices++] = device;
    return TRUE;
  }
  return FALSE;
}

USB_DEVICE_INFO* usb_get_device(int device_id)
{
  if(device_id < num_devices) return devices[device_id];
  return NULL;
}

usb_hcd_t*
get_usb_hcd(uint32_t index)
{
  if(index < usb_hcd_list_next) return usb_hcd_list[index];
  return NULL;
}

/*
 * -- EM -- Currently mem_flags does not do anything, just have it
 * here for now for portability with Linux
 */
int usb_submit_urb (struct urb *urb, gfp_t mem_flags)
{
  urb->active = TRUE;
  if(mem_flags != 0) {
    /*
     * -- EM -- mem_flags can only be zero for right now.  Panic if it
     * is not to avoid when we copy over drivers that start using the
     * full functionality of usb_submit_urb
     */
    DLOG("mem_flags can only be zero for right now");
    panic("mem_flags != 0 in usb_submit_urb");
  }
  return urb->dev->hcd->usb_submit_urb(urb, mem_flags);
}

/*
 * -- EM -- This function just calls the appropriate HCD kill_urb
 * function, it would be better if this some of this was abstracted
 * out into the core but for now it is all host controller specific
 */
void usb_kill_urb(struct urb *urb)
{
  if(urb->active) {
    urb->dev->hcd->usb_kill_urb(urb);
  }
}



struct urb *usb_alloc_urb(int iso_packets, gfp_t mem_flags)
{
  struct urb *urb;
  
  pow2_alloc(sizeof(struct urb) +
             iso_packets * sizeof(struct usb_iso_packet_descriptor),
                   (uint8_t**)&urb);
  if (!urb) {
    DLOG("Failed to allocate urb");
    return NULL;
  }
  usb_init_urb(urb);
  return urb;
}

static void usb_core_blocking_completion(struct urb* urb)
{
  *((bool*)urb->context) = TRUE;
}


int usb_isochronous_msg(struct usb_device *dev, unsigned int pipe,
                        void* data, int packet_size, int num_packets,
                        unsigned int* actual_lens, int* statuses,
                        int timeout)
{
  /*
   * -- EM -- It should be okay to put these on the stack because this
   * function won't complete until the callback is called
   */
  struct urb* urb;
  bool done = FALSE;

  int i;
  int ret;
  usb_complete_t complete_callback;
  struct usb_host_endpoint *ep;

  ep = usb_pipe_endpoint(dev, pipe);

  urb = usb_alloc_urb(num_packets, 0);

  memset(actual_lens, 0, sizeof(*actual_lens) * num_packets);
  memset(statuses,    0, sizeof(*statuses) * num_packets);

  if(urb == NULL) {
    return -1;
  }
  
  if(mp_enabled) {
    complete_callback = usb_core_blocking_completion;
    
    /*
     * -- EM -- So the best way to do this would have the processes
     * sleep the entire time until either woken up or a timeout
     * occurred.  Quest does not have this functionality yet (or maybe
     * it does and I just don't know) so just going to do a spin/sleep
     * combo looking at a boolean to see if the completion has
     * finished
     */
  }
  else {
    complete_callback  = NULL;
    urb->timeout = timeout;
  }
  
  usb_fill_iso_urb(urb, dev, pipe, data, complete_callback, &done,
                   ep->desc.bInterval, num_packets, packet_size);
  
  ret = usb_submit_urb(urb, 0);
  
  if(ret < 0) goto usb_isochronous_msg_out;
  
  if(mp_enabled) {
    /*
     * -- EM -- Timeout is specified in jiffies, right now assuming
     * the Linux default of 4ms
     */

    /* add one for integer rounding*/
    
    for(i = 0; (!done) && (i < (timeout / USB_MSG_SLEEP_INTERVAL) + 1 ); ++i) {
      sched_usleep(4000 * USB_MSG_SLEEP_INTERVAL);
      
    }

    ret = done ? 0 : -1;

  }

 usb_isochronous_msg_out:
  
  for(i = 0; i < num_packets; ++i) {
    actual_lens[i] = urb->iso_frame_desc[i].actual_length;
    statuses[i] = urb->iso_frame_desc[i].status;
  }
  usb_free_urb(urb);
  return ret;
}

int usb_interrupt_msg(struct usb_device *usb_dev, unsigned int pipe,
                      void *data, int len, int *actual_length,
                      int timeout)
{
  /*
   * -- EM -- It should be okay to put these on the stack because this
   * function won't complete until the callback is called
   */

  struct urb urb;
  task_id current_task;

  
  int ret;
  usb_complete_t complete_callback;
  void* context;
  struct usb_host_endpoint *ep;

  ep = usb_pipe_endpoint(usb_dev, pipe);

  if(mp_enabled) {
    complete_callback = usb_core_blocking_completion;
    current_task = str();

    /*
     * -- EM -- The context will most likely have to be more
     * complicated than this
     */
    context = &current_task;
    
    DLOG("mp_enabled usb_interrupt_msg not finished");
    panic("mp_enabled usb_interrupt_msg not finished");
  }
  else {
    complete_callback = context = NULL;
    urb.timeout = timeout;
  }

  usb_fill_int_urb(&urb, usb_dev, pipe, data, len, complete_callback, context,
                   ep->desc.bInterval);
  urb.actual_length = 0;

  ret = usb_submit_urb(&urb, 0);

  if(ret < 0) goto usb_interrupt_msg_out;
  
  if(mp_enabled) {
    /*
     * -- EM -- should sleep here after, need some way to check for
     * race condition where urb finishes before getting here
     */
  }

  /*
   * -- EM -- Set the ret val from the context passed to the callback
   */
  
 usb_interrupt_msg_out:

  *actual_length = urb.actual_length;

  return ret;

}

int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
                 void *data, int len, int *actual_length,
                 int timeout)
{
  /*
   * -- EM -- It should be okay to put these on the stack because this
   * function won't complete until the callback is called
   */
  struct urb urb;
  task_id current_task;

  
  int ret;
  usb_complete_t complete_callback;
  void* context;

  if(mp_enabled) {
    complete_callback = usb_core_blocking_completion;
    current_task = str();

    /*
     * -- EM -- The context will most likely have to be more
     * complicated than this
     */
    context = &current_task;
    
    DLOG("mp_enabled usb_bulk_msg not finished");
    panic("mp_enabled usb_bulk_msg not finished");
  }
  else {
    complete_callback = context = NULL;
    urb.timeout = timeout;
  }

  usb_fill_bulk_urb(&urb, usb_dev, pipe, data, len, complete_callback, context);
  urb.actual_length = 0;

  ret = usb_submit_urb(&urb, 0);

  if(ret < 0) goto usb_bulk_msg_out;
  
  if(mp_enabled) {
    /*
     * -- EM -- should sleep here after, need some way to check for
     * race condition where urb finishes before getting here
     */
  }

  /*
   * -- EM -- Set the ret val from the context passed to the callback
   */
  
 usb_bulk_msg_out:

  *actual_length = urb.actual_length;

  return ret;
}


int usb_control_msg(struct usb_device *dev, unsigned int pipe,
                    uint8_t request, uint8_t requesttype,
                    uint16_t value, uint16_t index,
                    void *data, uint16_t size, int timeout)
{
  /*
   * -- EM -- It should be okay to put these on the stack because this
   * function won't complete until the callback is called
   */
  struct urb urb;
  task_id current_task;

  
  USB_DEV_REQ cmd;
  int ret;
  usb_complete_t complete_callback;
  void* context;
  
  
  cmd.bmRequestType = requesttype;
  cmd.bRequest = request;
  cmd.wValue = cpu_to_le16(value);
  cmd.wIndex = cpu_to_le16(index);
  cmd.wLength = cpu_to_le16(size);
  
  if(mp_enabled) {
    complete_callback = usb_core_blocking_completion;
    current_task = str();

    /*
     * -- EM -- The context will most likely have to be more
     * complicated than this
     */
    context = &current_task;
    
    DLOG("mp_enabled usb_control_msg not finished");
    panic("mp_enabled usb_control_msg not finished");
  }
  else {
    complete_callback = context = NULL;
    urb.timeout = timeout;
  }
  
  usb_fill_control_urb(&urb, dev, pipe, (unsigned char *)&cmd, data,
                       size, complete_callback, context);

  
  ret = usb_submit_urb(&urb, 0);

  if(ret < 0) goto usb_control_msg_out;

  if(mp_enabled) {
    /*
     * -- EM -- should sleep here after, need some way to check for
     * race condition where urb finishes before getting here
     */
  }

  /*
   * -- EM -- Set the ret val from the context passed to the callback
   */
  
 usb_control_msg_out:
  
  return ret;  
}

int
usb_get_descriptor(USB_DEVICE_INFO* dev,
                   uint16_t dtype,
                   uint16_t dindex,
                   uint16_t index,
                   uint16_t length,
                   addr_t desc)
{
  return usb_control_msg(dev, usb_rcvctrlpipe(dev,0), USB_GET_DESCRIPTOR,
                         USB_DIR_IN, (dtype << 8) + dindex, index, desc, length,
                         USB_DEFAULT_CONTROL_MSG_TIMEOUT);
}


int
usb_set_address (USB_DEVICE_INFO* dev, uint8_t new_addr)
{
  sint status;


  status = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), USB_SET_ADDRESS, USB_DIR_OUT,
                           new_addr, 0, NULL, 0, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
  
  if (status >= 0) {
    dev->endpoint_toggles = 0;
  }
  return status;
  
}

int
usb_get_configuration(USB_DEVICE_INFO* dev)
{
  switch (dev->host_type)
    {
    case USB_TYPE_HC_UHCI :
      panic("UHCI broken: usb_get_configuration");
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_get_configuration(dev->address,
                                      (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_get_configuration");
      panic("EHCI Host Controller is not supported now! usb_get_configuration");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
    }
  return -1;
  
}

int
usb_set_configuration(USB_DEVICE_INFO * dev, uint8_t conf)
{
  return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), USB_SET_CONFIGURATION,
                         USB_DIR_OUT, conf, 0, NULL, 0, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
}

int
usb_get_interface(USB_DEVICE_INFO * dev, uint16_t interface)
{
  switch (dev->host_type)
    {
    case USB_TYPE_HC_UHCI :
      panic("UHCI broken: usb_get_interface");
      if ((dev->devd).bMaxPacketSize0 == 0) {
        DLOG("USB_DEVICE_INFO is probably not initialized!");
        return -1;
      } else {
        return uhci_get_interface(dev->address, interface,
                                  (dev->devd).bMaxPacketSize0);
      }

    case USB_TYPE_HC_EHCI :
      DLOG("EHCI Host Controller is not supported now! usb_get_interface");
      panic("ehci usb_get_interface not supported");
      return -1;

    case USB_TYPE_HC_OHCI :
      DLOG("OHCI Host Controller is not supported now!");
      return -1;

    default :
      DLOG("Unknown Host Controller request!");
      return -1;
    }
  return -1;
}

int
usb_set_interface(USB_DEVICE_INFO * dev, uint16_t alt, uint16_t interface)
{
  return usb_control_msg(dev, usb_sndctrlpipe(dev, 0), USB_SET_INTERFACE,
                         0x01, alt, interface, NULL, 0, USB_DEFAULT_CONTROL_MSG_TIMEOUT);
}


bool
usb_register_driver (USB_DRIVER *driver)
{
  if (num_drivers >= USB_MAX_DEVICE_DRIVERS) return FALSE;
  memcpy (&drivers[num_drivers], driver, sizeof (USB_DRIVER));
  num_drivers++;
  return TRUE;
}

static void
dlog_devd (USB_DEV_DESC *devd)
{
  DLOG ("DEVICE DESCRIPTOR len=%d type=%d bcdUSB=0x%x",
        devd->bLength, devd->bDescriptorType, devd->bcdUSB);
  DLOG ("  class=0x%x subclass=0x%x proto=0x%x maxpkt0=%d",
        devd->bDeviceClass, devd->bDeviceSubClass, devd->bDeviceProtocol,
        devd->bMaxPacketSize0);
  DLOG ("  vendor=0x%x product=0x%x bcdDevice=0x%x numcfgs=%d",
        devd->idVendor, devd->idProduct, devd->bcdDevice,
        devd->bNumConfigurations);
}

static void
dlog_info (USB_DEVICE_INFO *info)
{
  DLOG ("ADDRESS %d", info->address);
  dlog_devd (&info->devd);

#if 0
  uint8 strbuf[64];
  uint8 str[32];
#define do_str(slot,label)                                              \
  if (info->devd.slot != 0 &&                                           \
      uhci_get_string (info->address, info->devd.slot, 0,               \
                       sizeof (strbuf), strbuf,                         \
                       info->devd.bMaxPacketSize0)                      \
      == 0) {                                                           \
    memset (str, 0, sizeof (str));                                      \
    if (uhci_interp_string ((USB_STR_DESC *)strbuf, sizeof (strbuf), 0, \
                            str, sizeof (str)-1) > 0) {                 \
      DLOG ("  "label": %s", str);                                      \
    }                                                                   \
  }

  do_str (iManufacturer, "Manufacturer");
  do_str (iProduct, "Product");
#undef do_str
#endif
}

static void
find_device_driver (USB_DEVICE_INFO *info, USB_CFG_DESC *cfgd, USB_IF_DESC *ifd)
{
  int d;
  dlog_info(info);
  for (d=0; d<num_drivers; d++) {
    if (drivers[d].probe (info, cfgd, ifd)) return;
  }
  DLOG("Unknown device file %s line %d", __FILE__, __LINE__);
  //panic("Unknown Device");
}





/* figures out what device is attached as address 0 */
bool
usb_enumerate(usb_hcd_t* usb_hcd)
{
  USB_DEV_DESC devd;
  USB_CFG_DESC *cfgd;
  USB_IF_DESC *ifd;
#define TEMPSZ 256
  uint8 temp[TEMPSZ], *ptr;
  sint status, c, i, total_length=0;
  USB_DEVICE_INFO *info;
  uint curdev = usb_hcd->next_address;

  if (curdev > USB_MAX_DEVICES) {
    DLOG ("usb_enumerate: too many devices");
    return FALSE;
  }

  DLOG ("usb_enumerate: curdev=%d", curdev);

  /* clear device info */
  info = &usb_hcd->devinfo[curdev];
  memset (info, 0, sizeof (USB_DEVICE_INFO));
  info->address = 0;
  info->host_type = usb_hcd->usb_hc_type;
  info->hcd = usb_hcd;
  info->endpoint_toggles = 0;
  info->ep_in[0].desc.wMaxPacketSize = 64;
  info->ep_out[0].desc.wMaxPacketSize = 64;
  

  /* --WARN-- OK, here is the deal. The spec says you should use the
   * maximum packet size in the data phase of control transfer if the
   * data is larger than one packet. Since we do not want to support
   * low speed device for now, the bMaxPacketSize0 is always set to 64
   * bytes for full speed device. So, do not be surprised if your USB
   * mouse does not work in Quest!
   */
  info->devd.bMaxPacketSize0 = 64;

  memset (&devd, 0, sizeof (USB_DEV_DESC));

  /* get device descriptor */
  status = usb_get_descriptor (info, USB_TYPE_DEV_DESC, 0, 0,
                               sizeof (USB_DEV_DESC), &devd);
  if (status < 0) {
    DLOG("Could not get descriptor");
    goto abort;
  }
  
  dlog_devd(&devd);

  if (devd.bMaxPacketSize0 == 8) {
    /* get device descriptor */
    info->devd.bMaxPacketSize0 = 8;
    status = usb_get_descriptor(info, USB_TYPE_DEV_DESC, 0, 0,
                                sizeof(USB_DEV_DESC), &devd);
    if (status < 0)
      goto abort;
  }
  
  if (devd.bNumConfigurations == 255)
    devd.bNumConfigurations = 1; /* --!!-- hack */
  
  /* Update device info structure. Put it in USB core might be better */
  memcpy (&info->devd, &devd, sizeof (USB_DEV_DESC));
  
  /* assign an address */
  if (usb_set_address (info, curdev) < 0)
    goto abort;
  
  DLOG ("usb_enumerate: set (0x%x, 0x%x, 0x%x) to addr %d",
        devd.bDeviceClass, devd.idVendor, devd.idProduct, curdev);
  delay (2);
  
  DLOG ("usb_enumerate: num configs=%d", devd.bNumConfigurations);
  
  /* Update device info structure for new address. */
  info->address = curdev;



  for (c=0; c<devd.bNumConfigurations; c++) {
    /* get a config descriptor for size field */
    memset (temp, 0, TEMPSZ);
    status = usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0,
                                 sizeof (USB_CFG_DESC), temp);
    if (status < 0) {
      DLOG ("usb_enumerate: failed to get config descriptor for c=%d", c);
      goto abort;
    }

    cfgd = (USB_CFG_DESC *)temp;
    DLOG ("usb_enumerate: c=%d cfgd->wTotalLength=%d", c, cfgd->wTotalLength);
    total_length += cfgd->wTotalLength;
  }

  DLOG ("usb_enumerate: total_length=%d", total_length);

  /* allocate memory to hold everything */
  pow2_alloc (total_length, &info->configurations);
  if (!info->configurations) {
    DLOG ("usb_enumerate: pow2_alloc (%d) failed", total_length);
    goto abort;
  }

  /* read all cfg, if, and endpoint descriptors */
  ptr = info->configurations;
  for (c=0; c < devd.bNumConfigurations; c++) {
    /* obtain precise size info */
    memset (temp, 0, TEMPSZ);
    status = usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0,
                                 sizeof (USB_CFG_DESC), temp);
    if (status < 0) {
      DLOG ("usb_enumerate: failed to get config descriptor for c=%d", c);
      goto abort_mem;
    }
    cfgd = (USB_CFG_DESC *)temp;

    /* get cfg, if, and endp descriptors */
    status =
      usb_get_descriptor (info, USB_TYPE_CFG_DESC, c, 0, cfgd->wTotalLength, ptr);
    if (status < 0)
      goto abort_mem;

    cfgd = (USB_CFG_DESC *)ptr;
    DLOG ("usb_enumerate: cfg %d has num_if=%d", c, cfgd->bNumInterfaces);
    ptr += cfgd->wTotalLength;
  }

  /* incr this here because hub drivers may recursively invoke enumerate */
  usb_hcd->next_address++;

  /* parse cfg and if descriptors */
  ptr = info->configurations;
  for (c=0; c < devd.bNumConfigurations; c++) {
    cfgd = (USB_CFG_DESC *) ptr;
    ptr += cfgd->bLength;
    for (i=0; i < cfgd->bNumInterfaces; i++) {
      /* find the next if descriptor, skipping any class-specific stuff */
      for (ifd = (USB_IF_DESC *) ptr;
           ifd->bDescriptorType != USB_TYPE_IF_DESC;
           ifd = (USB_IF_DESC *)((uint8 *)ifd + ifd->bLength)) {
        //DLOG ("ifd=%p len=%d type=0x%x", ifd, ifd->bLength, ifd->bDescriptorType);
      }
      ptr = (uint8 *) ifd;
      DLOG ("usb_enumerate: examining (%d, %d) if_class=0x%X sub=0x%X proto=0x%X #endp=%d",
            c, i, ifd->bInterfaceClass,
            ifd->bInterfaceSubClass,
            ifd->bInterfaceProtocol,
            ifd->bNumEndpoints);

      /* find a device driver interested in this interface */
      find_device_driver (info, cfgd, ifd);

      ptr += ifd->bLength;
    }
    ptr = ((uint8 *)cfgd) + cfgd->wTotalLength;
  }

  /*
   * --??-- what happens if more than one driver matches more than one
   * config?
   */
  

  return TRUE;

 abort_mem:
  pow2_free (info->configurations);
 abort:
  return FALSE;
}

void dlog_usb_hcd(usb_hcd_t* usb_hcd)
{
  DLOG("usb_hc_type:  %d", usb_hcd->usb_hc_type);
  DLOG("next_address: %d", usb_hcd->next_address);
}

bool
usb_init (void)
{
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = usb_init
};

DEF_MODULE (usb, "USB manager", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
