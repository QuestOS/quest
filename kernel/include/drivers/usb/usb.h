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

#ifndef _USB_H_
#define _USB_H_

#include <types.h>
#include <drivers/usb/linux_usb.h>
#include <stddef.h>


#define pci_config_rd8(bus, slot, func, reg) \
  pci_read_byte (pci_addr (bus, slot, func, reg))
#define pci_config_wr8(bus, slot, func, reg, val) \
  pci_write_byte (pci_addr (bus, slot, func, reg), val)

#define pci_config_rd16(bus, slot, func, reg) \
  pci_read_word (pci_addr (bus, slot, func, reg))
#define pci_config_wr16(bus, slot, func, reg, val) \
  pci_write_word (pci_addr (bus, slot, func, reg), val)

#define pci_config_rd32(bus, slot, func, reg) \
  pci_read_dword (pci_addr (bus, slot, func, reg))
#define pci_config_wr32(bus, slot, func, reg, val) \
  pci_write_dword (pci_addr (bus, slot, func, reg), val)

#define delay(x) sched_usleep (x*1000)

/* -- EM -- Both EHCI and net2280 use this so I am putting it here for
      now */
bool
handshake(uint32_t *ptr, uint32_t mask, uint32_t done, uint32_t usec);

/* Async takes  */

#ifdef USB_REALTIME_ASYNC
#define USEC_PER_MICRO_FRAME_FOR_ASYNC 121
#endif

#define USEC_PER_MICRO_FRAME_FOR_PERIODIC 100


#define USB_MAX_LEN       0x3FE
#define USB_NULL_PACKET   0x7FF

#define USB_GET_STATUS            0x00
#define USB_REQ_GET_STATUS        0x00
#define USB_CLEAR_FEATURE         0x01
#define USB_REQ_CLEAR_FEATURE     0x01
#define USB_SET_FEATURE           0x03
#define USB_REQ_SET_FEATURE       0x03
#define USB_SET_ADDRESS           0x05
#define USB_REQ_SET_ADDRESS       0x05
#define USB_GET_DESCRIPTOR        0x06
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_SET_DESCRIPTOR        0x07
#define USB_REQ_SET_DESCRIPTOR    0x07
#define USB_GET_CONFIGURATION     0x08
#define USB_REQ_GET_CONFIGURATION 0x08
#define USB_SET_CONFIGURATION     0x09
#define USB_REQ_SET_CONFIGURATION 0x09
#define USB_GET_INTERFACE         0x0A
#define USB_REQ_GET_INTERFACE     0x0A
#define USB_SET_INTERFACE         0x0B
#define USB_REQ_SET_INTERFACE     0x0B
#define USB_SYNCH_FRAME           0x0C
#define USB_REQ_SYNCH_FRAME       0x0C
#define USB_SET_SEL               0x30
#define USB_REQ_SET_SEL           0x30
#define USB_SET_ISOCH_DELAY       0x31
#define USB_REQ_SET_ISOCH_DELAY   0x31

#define USB_TYPE_DEV_DESC         0x01
#define USB_DT_DEVICE             0x01
#define USB_TYPE_CFG_DESC         0x02
#define USB_DT_CONFIG             0x02
#define USB_TYPE_STR_DESC         0x03
#define USB_DT_STRING             0x03
#define USB_TYPE_IF_DESC          0x04
#define USB_DT_INTERFACE          0x04
#define USB_TYPE_EPT_DESC         0x05
#define USB_DT_ENDPOINT           0x05
#define USB_TYPE_QUA_DESC         0x06
#define USB_DT_DEVICE_QUALIFIER   0x06
#define USB_TYPE_SPD_CFG_DESC     0x07
#define USB_DT_OTHER_SPEED_CONFIG 0x07
#define USB_TYPE_IF_PWR_DESC      0x08
#define USB_DT_INTERFACE_POWER    0x08
#define USB_TYPE_BOS_DESC         0x0f
#define USB_DT_BOS                0x0f

#define USB_TYPE_HC_UHCI    0x00
#define USB_TYPE_HC_EHCI    0x01
#define USB_TYPE_HC_OHCI    0x02


#define PIPE_ISOCHRONOUS  0
#define PIPE_INTERRUPT    1
#define PIPE_CONTROL      2
#define PIPE_BULK         3

#define USB_SPEED_UNKNOWN  0
#define USB_SPEED_LOW      1
#define USB_SPEED_FULL     2
#define USB_SPEED_HIGH     3
#define USB_SPEED_WIRELESS 4
#define USB_SPEED_SUPER    5

/*
 * Device and/or Interface Class codes
 * as found in bDeviceClass or bInterfaceClass
 * and defined by www.usb.org documents
 */
#define USB_CLASS_PER_INTERFACE         0       /* for DeviceClass */
#define USB_CLASS_AUDIO                 1
#define USB_CLASS_COMM                  2
#define USB_CLASS_HID                   3
#define USB_CLASS_PHYSICAL              5
#define USB_CLASS_STILL_IMAGE           6
#define USB_CLASS_PRINTER               7
#define USB_CLASS_MASS_STORAGE          8
#define USB_CLASS_HUB                   9
#define USB_CLASS_CDC_DATA              0x0a
#define USB_CLASS_CSCID                 0x0b    /* chip+ smart card */
#define USB_CLASS_CONTENT_SEC           0x0d    /* content security */
#define USB_CLASS_VIDEO                 0x0e
#define USB_CLASS_WIRELESS_CONTROLLER   0xe0
#define USB_CLASS_MISC                  0xef
#define USB_CLASS_APP_SPEC              0xfe
#define USB_CLASS_VENDOR_SPEC           0xff

#define USB_SUBCLASS_VENDOR_SPEC        0xff

/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_MASK                   (0x03 << 5)
#define USB_TYPE_STANDARD               (0x00 << 5)
#define USB_TYPE_CLASS                  (0x01 << 5)
#define USB_TYPE_VENDOR                 (0x02 << 5)
#define USB_TYPE_RESERVED               (0x03 << 5)

#define USB_DIR_IN  0x80
#define USB_DIR_OUT 0


#define USB_INTRF_FUNC_SUSPEND  0       /* function suspend */
#define USB_ENDPOINT_HALT       0       /* IN/OUT will STALL */

/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK        0x0f    /* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK           0x80


/*
 * USB recipients, the third of three bRequestType fields
 */
#define USB_RECIP_MASK                  0x1f
#define USB_RECIP_DEVICE                0x00
#define USB_RECIP_INTERFACE             0x01
#define USB_RECIP_ENDPOINT              0x02
#define USB_RECIP_OTHER                 0x03


#define USB_USER_READ  0
#define USB_USER_WRITE 1
#define USB_USER_OPEN  2
#define USB_USER_CLOSE 3

#define USB_JIFFIES_TO_USEC (4000)

// timeout is specified in jiffies 1 jiffy = 4ms
#define USB_DEFAULT_CONTROL_MSG_TIMEOUT     1250
#define USB_DEFAULT_BULK_MSG_TIMEOUT        1250
#define USB_DEFAULT_INTERRUPT_MSG_TIMEOUT   1250
#define USB_DEFAULT_ISOCHRONOUS_MSG_TIMEOUT 2250

#define USB_ENDPOINT_XFERTYPE_MASK      0x03    /* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL       0
#define USB_ENDPOINT_XFER_ISOC          1
#define USB_ENDPOINT_XFER_BULK          2
#define USB_ENDPOINT_XFER_INT           3
#define USB_ENDPOINT_MAX_ADJUSTABLE     0x80


#define USB_MSG_SLEEP_INTERVAL 25 // sleep for (4 * USB_MSG_SLEEP_INTERVAL) ms

//struct _usb_hcd_t;

typedef struct _usb_hcd_t usb_hcd_t;

struct usb_string_descriptor {
  u8  bLength;
  u8  bDescriptorType;
  
  u16 wData[1];                /* UTF-16LE encoded */
} PACKED;

struct usb_descriptor_header {
  u8 bLength;
  u8 bDescriptorType;
} PACKED;

/*
 * USB_DEV_REQ : USB Device Request
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 1.1, Page 183
 */
struct usb_dev_req
{
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} PACKED;

#define usb_ctrlrequest usb_dev_req

#define IS_INPUT_USB_DEV_REQ(dev_req_addr) ( (*((uint8_t*)dev_req_addr) ) & USB_DIR_IN )

typedef struct usb_dev_req USB_DEV_REQ;

/*
 * USB_DEV_DESC : Standard Device Descriptor
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 1.1, Page 197
 */
struct usb_dev_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
} PACKED;

#define usb_device_descriptor usb_dev_desc

typedef struct usb_dev_desc USB_DEV_DESC;

/*
 * USB_CFG_DESC : Standard Configuration Descriptor
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 1.1, Page 199
 */
struct usb_cfg_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
} PACKED;


typedef struct usb_cfg_desc USB_CFG_DESC;


#define usb_config_descriptor usb_cfg_desc

#define USB_DT_CONFIG_SIZE              9

/* from config descriptor bmAttributes */
#define USB_CONFIG_ATT_ONE              (1 << 7)        /* must be set */
#define USB_CONFIG_ATT_SELFPOWER        (1 << 6)        /* self powered */
#define USB_CONFIG_ATT_WAKEUP           (1 << 5)        /* can wakeup */
#define USB_CONFIG_ATT_BATTERY          (1 << 4)        /* battery powered */


/* USB 2.0 Extension descriptor */
#define USB_CAP_TYPE_EXT                2

struct usb_ext_cap_descriptor {         /* Link Power Management */
  u8  bLength;
  u8  bDescriptorType;
  u8  bDevCapabilityType;
  u32 bmAttributes;
#define USB_LPM_SUPPORT                 (1 << 1)        /* supports LPM */
#define USB_BESL_SUPPORT                (1 << 2)        /* supports BESL */
#define USB_BESL_BASELINE_VALID         (1 << 3)        /* Baseline BESL valid*/
#define USB_BESL_DEEP_VALID             (1 << 4)        /* Deep BESL valid */
#define USB_GET_BESL_BASELINE(p)        (((p) & (0xf << 8)) >> 8)
#define USB_GET_BESL_DEEP(p)            (((p) & (0xf << 12)) >> 12)
} PACKED;

#define USB_DT_USB_EXT_CAP_SIZE 7


/*
 * USB_SPD_CFG_DESC : Other Speed Configuration Descriptor
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 2.0, Page 267
 */
struct usb_spd_cfg_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
} PACKED;

typedef struct usb_spd_cfg_desc USB_SPD_CFG_DESC;

/*
 * USB_IF_DESC : Standard Interface Descriptor
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 1.1, Page 202
 */
struct usb_if_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
} PACKED;

typedef struct usb_if_desc USB_IF_DESC;

#define usb_interface_descriptor usb_if_desc

/*
 * USB_EPT_DESC : Standard Endpoint Descriptor
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 1.1, Page 203
 */
struct usb_ept_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
} PACKED;

typedef struct usb_ept_desc USB_EPT_DESC;

#define usb_endpoint_descriptor usb_ept_desc


#define USB_DT_ENDPOINT_SIZE            7

/* 
 * USB_STR_DESC: Standard String Descriptor
 *
 * Reference :
 *     Universal Serial Bus Specification
 *     Revision 1.1, Page 205
 */

struct usb_str_desc
{
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bString[];
} PACKED;

typedef struct usb_str_desc USB_STR_DESC;

/* USB_DT_DEVICE_QUALIFIER: Device Qualifier descriptor */
struct usb_qualifier_descriptor {
  u8  bLength;
  u8  bDescriptorType;
  
  u16 bcdUSB;
  u8  bDeviceClass;
  u8  bDeviceSubClass;
  u8  bDeviceProtocol;
  u8  bMaxPacketSize0;
  u8  bNumConfigurations;
  u8  bRESERVED;
} PAKCKED;



/* ************************************************** */

/*
 * ********************************************************************
 * The following is copied from Linux but needs to be put here for
 * compiler reasons
 * ********************************************************************
 */

struct usb_host_endpoint {
  USB_EPT_DESC desc;
  
  /*
   * -- EM -- Removing the below data structures because I do not
   * need them and do not know what some are for
   */
  
  //struct list_head                urb_list;
  //void                            *hcpriv;
  //struct ep_device                *ep_dev;        /* For sysfs info */
  //struct usb_host_ss_ep_comp      *ss_ep_comp;    /* For SS devices */
  
  //unsigned char *extra;   /* Extra descriptors */
  //int extralen;
  //int enabled;
};

typedef struct usb_host_endpoint USB_HOST_ENDPOINT;



/*
 * ********************************************************************
 * End of functions copied from Linux
 * ********************************************************************
 */

struct _USB_DRIVER;

typedef struct usb_device
{
  uint8 address;
  USB_DEV_DESC devd;
  uint8 host_type;
  uint8 speed;
  usb_hcd_t* hcd;
  uint8 *configurations;
  
  /* The device number of the hub the device is connected to is 0 is
     connected to the root hub */
  uint hub_addr;

  /* The port number of the hub the device is connected to */
  uint port_num;

  struct _USB_DRIVER* driver;
  void* device_priv; /* A place for a device to put its own private data */
  
  /*
   *  device can have at most 31 endpoints, 16 endpoint numbers and
   *  each number can be shared by 2 endpoints one IN and one OUT
   *  except endpoint 0 which is a bidirectional control endpoint
   */
  uint32 endpoint_toggles;
  
  USB_HOST_ENDPOINT ep_in[16];
  USB_HOST_ENDPOINT ep_out[16];
} USB_DEVICE_INFO;

/*
 * ********************************************************************
 * The following is copied from Linux but needs to be put here for
 * compiler reasons
 * ********************************************************************
 */


static inline uint32_t
__create_pipe(struct usb_device *dev, unsigned int endpoint)
{
  return (dev->address << 8) | (endpoint << 15);
}

/* Create various pipes... */
#define usb_sndctrlpipe(dev, endpoint)  \
        ((PIPE_CONTROL << 30) | __create_pipe(dev, endpoint))
#define usb_rcvctrlpipe(dev, endpoint)  \
        ((PIPE_CONTROL << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)
#define usb_sndisocpipe(dev, endpoint)  \
        ((PIPE_ISOCHRONOUS << 30) | __create_pipe(dev, endpoint))
#define usb_rcvisocpipe(dev, endpoint)  \
        ((PIPE_ISOCHRONOUS << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)
#define usb_sndbulkpipe(dev, endpoint)  \
        ((PIPE_BULK << 30) | __create_pipe(dev, endpoint))
#define usb_rcvbulkpipe(dev, endpoint)  \
        ((PIPE_BULK << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)
#define usb_sndintpipe(dev, endpoint)   \
        ((PIPE_INTERRUPT << 30) | __create_pipe(dev, endpoint))
#define usb_rcvintpipe(dev, endpoint)   \
        ((PIPE_INTERRUPT << 30) | __create_pipe(dev, endpoint) | USB_DIR_IN)


static inline uint16_t
usb_maxpacket(struct usb_device *udev, int pipe)
{
  unsigned epnum = usb_pipeendpoint(pipe);

  return usb_pipein(pipe) ? udev->ep_in[epnum].desc.wMaxPacketSize :
    udev->ep_out[epnum].desc.wMaxPacketSize;
}

/*
 * ********************************************************************
 * End of functions copied from Linux
 * ********************************************************************
 */

void print_all_descriptors(void* descriptor_start, uint total_length);
void* get_next_desc(uint desc_type, void* descriptor_start, uint remaining_length);
void print_ept_desc_info(USB_EPT_DESC* ept_desc);


#define IS_USB_ENDPOINT_TOGGLED(dev_info, endpoint, is_input)           \
  (!!(((dev_info)->endpoint_toggles) & (1 << (endpoint + ( (!!(is_input)) << 4)))))


#define SET_USB_ENDPOINT_TOGGLE(dev_info, endpoint, is_input, val)      \
  do {                                                                  \
    if(val) {                                                           \
      ((dev_info)->endpoint_toggles) |=                                 \
        (1 << (endpoint + ( (!!(is_input)) << 4)));                     \
    }                                                                   \
    else {                                                              \
      ((dev_info)->endpoint_toggles) &=                                 \
        ~(1 << (endpoint + ( (!!(is_input)) << 4)));                    \
    }                                                                   \
  }                                                                     \
  while(0)


typedef struct _USB_DRIVER
{
  bool (*probe) (USB_DEVICE_INFO *, USB_CFG_DESC *, USB_IF_DESC *);
  int (*open) (USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len);
  int (*close) (USB_DEVICE_INFO* device, int dev_num);
  int (*write) (USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len);
  int (*read) (USB_DEVICE_INFO* device, int dev_num, char* buf, int data_len);
} USB_DRIVER;

int usb_register_device(USB_DEVICE_INFO* device, USB_DRIVER* driver);

bool usb_register_driver (USB_DRIVER *driver);

USB_DEVICE_INFO* usb_get_device(int device_id);

int usb_syscall_handler(uint32_t device_id, uint32_t operation, char* buf,
                        uint32_t data_len);

/* Generic USB operations */

int usb_get_descriptor(USB_DEVICE_INFO* dev, uint16_t dtype,
                              uint16_t dindex, uint16_t index,
                              uint16_t length, addr_t desc);



int usb_set_address(USB_DEVICE_INFO * dev, uint8_t new_addr);

int usb_get_configuration(USB_DEVICE_INFO* dev);

int usb_set_configuration(USB_DEVICE_INFO* dev, uint8_t conf);

int usb_get_interface(USB_DEVICE_INFO* dev, uint16_t interface);

int usb_set_interface(USB_DEVICE_INFO* dev, uint16_t alt,
                             uint16_t interface);

int usb_payload_size(USB_DEVICE_INFO* dev,
                     USB_EPT_DESC* endpoint);

int usb_rt_iso_update_packets(struct urb* urb, int max_packets);

int usb_rt_iso_free_packets(struct urb* urb, int number_of_packets);

int usb_rt_data_lost(struct urb* urb);

int usb_rt_free_data(struct urb* urb, int count);

int usb_rt_update_data(struct urb* urb, int max_count);

int usb_rt_push_data(struct urb* urb, char* data, int count, uint interrupt_rate);

/*
 * ********************************************************************
 * The following is copied from Linux but needs to be put here for
 * compiler reasons or for convenience
 * ********************************************************************
 */


int usb_submit_urb (struct urb *urb, gfp_t mem_flags);

void usb_kill_urb(struct urb *urb);


/*
 * -- EM -- The implementation of the four usb_*_msg functions is
 * annoyingly similar with a few minor differences. It would be best
 * to find a way to abstract out these minor differences and have 1
 * main function/macro that is the major guts of the implementation so
 * future changes do not have to be copied across all functions.
 */

int usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
                 void *data, int len, int *actual_length,
                 int timeout);

/*
 * Linux provides this function but in our system it breaks the idea
 * of a RT-USB system. It is however useful for testing device drivers
 * and therefore it exists for that purpose
 */
int usb_interrupt_msg(struct usb_device *usb_dev, unsigned int pipe,
                      void *data, int len, int *actual_length,
                      int timeout);


int usb_control_msg(struct usb_device *dev, unsigned int pipe,
                    uint8_t request, uint8_t requesttype,
                    uint16_t value, uint16_t index,
                    void *data, uint16_t size, int timeout);


/*
 * This function should never really be used since it breaks all the
 * principles of isochronous transfers, especially in our RT-USB
 * system.  It is however useful for testing device drivers and
 * therefore it exists for that purpose
 */
int usb_isochronous_msg(struct usb_device *dev, unsigned int pipe,
                        void* data, int packet_size, int num_packets,
                        unsigned int* actual_lens, int* statuses,
                        int timeout);

static inline void
usb_fill_bulk_urb(struct urb *urb,
                  struct usb_device *dev,
                  unsigned int pipe,
                  void *transfer_buffer,
                  int buffer_length,
                  usb_complete_t complete_fn,
                  void *context)
{
  urb->dev = dev;
  urb->pipe = pipe;
  urb->transfer_buffer = transfer_buffer;
  urb->transfer_buffer_length = buffer_length;
  urb->complete = complete_fn;
  urb->context = context;
}


static inline void
usb_fill_control_urb(struct urb *urb,
                     struct usb_device *dev,
                     unsigned int pipe,
                     unsigned char *setup_packet,
                     void *transfer_buffer,
                     int buffer_length,
                     usb_complete_t complete_fn,
                     void *context)
{
  urb->dev = dev;
  urb->pipe = pipe;
  urb->setup_packet = setup_packet;
  urb->transfer_buffer = transfer_buffer;
  urb->transfer_buffer_length = buffer_length;
  urb->complete = complete_fn;
  urb->context = context;
}

static inline void
usb_fill_int_urb(struct urb *urb,
                 struct usb_device *dev,
                 unsigned int pipe,
                 void *transfer_buffer,
                 int buffer_length,
                 usb_complete_t complete_fn,
                 void *context,
                 int interval)
{
  urb->dev = dev;
  urb->pipe = pipe;
  urb->transfer_buffer = transfer_buffer;
  urb->transfer_buffer_length = buffer_length;
  urb->complete = complete_fn;
  urb->context = context;
  if (dev->speed == USB_SPEED_HIGH || dev->speed == USB_SPEED_SUPER) {
    urb->interval = 1 << (interval - 1);
  }
  else {
    urb->interval = interval;
  }
  urb->start_frame = -1;
}


static inline struct usb_host_endpoint *
usb_pipe_endpoint(struct usb_device *dev, unsigned int pipe)
{
  return usb_pipein(pipe) ? &(dev->ep_in[usb_pipeendpoint(pipe)]) :
    &(dev->ep_out[usb_pipeendpoint(pipe)]);
}

/*
 * ********************************************************************
 * End of functions copied from Linux
 * ********************************************************************
 */

/*
 * We are nicer than Linux we provide a usb_fill_iso_urb function ;-)
 */

static inline void usb_fill_iso_urb(struct urb* urb,
                                    struct usb_device *dev,
                                    unsigned int pipe,
                                    void *transfer_buffer,
                                    usb_complete_t complete_fn,
                                    void *context,
                                    int interval,
                                    int num_packets,
                                    int packet_len)
{
  int i;
   
  urb->dev = dev;
  urb->pipe = pipe;
  urb->transfer_buffer = transfer_buffer;
  urb->complete = complete_fn;
  urb->context = context;
  urb->number_of_packets = num_packets;
  if (dev->speed == USB_SPEED_HIGH ||
      dev->speed == USB_SPEED_SUPER ||
      USB_SPEED_FULL) {
    urb->interval = 1 << (interval - 1);
  }
  else {
    urb->interval = interval;
  }

  for(i = 0; i < num_packets; ++i) {
    urb->iso_frame_desc[i].length = packet_len;
    urb->iso_frame_desc[i].offset = i * packet_len;
    urb->iso_frame_desc[i].data_available = FALSE;
  }
}

static inline void
usb_fill_rt_iso_urb(struct urb* urb,
                    struct usb_device *dev,
                    unsigned int pipe,
                    void *transfer_buffer,
                    int interval,
                    int num_packets,
                    int packet_len)
{
  usb_fill_iso_urb(urb, dev, pipe, transfer_buffer, NULL, NULL,
                   interval, num_packets, packet_len);
  urb->realtime = TRUE;
  
}

static inline void
usb_fill_rt_int_urb(struct urb *urb,
                 struct usb_device *dev,
                 unsigned int pipe,
                 void *transfer_buffer,
                 int buffer_length,
                 int interval)
{
  usb_fill_int_urb(urb, dev, pipe, transfer_buffer, buffer_length,
                   NULL, NULL, interval);
  urb->realtime = TRUE;
}

static inline void
usb_fill_rt_bulk_urb(struct urb *urb,
                 struct usb_device *dev,
                 unsigned int pipe,
                 void *transfer_buffer,
                 int buffer_length,
                 int interval)
{
  usb_fill_rt_int_urb(urb, dev, pipe, transfer_buffer, buffer_length, interval);
}

static inline uint
usb_get_endpoint_transfer_type(USB_EPT_DESC* ept)
{
  switch(ept->bmAttributes & 0x3) {
  case 0:
    return PIPE_CONTROL;
  case 1:
    return PIPE_ISOCHRONOUS;
  case 2:
    return PIPE_BULK;
  default: // case 3:
    return PIPE_INTERRUPT;
  }
}

static inline uint
usb_is_endpoint_in(USB_EPT_DESC* ept)
{
  return (ept->bEndpointAddress & USB_DIR_IN);
}

static inline uint usb_create_pipe(USB_DEVICE_INFO *device, USB_EPT_DESC* ept)
{
  switch(usb_get_endpoint_transfer_type(ept)) {
  case PIPE_ISOCHRONOUS:
    if(usb_is_endpoint_in(ept)) {
      return usb_rcvisocpipe(device, ept->bEndpointAddress & 0xF);
    }
    else {
      return usb_sndisocpipe(device, ept->bEndpointAddress & 0xF);
    }

  case PIPE_INTERRUPT:
    if(usb_is_endpoint_in(ept)) {
      return usb_rcvintpipe(device, ept->bEndpointAddress & 0xF);
    }
    else {
      return usb_sndintpipe(device, ept->bEndpointAddress & 0xF);
    }

  case PIPE_BULK:
    if(usb_is_endpoint_in(ept)) {
      return usb_rcvbulkpipe(device, ept->bEndpointAddress & 0xF);
    }
    else {
      return usb_sndbulkpipe(device, ept->bEndpointAddress & 0xF);
    }

  default: // case PIPE_CONTROL:
    if(usb_is_endpoint_in(ept)) {
      return usb_rcvctrlpipe(device, ept->bEndpointAddress & 0xF);
    }
    else {
      return usb_sndctrlpipe(device, ept->bEndpointAddress & 0xF);
    }
  }
}

bool usb_enumerate(usb_hcd_t* usb_hcd, uint dev_speed, uint hub_addr, uint port_num);

typedef bool (*usb_reset_root_ports_func) (usb_hcd_t* usb_hcd);
typedef bool (*usb_post_enumeration_func)(usb_hcd_t* usb_hcd);
typedef int  (*usb_submit_urb_func)(struct urb* urb, gfp_t mem_flags);
typedef void (*usb_kill_urb_func)(struct urb* urb);
typedef int  (*usb_rt_iso_free_packets_func)(struct urb* urb, int number_of_packets);
typedef int  (*usb_rt_iso_update_packets_func)(struct urb* urb, int max_packets);
typedef int  (*usb_rt_data_lost_func)(struct urb* urb);
typedef int  (*usb_rt_free_data_func)(struct urb* urb, int count);
typedef int  (*usb_rt_update_data_func)(struct urb* urb, int max_count);
typedef int  (*usb_rt_push_data_func)(struct urb* urb, char* data, int count, uint interrupt_rate);


/*
 * Generic USB Host controller Device object 
 */
struct _usb_hcd_t
{
  #define USB_MAX_DEVICES 127
  
  uint32_t usb_hc_type;
  usb_reset_root_ports_func      reset_root_ports;
  usb_post_enumeration_func      post_enumeration;
  usb_submit_urb_func            submit_urb;
  usb_kill_urb_func              kill_urb;
  usb_rt_iso_update_packets_func rt_iso_update_packets;
  usb_rt_iso_free_packets_func   rt_iso_free_packets;
  usb_rt_data_lost_func     rt_data_lost;
  usb_rt_free_data_func     rt_free_data;
  usb_rt_update_data_func   rt_update_data;
  usb_rt_push_data_func     rt_push_data;
  uint32_t next_address;
  
  
  USB_DEVICE_INFO devinfo[USB_MAX_DEVICES+1];
  
};


bool initialise_usb_hcd(usb_hcd_t* usb_hcd, uint32_t usb_hc_type,
                        usb_reset_root_ports_func reset_root_ports,
                        usb_post_enumeration_func post_enumeration,
                        usb_submit_urb_func submit_urb,
                        usb_kill_urb_func kill_urb,
                        usb_rt_iso_update_packets_func rt_iso_update_packets,
                        usb_rt_iso_free_packets_func rt_iso_free_packets,
                        usb_rt_data_lost_func rt_data_lost,
                        usb_rt_update_data_func rt_update_data,
                        usb_rt_free_data_func rt_free_data,
                        usb_rt_push_data_func rt_push_data);

bool add_usb_hcd(usb_hcd_t* usb_hcd);

usb_hcd_t* get_usb_hcd(uint32_t index);

void dlog_usb_hcd(usb_hcd_t* usb_hcd);

const char *usb_speed_string(int speed);

#endif


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
