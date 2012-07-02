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

#ifndef _USB_H_
#define _USB_H_

#include <types.h>
#include <drivers/usb/linux_usb.h>


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

#define USB_MAX_LEN       0x3FE
#define USB_NULL_PACKET   0x7FF

#define USB_GET_STATUS           0x00
#define USB_CLEAR_FEATURE        0x01
#define USB_SET_FEATURE          0x03
#define USB_SET_ADDRESS          0x05
#define USB_GET_DESCRIPTOR       0x06
#define USB_SET_DESCRIPTOR       0x07
#define USB_GET_CONFIGURATION    0x08
#define USB_SET_CONFIGURATION    0x09
#define USB_GET_INTERFACE        0x0A
#define USB_SET_INTERFACE        0x0B
#define USB_SYNCH_FRAME          0x0C

#define USB_TYPE_DEV_DESC      0x01
#define USB_TYPE_CFG_DESC      0x02
#define USB_TYPE_STR_DESC      0x03
#define USB_TYPE_IF_DESC       0x04
#define USB_TYPE_EPT_DESC      0x05
#define USB_TYPE_QUA_DESC      0x06
#define USB_TYPE_SPD_CFG_DESC  0x07
#define USB_TYPE_IF_PWR_DESC   0x08

#define USB_TYPE_HC_UHCI    0x00
#define USB_TYPE_HC_EHCI    0x01
#define USB_TYPE_HC_OHCI    0x02


#define PIPE_ISOCHRONOUS  0
#define PIPE_INTERRUPT    1
#define PIPE_CONTROL      2
#define PIPE_BULK         3

#define USB_SPEED_LOW  1
#define USB_SPEED_FULL 2
#define USB_SPEED_HIGH 3

#define USB_DIR_IN  0
#define USB_DIR_OUT 1


#define USB_USER_READ  0
#define USB_USER_WRITE 1



//struct _usb_hcd_t;

typedef struct _usb_hcd_t usb_hcd_t;


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

#define IS_INPUT_USB_DEV_REQ(dev_req_addr) ( (*((uint8_t*)dev_req_addr) ) & 0x80 )

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
  uint8_t MaxPower;
} PACKED;

typedef struct usb_cfg_desc USB_CFG_DESC;

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

/* ************************************************** */

struct _USB_DRIVER;

typedef struct 
{
  uint8 address;
  USB_DEV_DESC devd;
  uint8 host_type;
  usb_hcd_t* hcd;
  uint8 *configurations;
  struct _USB_DRIVER* driver;
  void* device_data; /* A place for a device to put its own private data */

  /*
   *  device can have 32 endpoints (16 endpoint numbers and each
   *  number can be shared by 2 endpoints one IN and one OUT
   */
  uint32 endpoint_toggles;
} USB_DEVICE_INFO;

#define IS_ENDPOINT_TOGGLED(dev_info, endpoint, is_input)               \
  (!!(((dev_info)->endpoint_toggles) & (1 << (endpoint + ( (!!(is_input)) << 4)))))


#define SET_ENDPOINT_TOGGLE(dev_info, endpoint, is_input, val)          \
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
  int (*write) (USB_DEVICE_INFO* device, char* buf, int data_len);
  int (*read) (USB_DEVICE_INFO* device, char* buf, int data_len);
} USB_DRIVER;

bool usb_register_device(USB_DEVICE_INFO* device, USB_DRIVER* driver);

bool usb_register_driver (USB_DRIVER *driver);

USB_DEVICE_INFO* usb_get_device(int device_id);

int usb_syscall_handler(int device_id, int operation, char* buf, int data_len);

/* Generic USB operations */
extern int usb_control_transfer(USB_DEVICE_INFO* dev, addr_t setup_req,
                                uint16_t req_len, addr_t data,
                                uint16_t data_len);

extern int usb_bulk_transfer(USB_DEVICE_INFO* dev, uint8_t endp,
                             addr_t data, uint16_t len, uint16_t packet_len,
                             uint8_t dir, uint32_t *act_len);


extern int usb_isochronous_transfer(USB_DEVICE_INFO* dev,
                                    uint8_t endpoint,
                                    uint16_t packet_len,
                                    uint8_t direction, addr_t data,
                                    usb_iso_packet_descriptor_t* packets,
                                    int num_packets);


extern int usb_get_descriptor(USB_DEVICE_INFO* dev, uint16_t dtype,
                              uint16_t dindex, uint16_t index,
                              uint16_t length, addr_t desc);

extern int usb_set_address(USB_DEVICE_INFO * dev, uint8_t new_addr);

extern int usb_get_configuration(USB_DEVICE_INFO* dev);

extern int usb_set_configuration(USB_DEVICE_INFO* dev, uint8_t conf);

extern int usb_get_interface(USB_DEVICE_INFO* dev, uint16_t interface);

extern int usb_set_interface(USB_DEVICE_INFO* dev, uint16_t alt, uint16_t interface);


bool usb_enumerate(usb_hcd_t* usb_hcd);

typedef bool (*usb_reset_root_ports_func) (usb_hcd_t* usb_hcd);
typedef bool (*usb_post_enumeration_func)(usb_hcd_t* usb_hcd);

/*
 * Generic USB Host controller Device object 
 */
struct _usb_hcd_t
{
  #define USB_MAX_DEVICES 127
  
  uint32_t usb_hc_type;
  usb_reset_root_ports_func reset_root_ports;
  usb_post_enumeration_func post_enumeration;
  uint32_t next_address;
  USB_DEVICE_INFO devinfo[USB_MAX_DEVICES+1];  // +1 so device address corresponds to index
  
};

int usb_syscall_handler(int device_id, int operation, char* buf, int data_len);

bool initialise_usb_hcd(usb_hcd_t* usb_hcd, uint32_t usb_hc_type,
                        usb_reset_root_ports_func reset_root_ports,
                        usb_post_enumeration_func post_enumeration);

bool add_usb_hcd(usb_hcd_t* usb_hcd);

usb_hcd_t* get_usb_hcd(uint32_t index);

void dlog_usb_hcd(usb_hcd_t* usb_hcd);

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
