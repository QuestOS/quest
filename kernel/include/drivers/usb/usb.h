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
#include <drivers/usb/uhci.h>

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

#define USB_TYPE_DEV_DESC    0x01
#define USB_TYPE_CFG_DESC    0x02
#define USB_TYPE_STR_DESC    0x03
#define USB_TYPE_IF_DESC     0x04
#define USB_TYPE_EPT_DESC    0x05

#define USB_TYPE_HC_UHCI    0x00
#define USB_TYPE_HC_EHCI    0x01
#define USB_TYPE_HC_OHCI    0x02

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

/* ************************************************** */

typedef struct
{
  uint8 address;
  USB_DEV_DESC devd;
  uint8 host_type;
  uint8 *raw;
} USB_DEVICE_INFO;

typedef struct
{
  bool (*probe) (USB_DEVICE_INFO *, USB_CFG_DESC *, USB_IF_DESC *);
} USB_DRIVER;

bool usb_register_driver (USB_DRIVER *);


/* Generic USB operations */
extern int usb_control_transfer(USB_DEVICE_INFO *, addr_t, uint16_t,
    addr_t, uint16_t);
extern int usb_bulk_transfer(USB_DEVICE_INFO *, uint8_t, addr_t,
    uint16_t, uint8_t, uint8_t);

extern int usb_get_descriptor(USB_DEVICE_INFO *, uint16_t, uint16_t,
    uint16_t, uint16_t, addr_t);
extern int usb_set_address(USB_DEVICE_INFO *, uint8_t);
extern int usb_get_configuration(USB_DEVICE_INFO *);
extern int usb_set_configuration(USB_DEVICE_INFO *, uint8_t);
extern int usb_get_interface(USB_DEVICE_INFO *, uint16_t);
extern int usb_set_interface(USB_DEVICE_INFO *, uint16_t, uint16_t);

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
