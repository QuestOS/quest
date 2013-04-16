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

#ifndef _HUB_H_
#define _HUB_H_

struct usb_hub_desc
{
  uint8 bDescLength;
  uint8 bDescriptorType;
  uint8 bNbrPorts;
  union {
    uint16 wHubCharacteristics;
    struct {
      uint16 lpsMode:2;         /* logical power switching */
      uint16 compound:1;        /* identifies compound device */
      uint16 opMode:2;          /* over-current protection */
      uint16 _reserved:11;
    };
  };
  uint8 bPwrOn2PwrGood;         /* (in 2ms intervals) */
  uint8 bHubContrCurrent;       /* max power requirement in mA */
  /* followed by DeviceRemovable / PortPwrCtrlMask variable-length fields */
} PACKED;
typedef struct usb_hub_desc USB_HUB_DESC;

#define STATUS_CHANGE_BUFFER_SIZE 2

typedef struct
{
  char status_change_buffer[STATUS_CHANGE_BUFFER_SIZE];
  uint32_t device_bitmap;
  USB_EPT_DESC status_change_endpoint;
  uint next_byte_to_read;
  uint bytes_available;
  struct urb* urb;
  USB_HUB_DESC hub_descriptor;
  USB_DEVICE_INFO* dev;
} hub_info_t;




#endif // _HUB_H_





/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
