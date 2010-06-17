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

#ifndef _UHCI_H_
#define _UHCI_H_

#include <types.h>
#include <arch/i386.h>
#include <drivers/pci/pci.h>
typedef void *addr_t;

#define NUM_FRM_PTRS 1024

#define UHCI_PID_OUT             0xE1
#define UHCI_PID_IN              0x69
#define UHCI_PID_SETUP           0x2D

#define UHCI_MAX_LEN      0x4FF

#define GET_USB_BASE(bus, dev, func)    pci_config_rd16(bus, dev, func, 0x20) & 0xFFE0
#define GET_USB_RELNUM(bus, dev, func)    pci_config_rd8(bus, dev, func, 0x60)
#define DISABLE_LEGACY(bus, dev, func)    pci_config_wr16(bus, dev, func, 0xC0, 0x8f00)
#define SET_LEGACY(bus, dev, func, leg)    pci_config_wr16(bus, dev, func, 0xC0, leg);
#define GET_LEGACY(bus, dev, func)    pci_config_rd16(bus, dev, func, 0xC0)
#define GET_PCICMD(bus, dev, func)    pci_config_rd16(bus, dev, func, 0x04)
#define SET_PCICMD(bus, dev, func, cmd)    pci_config_wr16(bus, dev, func, 0x04, cmd)
#define GET_PCISTS(bus, dev, func)    pci_config_rd16(bus, dev, func, 0x06)

#define GET_USBCMD(usb_base)    inw(usb_base + 0x00)
#define SET_USBCMD(usb_base, cmd)    outw(cmd, usb_base + 0x00)
#define GET_USBSTS(usb_base)    inw(usb_base + 0x02)
#define SET_USBSTS(usb_base, sts)    outw(sts, usb_base + 0x02)
#define SET_FRBASEADD(usb_base, frbase)    outl(frbase, usb_base + 0x08)
#define GET_FRBASEADD(usb_base)    inl(usb_base + 0x08)
#define SET_SOFMOD(usb_base, sofmod)    outb(sofmod, usb_base + 0x0C)
#define SET_FRNUM(usb_base, frnum)    outw(frnum, usb_base + 0x06)
#define GET_FRNUM(usb_base)    inw(usb_base + 0x06)
#define SET_PORTSC0(usb_base, sc)    outw(sc, usb_base + 0x10)
#define SET_PORTSC1(usb_base, sc)    outw(sc, usb_base + 0x12)
#define GET_PORTSC0(usb_base)    inw(usb_base + 0x10)
#define GET_PORTSC1(usb_base)    inw(usb_base + 0x12)
#define SET_USBINTR(usb_base, intr)    outw(intr, usb_base + 0x04)
#define GET_USBINTR(usb_base)    inw(usb_base + 0x04)

#define TD_POOL_SIZE 100
#define QH_POOL_SIZE 16
#define TYPE_TD 0
#define TYPE_QH 1
#define DIR_IN  0
#define DIR_OUT 1

/*
 * frm_lst_ptr : UHCI Frame List Pointer
 *
 * Reference :
 *     Universal Host Controller Interface (UHCI) Design Guide
 *     Revision 1.1, page 20, Intel
 */
typedef uint32_t frm_lst_ptr;

/*
 * UHCI_TD : UHCI Transfer Descriptor
 *
 * Fields  :
 *     link_ptr        TD Link Pointer
 *     ctl_status      TD Control and Status
 *     token           TD Token
 *     buf_ptr         TD Buffer Pointer
 *
 * Reference :
 *     Universal Host Controller Interface (UHCI) Design Guide
 *     Revision 1.1, Page 21, Intel
 */
typedef struct
{
  uint32_t link_ptr;

  union {
    uint32_t raw2;
    struct {
      uint32_t act_len:11;
      uint32_t reserve0:5;
      uint32_t status:8;
      uint32_t ioc:1;
      uint32_t iso:1;
      uint32_t ls:1;
      uint32_t c_err:2;
      uint32_t spd:1;
      uint32_t reserve1:2;
    };
  };

  union {
    uint32_t raw3;
    struct {
      uint32_t pid:8;
      uint32_t addr:7;
      uint32_t endp:4;
      uint32_t toggle:1;
      uint32_t reserve2:1;
      uint32_t max_len:11;
    };
  };

  uint32_t buf_ptr;

  /* Use reserved word for buf_ptr virt addr */
  addr_t buf_vptr;
  /* Call back funtion pointer of this TD */
  void (*call_back)(addr_t);

  /* --??-- This is problem for 64-bit */
  /* Reserved for software */
  uint32_t reserve[2];
} UHCI_TD;

/*
 * UHCI_QH : UHCI Queue Head
 *
 * Fields  :
 *     qh_ptr        Queue Head Link Pointer
 *     qe_ptr        Queue Element Link Pointer
 *
 * Reference :
 *     Universal Host Controller Interface (UHCI) Design Guide
 *     Revision 1.1, Page 25, Intel
 */
typedef struct
{
  uint32_t qh_ptr;
  uint32_t qe_ptr;
  uint32_t padding[2];          // QH must be aligned on 16-byte boundary
} UHCI_QH;

extern int uhci_init (void);
extern int uhci_reset (void);
extern int port_reset (uint8_t);
extern int uhci_isochronous_transfer (uint8_t, uint8_t, addr_t, int, uint16_t,
                                      uint8_t, void (*) (addr_t));
extern int uhci_control_transfer (uint8_t, addr_t, int, addr_t, int, int);
extern int uhci_bulk_transfer (uint8_t, uint8_t, addr_t, int, int, uint8_t);
extern int uhci_get_descriptor (uint8_t, uint16_t, uint16_t, uint16_t,
                                uint16_t, addr_t, uint8_t);
extern int uhci_set_address (uint8_t, uint8_t, uint8_t);
extern int uhci_get_configuration (uint8_t, uint8_t);
extern int uhci_set_configuration (uint8_t, uint8_t, uint8_t);
extern int uhci_get_interface (uint8_t, uint16_t, uint8_t);
extern int uhci_set_interface (uint8_t, uint16_t, uint16_t, uint8_t);

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
