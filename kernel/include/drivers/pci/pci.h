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

#ifndef _PCI_H_
#define _PCI_H_

#include "arch/i386.h"
#include "smp/apic.h"
#include "types.h"

/* ************************************************** */

/* PCI DEVICE definition */

#define PCI_MAX_DEVICES 32      /* arbitrary limit */

typedef struct {
  union {
    uint32 raw;
    struct {
      uint32 setBit:1;
      uint32 reserved:1;
      uint32 ioPort:30;
    } ioBAR;
    struct {
      uint32 clrBit:1;
      uint32 memType:2;
      uint32 prefetch:1;
      uint32 baseAddr:28;
    } memBAR;
  };
  uint32 mask;
} pci_bar;

typedef struct {
  uint32 bus, slot, func;
  uint16 vendor;
  uint16 device;
  uint8 classcode;
  uint8 subclass;
  uint8 progIF;
  uint8 headerType;
  pci_bar bar[6];
  uint32 data[0x10];
  uint index;
  void *drvdata;
} pci_device;

#define PCI_ANY_ID (~0)

/* ************************************************** */

/* PCI interface */

/* Initialize and probe for devices */
void pci_init (void);

/* Search for device by vendor, device, classcode, or subclass.  The
 * value (~0) is treated as a wildcard.  'start_index' allows you to
 * skip over devices, for example, if you are looking for multiple
 * matches. */
bool pci_find_device (uint16 vendor, uint16 device,
                      uint8 classcode, uint8 subclass,
                      uint start_index,
                      uint* index);

/* Decode a Base Address Register for the given device.  The value
 * will be written into either the mem_addr or the io_addr
 * parameters. */
bool pci_decode_bar (uint index, uint bar_index,
                     uint* mem_addr, uint* io_addr, uint* mask);

/* Get the interrupt line and pin from the PCI configuration info, for
 * the given device. */
bool pci_get_interrupt (uint index, uint* line, uint* pin);

/* Fill a pci_device struct given a valid device index. */
bool pci_get_device (uint index, pci_device* dev);

/* ************************************************** */

/* PCI IRQ routing interface */

/* IRQ polarity */
enum {
  POLARITY_DEFAULT=0,
  POLARITY_HIGH,
  POLARITY_LOW
};

/* IRQ trigger mode */
enum {
  TRIGGER_DEFAULT=0,
  TRIGGER_EDGE,
  TRIGGER_LEVEL
};

/* Enumerate pins from 1, as PCI config does. */
enum {
  PCI_PIN_A=1,
  PCI_PIN_B,
  PCI_PIN_C,
  PCI_PIN_D
};

#define PCI_DEFAULT_POLARITY POLARITY_LOW
#define PCI_DEFAULT_TRIGGER  TRIGGER_LEVEL

/* PCI IRQ table entry */
typedef struct {
  uint8 bus;
  uint8 dev:5;
  uint8 pin:3;
  uint8 gsi;
  uint8 trigger:4;
  uint8 polarity:4;
} pci_irq_t;

extern void pci_irq_register (pci_irq_t *irq);
extern bool pci_irq_find (uint8 bus, uint8 dev, uint8 pin, pci_irq_t *irq_out);
extern bool pci_irq_map (pci_irq_t *irq, uint8 vector,
                         uint8 destmask,
                         IOAPIC_destination_mode_t destmode,
                         IOAPIC_delivery_mode_t delivmode);

/* ************************************************** */

#define PCI_CONFIG_ADDRESS 0xCF8
#define PCI_CONFIG_DATA    0xCFC

#define PCI_MAX_BUS_NUM  ((1<<8)-1)
#define PCI_MAX_DEV_NUM  ((1<<5)-1)
#define PCI_MAX_FUNC_NUM ((1<<3)-1)

/* ************************************************** */

/* PCI configuration space access */

typedef struct _pci_config_addr
{
  uint32 regNum:8;
  uint32 funcNum:3;
  uint32 devNum:5;
  uint32 busNum:8;
  uint32 reserved:7;
  uint32 enable:1;
} pci_config_addr;

static inline void
pci_config_addr_init (pci_config_addr * a,
                      uint8 bus, uint8 dev, uint8 func, uint8 reg)
{
  a->regNum = reg;
  a->funcNum = func;
  a->devNum = dev;
  a->busNum = bus;
  a->reserved = 0;
  a->enable = 1;
}

static inline pci_config_addr
pci_addr (uint8 bus, uint8 dev, uint8 func, uint8 reg)
{
  pci_config_addr a;
  pci_config_addr_init (&a, bus, dev, func, reg);
  return a;
}

static inline uint8
pci_read_byte (pci_config_addr a)
{
  uint16 offs = a.regNum & 0x3;
  uint32 a32, i32;
  a32 = *((uint32 *) &a) & (~0x3);       /* zero out lowest 2 bits */
  outl (a32, PCI_CONFIG_ADDRESS);
  i32 = inl (PCI_CONFIG_DATA);
  /* offs selects the first, second, third, or fourth byte in i32. */
  return (i32 >> (offs << 3)) & 0xFF;
}

static inline uint16
pci_read_word (pci_config_addr a)
{
  uint16 offs = a.regNum & 0x2;
  uint32 a32, i32;
  a32 = *((uint32 *) &a) & (~0x3);       /* zero out lowest 2 bits */
  outl (a32, PCI_CONFIG_ADDRESS);
  i32 = inl (PCI_CONFIG_DATA);
  /* offs selects the first or second word in i32. */
  return (i32 >> (offs << 3)) & 0xFFFF;
}

static inline uint32
pci_read_dword (pci_config_addr a)
{
  uint32 a32, i32;
  a32 = *((uint32 *) &a) & (~0x3);       /* zero out lowest 2 bits */
  outl (a32, PCI_CONFIG_ADDRESS);
  i32 = inl (PCI_CONFIG_DATA);
  return i32;
}

static inline void
pci_write_byte (pci_config_addr a, uint8 v)
{
  uint16 offs = a.regNum & 0x3;
  uint32 a32;
  a32 = *((uint32 *) &a) & (~0x3);       /* zero out lowest 2 bits */
  outl (a32, PCI_CONFIG_ADDRESS);
  /* offs selects the first, second, third, or fourth byte in i32. */
  outb (v, PCI_CONFIG_DATA + offs);

}

static inline void
pci_write_word (pci_config_addr a, uint16 v)
{
  uint16 offs = a.regNum & 0x2;
  uint32 a32;
  a32 = *((uint32 *) &a) & (~0x3);       /* zero out lowest 2 bits */
  outl (a32, PCI_CONFIG_ADDRESS);
  /* offs selects the first or second word in i32. */
  outw (v, PCI_CONFIG_DATA + offs);
}

static inline void
pci_write_dword (pci_config_addr a, uint32 v)
{
  uint32 a32;
  a32 = *((uint32 *) &a) & (~0x3);       /* zero out lowest 2 bits */
  outl (a32, PCI_CONFIG_ADDRESS);
  outl (v, PCI_CONFIG_DATA);
}

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
