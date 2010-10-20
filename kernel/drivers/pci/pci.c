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

#include "drivers/pci/pci.h"
#include "drivers/pci/pcidb.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "kernel.h"

#define DEBUG_PCI

#ifdef DEBUG_PCI
#define DLOG(fmt,...) DLOG_PREFIX("PCI",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


#define READ(bus, slot, func, reg, type) \
  pci_read_##type (pci_addr (bus, slot, func, reg))
#define WRITE(bus, slot, func, reg, type, val) \
  pci_write_##type (pci_addr (bus, slot, func, reg), val)


bool
pci_search_ven_table (uint32 vendor, PCI_VENTABLE* e)
{
  uint32 i;
  for (i=0; i<PCI_VENTABLE_LEN; i++)
    if (PciVenTable[i].VenId == vendor) {
      *e = PciVenTable[i];
      return TRUE;
    }
  return FALSE;
}

bool
pci_search_dev_table (uint32 vendor, uint32 dev, PCI_DEVTABLE* e)
{
  uint32 i;
  for (i=0; i<PCI_DEVTABLE_LEN; i++)
    if (PciDevTable[i].VenId == vendor && PciDevTable[i].DevId == dev) {
      *e = PciDevTable[i];
      return TRUE;
    }
  return FALSE;
}

bool
pci_search_class_code_table (uint32 base, uint32 sub, uint32 prog,
                             PCI_CLASSCODETABLE* e)
{
  uint32 i;
  for (i=0; i<PCI_CLASSCODETABLE_LEN; i++)
    if (PciClassCodeTable[i].BaseClass == base &&
        PciClassCodeTable[i].SubClass == sub &&
        PciClassCodeTable[i].ProgIf == prog) {
      *e = PciClassCodeTable[i];
      return TRUE;
    }
  return FALSE;
}


/* store table of PCI devices */
static pci_device devices[PCI_MAX_DEVICES];
static uint32 num_devices = 0;


static void
probe (void)
{
  uint32 bus, slot, func, i;
#ifdef DEBUG_PCI
  PCI_VENTABLE ven;
  PCI_DEVTABLE dev;
  PCI_CLASSCODETABLE cc;
#endif

  for (bus=0; bus <= PCI_MAX_BUS_NUM; bus++)
    for (slot=0; slot <= PCI_MAX_DEV_NUM; slot++)
      for (func=0; func <= PCI_MAX_FUNC_NUM; func++)
        if (READ (bus, slot, func, 0x00, dword) != 0xFFFFFFFF) {

          uint16 vendorID = READ (bus, slot, func, 0x00, word);
          uint16 deviceID = READ (bus, slot, func, 0x02, word);
          uint8 classID   = READ (bus, slot, func, 0x0B, byte);
          uint8 subclID   = READ (bus, slot, func, 0x0A, byte);
          uint8 prgIFID   = READ (bus, slot, func, 0x09, byte);
          uint8 header    = READ (bus, slot, func, 0x0E, byte);

          for (i=0; i<0x10; i++)
            devices[num_devices].data[i] =
              /* BTW, misaligned read from PCI config space causes
               * VMware to crash. :) */
              READ (bus, slot, func, i<<2, dword);

#ifdef DEBUG_PCI
          DLOG ("%.02x:%.02x.%x", bus, slot, func);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[0],
                devices[num_devices].data[1],
                devices[num_devices].data[2],
                devices[num_devices].data[3]);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[4],
                devices[num_devices].data[5],
                devices[num_devices].data[6],
                devices[num_devices].data[7]);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[8],
                devices[num_devices].data[9],
                devices[num_devices].data[10],
                devices[num_devices].data[11]);
          DLOG ("  %.08x %.08x %.08x %.08x",
                devices[num_devices].data[12],
                devices[num_devices].data[13],
                devices[num_devices].data[14],
                devices[num_devices].data[15]);
#endif

#ifdef DEBUG_PCI
          if (classID == 0x02 && subclID == 0)
            printf ("PCI ethernet: %x %x\n", vendorID, deviceID);
          if (classID == 0x0C && subclID == 0x03)
            printf ("USB host controller: %x %x\n", vendorID, deviceID);
          if (pci_search_ven_table (vendorID, &ven))
            DLOG ("  %s (0x%x)", ven.VenFull, vendorID);
          else
            DLOG ("  0x%x", vendorID);
          if (pci_search_dev_table (vendorID, deviceID, &dev))
            DLOG ("  %s (0x%x)", dev.ChipDesc, deviceID);
          else
            DLOG ("  0x%x", deviceID);
          if (pci_search_class_code_table (classID, subclID, prgIFID, &cc))
            DLOG ("  %s (%x) %s (%x) %s (%x)",
                  cc.BaseDesc, classID,
                  cc.SubDesc, subclID,
                  cc.ProgDesc, prgIFID);
#endif

          devices[num_devices].vendor = vendorID;
          devices[num_devices].device = deviceID;
          devices[num_devices].bus = bus;
          devices[num_devices].slot = slot;
          devices[num_devices].func = func;
          devices[num_devices].classcode = classID;
          devices[num_devices].subclass = subclID;
          devices[num_devices].progIF = prgIFID;
          devices[num_devices].headerType = header;
          devices[num_devices].index = num_devices;

          if ((header & 0x7F) == 0) {
            /* 6 BARs */
            for (i=0; i<6; i++) {
              devices[num_devices].bar[i].raw = devices[num_devices].data[4+i];
              if (devices[num_devices].bar[i].raw != 0) {
                /* Save raw data */
                uint32 raw = devices[num_devices].bar[i].raw;
#ifdef DEBUG_PCI
                DLOG ("  BAR%d raw: %p", i, devices[num_devices].bar[i].raw);
#endif
                /* Fill with 1s */
                WRITE (bus, slot, func, 0x10 + i*4, dword, ~0);
                /* Read back mask */
                uint32 mask = READ (bus, slot, func, 0x10 + i*4, dword);
#ifdef DEBUG_PCI
                DLOG ("  BAR%d mask: %p", i, mask);
#endif
                devices[num_devices].bar[i].mask = mask;
                /* Restore raw data */
                WRITE (bus, slot, func, 0x10 + i*4, dword, raw);
              }
            }
          }
          num_devices++;
          if (num_devices >= PCI_MAX_DEVICES) /* reached our limit */
            return;
        }

}

void
pci_init (void)
{
  DLOG("init");
  probe ();
}

bool
pci_get_device (uint n, pci_device *dev)
{
  if (n < num_devices) {
    memcpy (dev, &devices[n], sizeof (pci_device));
    return TRUE;
  }
  return FALSE;
}

bool
pci_find_device (uint16 vendor, uint16 device,
                 uint8 classcode, uint8 subclass,
                 uint start_index,
                 uint* index)
{
#ifdef DEBUG_PCI
  DLOG ("find_device (%p,%p,%p,%p,%p) num_devices=%d",
        vendor, device, classcode, subclass, start_index, num_devices);
#endif
  uint i;
  for (i=start_index; i<num_devices; i++) {
    if ((vendor == 0xFFFF    || vendor == devices[i].vendor) &&
        (device == 0xFFFF    || device == devices[i].device) &&
        (classcode == 0xFF   || classcode == devices[i].classcode) &&
        (subclass == 0xFF    || subclass == devices[i].subclass)) {
      *index = i;
      return TRUE;
    }
  }
  return FALSE;
}

bool
pci_decode_bar (uint index, uint bar_index,
                uint* mem_addr, uint* io_addr, uint* mask)
{
  pci_device *dev;
  pci_bar *bar;

  if (index >= num_devices) return FALSE;

  dev = &devices[index];

  if ((dev->headerType & 0x7F) == 0) {
    /* there are 6 BARs */
    if (bar_index >= 6) return FALSE;

    bar = &dev->bar[bar_index];

    DLOG ("pci_decode_bar (%d, %d) bar->raw=%p", index, bar_index, bar->raw);
    if (bar->raw & 0x1) {
      /* IO port */
      if (io_addr)
        *io_addr = bar->ioBAR.ioPort << 2;
      if (mem_addr)
        *mem_addr = 0;
      if (mask)
        *mask = bar->mask;
    } else {
      /* Memory-mapped */
      if (io_addr)
        *io_addr = 0;
      if (mem_addr)
        *mem_addr = bar->memBAR.baseAddr << 4;
      if (mask)
        *mask = bar->mask;
    }

    return TRUE;
  }

  return FALSE;
}

bool
pci_get_interrupt (uint index, uint* line, uint* pin)
{
  if (index >= num_devices) return FALSE;

  *line = devices[index].data[0xF] & 0xFF;
  *pin = (devices[index].data[0xF] >> 8) & 0xFF;
  return TRUE;
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
