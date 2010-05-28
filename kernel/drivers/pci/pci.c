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
#ifdef DEBUG_PCI
          DLOG ("%.02x:%.02x.%x", bus, slot, func);
          DLOG ("  %.08x %.08x %.08x %.08x",
                READ (bus, slot, func, 0x00, dword),
                READ (bus, slot, func, 0x04, dword),
                READ (bus, slot, func, 0x08, dword),
                READ (bus, slot, func, 0x0C, dword));
          DLOG ("  %.08x %.08x %.08x %.08x",
                READ (bus, slot, func, 0x10, dword),
                READ (bus, slot, func, 0x14, dword),
                READ (bus, slot, func, 0x18, dword),
                READ (bus, slot, func, 0x1C, dword));
          DLOG ("  %.08x %.08x %.08x %.08x",
                READ (bus, slot, func, 0x20, dword),
                READ (bus, slot, func, 0x24, dword),
                READ (bus, slot, func, 0x28, dword),
                READ (bus, slot, func, 0x2C, dword));
          DLOG ("  %.08x %.08x %.08x %.08x",
                READ (bus, slot, func, 0x30, dword),
                READ (bus, slot, func, 0x34, dword),
                READ (bus, slot, func, 0x38, dword),
                READ (bus, slot, func, 0x3C, dword));
#endif
          uint16 vendorID = READ (bus, slot, func, 0x00, word);
          uint16 deviceID = READ (bus, slot, func, 0x02, word);
          uint8 classID   = READ (bus, slot, func, 0x0B, byte);
          uint8 subclID   = READ (bus, slot, func, 0x0A, byte);
          uint8 prgIFID   = READ (bus, slot, func, 0x09, byte);
          uint8 header    = READ (bus, slot, func, 0x0E, byte);

#ifdef DEBUG_PCI
          if (pci_search_ven_table (vendorID, &ven))
            DLOG ("  %s", ven.VenFull);
          if (pci_search_dev_table (vendorID, deviceID, &dev))
            DLOG ("  %s", dev.ChipDesc);
          if (pci_search_class_code_table (classID, subclID, prgIFID, &cc))
            DLOG ("  %s %s %s", cc.BaseDesc, cc.SubDesc, cc.ProgDesc);
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

          for (i=0; i<0x10; i++)
            devices[num_devices].data[i] =
              // BTW, misaligned read from PCI config space causes
              // VMware to crash. :)
              READ (bus, slot, func, i<<2, dword);

          if (header == 0) {
            // 6 BARs
            for (i=0; i<6; i++) {
              devices[num_devices].bar[i].raw = devices[num_devices].data[4+i];
              if (devices[num_devices].bar[i].raw != 0) {
                // Save raw data
                uint32 raw = devices[num_devices].bar[i].raw;
#ifdef DEBUG_PCI
                DLOG ("  BAR%d raw: %p", i, devices[num_devices].bar[i].raw);
#endif
                // Fill with 1s
                WRITE (bus, slot, func, 0x10 + i*4, dword, ~0);
                // Read back mask
                uint32 mask = READ (bus, slot, func, 0x10 + i*4, dword);
#ifdef DEBUG_PCI
                DLOG ("  BAR%d mask: %p", i, mask);
#endif
                devices[num_devices].bar[i].mask = mask;
                // Restore raw data
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
