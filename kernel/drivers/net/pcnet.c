#include "drivers/pci/pci.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "kernel.h"

#define DEBUG_PCNET

#ifdef DEBUG_PCNET
#define DLOG(fmt,...) DLOG_PREFIX("PCnet",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct { uint16 vendor, device; } compatible_ids[] = {
  { 0x1022, 0x2000 },
  { 0xFFFF, 0xFFFF }
};

static uint device_index, io_base, irq_line, irq_pin;

void pcnet_init (void)
{
  uint i, mem_addr, mask;

  for (i=0; compatible_ids[i].vendor != 0xFFFF; i++)
    if (pci_find_device (compatible_ids[i].vendor, compatible_ids[i].device, 
                         0xFF, 0xFF, 0, &device_index))
      break;
    else
      device_index = ~0;

  if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    goto give_up;
  }

  DLOG ("Found device_index=%d", device_index);

  if (!pci_decode_bar (device_index, 0, &mem_addr, &io_base, &mask)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    goto give_up;
  }

  if (io_base == 0) {
    DLOG ("Memory-mapped I/O not implemented");
    goto give_up;
  }

  DLOG ("Using io_base=%.04X", io_base);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto give_up;
  }

  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

 give_up:
  return;
}
