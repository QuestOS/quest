/* Intel e1000 NIC driver */

#include "drivers/pci/pci.h"
#include "drivers/net/ethernet.h"
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "kernel.h"

#define DEBUG_E1000

#ifdef DEBUG_E1000
#define DLOG(fmt,...) DLOG_PREFIX("e1000",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* List of compatible cards (ended by { 0xFFFF, 0xFFFF }) */
static struct { uint16 vendor, device; } compatible_ids[] = {
  { 0x8086, 0x100E },
  { 0xFFFF, 0xFFFF }
};

static uint8 hwaddr[ETH_ADDR_LEN];
static uint device_index, mem_addr, irq_line, irq_pin, e1000_phys;
static uint32 *mmio_base;

#define REG(x) (mmio_base[x])
#define EERD   (REG (5))

static struct e1000 {
} e1000 ALIGNED(4096);

/* Virtual-to-Physical */
#define V2P(ty,p) ((ty)((((uint) (p)) - ((uint) &e1000))+e1000_phys))
/* Physical-to-Virtual */
#define P2V(ty,p) ((ty)((((uint) (p)) - e1000_phys)+((uint) &e1000)))

extern bool
e1000_init (void)
{
  uint i, io_base, mask;

  if (mp_ISA_PC) {
    DLOG ("Requires PCI support");
    goto abort;
  }

  for (i=0; compatible_ids[i].vendor != 0xFFFF; i++)
    if (pci_find_device (compatible_ids[i].vendor, compatible_ids[i].device,
                         0xFF, 0xFF, 0, &device_index))
      break;
    else
      device_index = ~0;

  if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    goto abort;
  }

  DLOG ("Found device_index=%d", device_index);

  if (!pci_decode_bar (device_index, 0, &mem_addr, &io_base, &mask)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    goto abort;
  }

  if (mem_addr == 0) {
    DLOG ("Unable to detect memory mapped IO region");
    goto abort;
  }

  mmio_base = map_virtual_page (mem_addr | 3);

  if (mmio_base == NULL) {
    DLOG ("Unable to map page to phys=%p", mem_addr);
    goto abort;
  }

  DLOG ("Using memory mapped IO at phys=%p virt=%p", mem_addr, mmio_base);

  e1000_phys = (uint32) get_phys_addr (&e1000);

  DLOG ("e1000 structure mapped at phys=%p virt=%p", e1000_phys, &e1000);

  if (!pci_get_interrupt (device_index, &irq_line, &irq_pin)) {
    DLOG ("Unable to get IRQ");
    goto abort_mmap;
  }

  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

  /* read hardware individual address */
  EERD = 0x0001;                /* EERD.START=0 EERD.ADDR=0 */
  while (!(EERD & 0x10)) asm volatile ("pause"); /* wait for EERD.DONE */
  hwaddr[0] = (EERD >> 16) & 0xFF;
  hwaddr[1] = (EERD >> 24) & 0xFF;
  EERD = 0x0101;                /* EERD.START=0 EERD.ADDR=1 */
  while (!(EERD & 0x10)) asm volatile ("pause"); /* wait for EERD.DONE */
  hwaddr[2] = (EERD >> 16) & 0xFF;
  hwaddr[3] = (EERD >> 24) & 0xFF;
  EERD = 0x0201;                /* EERD.START=0 EERD.ADDR=2 */
  while (!(EERD & 0x10)) asm volatile ("pause"); /* wait for EERD.DONE */
  hwaddr[4] = (EERD >> 16) & 0xFF;
  hwaddr[5] = (EERD >> 24) & 0xFF;

  DLOG ("hwaddr=%.02x:%.02x:%.02x:%.02x:%.02x:%.02x",
        hwaddr[0], hwaddr[1], hwaddr[2], hwaddr[3], hwaddr[4], hwaddr[5]);

  return TRUE;

 abort_mmap:
  unmap_virtual_page (mmio_base);
 abort:
  return FALSE;
}
