#include "drivers/pci/pci.h"
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

void 
pci_init (void)
{
  DLOG("PCI_INIT");
}
