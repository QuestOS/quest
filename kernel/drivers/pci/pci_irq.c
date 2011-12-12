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
#include "arch/i386.h"
#include "util/printf.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "kernel.h"

/* A generic interface for storing and looking up PCI IRQ routing
 * information that is typically supplied by ACPI or Intel
 * Multiprocessor Specification tables. */

//#define DEBUG_PCI_IRQ

#ifdef DEBUG_PCI_IRQ
#define DLOG(fmt,...) DLOG_PREFIX("PCI_IRQ",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


#define PCI_IRQ_MAX_COUNT 1024
static uint32 num_pci_irqs=0;
pci_irq_t pci_irqs[PCI_IRQ_MAX_COUNT];

/* Create a PCI IRQ routing entry */
extern void
pci_irq_register (pci_irq_t *irq)
{
  if (num_pci_irqs >= PCI_IRQ_MAX_COUNT) {
    DLOG ("Exceeded PCI_IRQ_MAX_COUNT");
    return;
  }
  DLOG ("Registering %.02X:%.02X Pin %d as GSI=0x%.02X (T=%d P=%d)",
        irq->bus, irq->dev, irq->pin, irq->gsi, irq->trigger, irq->polarity);
  memcpy (&pci_irqs[num_pci_irqs], irq, sizeof (pci_irq_t));

  num_pci_irqs++;
}

/* Look up a PCI IRQ routing entry by PCI bus:dev:pin */
extern bool
pci_irq_find (uint8 bus, uint8 dev, uint8 pin, pci_irq_t *irq_out)
{
  uint i;
  for (i=0; i<num_pci_irqs; i++) {
    if (pci_irqs[i].bus == bus && pci_irqs[i].dev == dev && pci_irqs[i].pin == pin) {
      memcpy (irq_out, &pci_irqs[i], sizeof (pci_irq_t));
      return TRUE;
    }
  }
  return FALSE;
}

/* Map a PCI IRQ routing entry to a given vector with specified CPU
 * destination mask, destination mode, and delivery mode. */
extern bool
pci_irq_map (pci_irq_t *irq, uint8 vector,
             uint8 destmask,
             IOAPIC_destination_mode_t destmode,
             IOAPIC_delivery_mode_t delivmode)
{
  uint64 flags = (uint64) vector;
  flags |= ((uint64) destmask) << 56;
  flags |= (uint64) destmode;
  flags |= (uint64) delivmode;
  if (irq->polarity == POLARITY_LOW ||
      (irq->polarity == POLARITY_DEFAULT && PCI_DEFAULT_POLARITY == POLARITY_LOW))
    flags |= (uint64) IOAPIC_POLARITY_LOW;
  else
    flags |= (uint64) IOAPIC_POLARITY_HIGH;
  if (irq->trigger == TRIGGER_LEVEL ||
      (irq->trigger == TRIGGER_DEFAULT && PCI_DEFAULT_TRIGGER == TRIGGER_LEVEL))
    flags |= (uint64) IOAPIC_TRIGGER_LEVEL;
  else
    flags |= (uint64) IOAPIC_TRIGGER_EDGE;

  return IOAPIC_map_GSI (irq->gsi, vector, flags) >= 0;
}

extern bool
pci_irq_map_handler (pci_irq_t *irq, vector_handler handler,
                     uint8 destmask,
                     IOAPIC_destination_mode_t destmode,
                     IOAPIC_delivery_mode_t delivmode)
{
  u8 vector = find_unused_vector (MINIMUM_VECTOR_PRIORITY);
  DLOG ("vector=0x%X", vector);
  if (!vector)
    return FALSE;
  if (!pci_irq_map (irq, vector, destmask, destmode, delivmode))
    return FALSE;
  set_vector_handler (vector, handler);
  return TRUE;
}

/* Unmap given PCI IRQ routing entry */
extern bool
pci_irq_unmap (pci_irq_t *irq)
{
  return IOAPIC_map_GSI (irq->gsi, 0, 0x0000000000010000LL) >= 0;
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
