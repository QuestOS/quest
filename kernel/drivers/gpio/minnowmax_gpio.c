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

#include "drivers/pci/pci.h"
#include "util/printf.h"
#include "mem/mem.h"

#define DEBUG_MGPIO

#ifdef DEBUG_MGPIO
#define DLOG(fmt,...) DLOG_PREFIX("MINNOWMAX GPIO",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static void *mmio_base;

#define MINNOWMAX_iLB_VID			0x8086
#define	MINNOWMAX_iLB_DID		 	0x0f1c

static pci_device quark_gpio_pci_device;

bool
minnowmax_gpio_init()
{
	uint device_index, irq_line, irq_pin;
	uint mem_addr;
	pci_irq_t irq;

	if (!pci_find_device(MINNOWMAX_iLB_VID, MINNOWMAX_iLB_DID,
				0xFF, 0xFF, 0, &device_index))
		return FALSE;
	if (device_index == (uint)(~0)) {
    DLOG ("Unable to detect compatible device.");
    return FALSE;
  }
	DLOG ("Found device_index=%d", device_index);

	if (!pci_get_device(device_index, &quark_gpio_pci_device)) {
		DLOG("Unable to get PCI device from PCI subsystem");
		return FALSE;
	}
  DLOG ("Using PCI bus=%x dev=%x func=%x",
        quark_gpio_pci_device.bus,
        quark_gpio_pci_device.slot,
        quark_gpio_pci_device.func);

	if (!pci_decode_bar (device_index, 2, &mem_addr, NULL, NULL)) {
    DLOG ("Invalid PCI configuration or BAR0 not found");
    return FALSE;
  } 
	if (mem_addr == 0) {
    DLOG ("Unable to detect memory mapped IO region");
    return FALSE;
  }
  mmio_base = map_virtual_page (mem_addr | 0x3);
  if (mmio_base == NULL) {
    DLOG ("Unable to map page to phys=%p", mem_addr);
    return FALSE;
  }
  DLOG ("Using memory mapped IO at phys=%p virt=%p", mem_addr, mmio_base);

	if (!pci_get_interrupt(device_index, &irq_line, &irq_pin)) {
		DLOG("Unable to get IRQ");
		goto abort;
	}
	if (pci_irq_find(quark_gpio_pci_device.bus, quark_gpio_pci_device.slot,
				irq_pin, &irq)) {
		/* use PCI routing table */
    DLOG ("Found PCI routing entry irq.gsi=0x%x", irq.gsi);
		//if (!pci_irq_map_handler(&irq, shared_irq_handler, get_logical_dest_addr(0),
		//			IOAPIC_DESTINATION_LOGICAL,
		//			IOAPIC_DELIVERY_FIXED))
		//	goto abort;
		//irq_line = irq.gsi;
	}
  DLOG ("Using IRQ line=%.02X pin=%X", irq_line, irq_pin);

	/* set up cypress interrupt */
	//quark_gpio_set_interrupt_type(CYPRESS_INT_LINE, EDGE);
	//quark_gpio_set_interrupt_polarity(CYPRESS_INT_LINE, RISING_EDEG);
	//quark_gpio_clear_interrupt(CYPRESS_INT_LINE);
	//quark_gpio_interrupt_enable(CYPRESS_INT_LINE);
	return TRUE;

abort:
	unmap_virtual_page(mmio_base);
	return FALSE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = minnowmax_gpio_init
};

DEF_MODULE (minnowmax_quark_gpio, "MinnowBoard Max GPIO driver", &mod_ops, {"pci"});


