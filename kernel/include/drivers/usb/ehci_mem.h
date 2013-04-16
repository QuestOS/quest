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


#ifndef _EHCI_MEM_H_
#define _EHCI_MEM_H_


#include <types.h>
#include <arch/i386.h>
#include <drivers/pci/pci.h>
#include <drivers/usb/usb.h>
#include <drivers/usb/ehci.h>

inline void initialise_qtd(qtd_t* qtd, phys_addr_t dma_addr);

inline qtd_t* allocate_qtd(ehci_hcd_t* ehci_hcd);

/* If we cannot allocate a dummy qtd return NULL */
inline qh_t* allocate_qh(ehci_hcd_t* ehci_hcd);

inline itd_t* allocate_itd(ehci_hcd_t* ehci_hcd);

inline ehci_completion_element_t* allocate_ehci_completion_element();

inline void free_qh(ehci_hcd_t* ehci_hcd, qh_t* qh);

inline void free_qtd(ehci_hcd_t* ehci_hcd, qtd_t* qtd);

inline void free_itd(ehci_hcd_t* ehci_hcd, itd_t* itd);
inline void free_ehci_completion_element(ehci_completion_element_t* comp_element);

inline void free_qtds(ehci_hcd_t* ehci_hcd, qtd_t** items, uint num_items);

#endif // _EHCI_MEM_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
