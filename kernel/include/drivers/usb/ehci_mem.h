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


#ifndef _EHCI_MEM_H_
#define _EHCI_MEM_H_


#include <types.h>
#include <arch/i386.h>
#include <drivers/pci/pci.h>
#include <drivers/usb/usb.h>

#define EHCI_ELEMENTS_PER_BITMAP_ENTRY (sizeof(uint32_t) * 8)

/* Start of functions related to qtd_t memory management*/

inline void initialise_qtd(qtd_t* qtd);

uint32_t allocate_qtds(ehci_hcd_t* ehci_hcd, qtd_t** qtds, uint32_t num_qtd);

inline qtd_t* allocate_qtd(ehci_hcd_t* ehci_hcd);

void free_qtds(ehci_hcd_t* ehci_hcd, qtd_t** qtds, uint32_t num_qtd);

inline void free_qtd(ehci_hcd_t* ehci_hcd, qtd_t* qtd);

/* End of functions related to qtd_t memory management */



/* Start of functions related to qh_t memory management */

inline bool initialise_qh(ehci_hcd_t* ehci_hcd, qh_t* qh);

uint32_t allocate_qhs(ehci_hcd_t* ehci_hcd, qh_t** qhs, uint32_t num_qh);

inline qh_t* allocate_qh(ehci_hcd_t* ehci_hcd);

void free_qhs(ehci_hcd_t* ehci_hcd, qh_t** qhs, uint32_t num_qh);

inline void free_qh(ehci_hcd_t* ehci_hcd, qh_t* qh);

/* End of functions related to qh_t memory management */

void free_qh_and_qtds(ehci_hcd_t* ehci_hcd, qh_t* qh);


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
