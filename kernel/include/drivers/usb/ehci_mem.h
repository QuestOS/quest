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


#define EHCI_ELEMENTS_PER_BITMAP_ENTRY (sizeof(uint32_t) * 8)

  



#define __EHCI_POOL_PHYS_TO_VIRT(hcd, phys_addr, pool)                  \
  ((((uint32_t)phys_addr) - ((uint32_t)(hcd)->pool ## _phys_addr)) + ((uint32_t)(hcd)->pool))

#define __EHCI_POOL_VIRT_TO_PHYS(hcd, virt_addr, pool)                  \
  ((((uint32_t)virt_addr) - ((uint32_t)(hcd)->pool)) + ((uint32_t)(hcd)->pool ## _phys_addr))

#define EHCI_QH_VIRT_TO_PHYS(hcd, qh_virt_addr)                 \
  __EHCI_POOL_VIRT_TO_PHYS(hcd, qh_virt_addr, qh_pool)

#define EHCI_QH_PHYS_TO_VIRT(hcd, qh_phys_addr)                 \
  (qh_t*)__EHCI_POOL_PHYS_TO_VIRT(hcd, qh_phys_addr, qh_pool)

#define EHCI_QTD_VIRT_TO_PHYS(hcd, qtd_virt_addr)               \
  __EHCI_POOL_VIRT_TO_PHYS(hcd, qtd_virt_addr, qtd_pool)

#define EHCI_QTD_PHYS_TO_VIRT(hcd, qtd_phys_addr)               \
  (qtd_t*)__EHCI_POOL_PHYS_TO_VIRT(hcd, qtd_phys_addr, qtd_pool)

#define EHCI_ITD_VIRT_TO_PHYS(hcd, itd_virt_addr)               \
  __EHCI_POOL_VIRT_TO_PHYS(hcd, itd_virt_addr, itd_pool)

#define EHCI_ITD_PHYS_TO_VIRT(hcd, itd_phys_addr)               \
  (itd_t*)__EHCI_POOL_PHYS_TO_VIRT(hcd, itd_phys_addr, itd_pool)


#define DECLARE_EHCI_MEM_FUNCTIONS(type)                                \
  inline bool initialise_##type(ehci_hcd_t* ehci_hcd, type##_t* item);        \
  uint32_t allocate_##type##s(ehci_hcd_t* ehci_hcd, type##_t** items,   \
                              uint32_t num_items);                      \
  inline type##_t* allocate_##type(ehci_hcd_t* ehci_hcd);               \
  void free_##type##s(ehci_hcd_t* ehci_hcd, type##_t** items,           \
                      uint32_t num_items);                              \
  inline void free_##type(ehci_hcd_t* ehci_hcd, type##_t* item);

DECLARE_EHCI_MEM_FUNCTIONS(qtd)
DECLARE_EHCI_MEM_FUNCTIONS(qh)
DECLARE_EHCI_MEM_FUNCTIONS(itd)
DECLARE_EHCI_MEM_FUNCTIONS(ehci_completion_element)



#undef DECLARE_EHCI_MEM_FUNCTIONS

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
