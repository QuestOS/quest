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

#include <drivers/usb/ehci.h>
#include <drivers/usb/ehci_mem.h>
#include <util/debug.h>

#define DEBUG_EHCI_MEM
#define DEBUG_EHCI_MEM_VERBOSE

#ifdef DEBUG_EHCI_MEM
#define DLOG(fmt,...) DLOG_PREFIX("EHCI_MEM",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_EHCI_MEM_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("EHCI_MEM",fmt,##__VA_ARGS__)
#else
#define DLOGV(fmt,...) ;
#endif

#ifdef DEBUG_EHCI_MEM_VERBOSE
#define EHCI_MEM_ASSERT(test)						\
  do {                                                                  \
    if(!(test)) {                                                       \
      DLOG("assert in file " __FILE__ " line %d failed", __LINE__);     \
      panic("assert failed");                                           \
    }                                                                   \
  }                                                                     \
  while(0)
#endif

#define BITMAP_ALL_USED_MASK 0xFFFFFFFF
#define BITMAP_INDEX_SHIFT 5
#define BITMAP_SUBINDEX_MASK (EHCI_ELEMENTS_PER_BITMAP_ENTRY - 1)


/* Start of initialisation functions */

inline void
initialise_qtd(qtd_t* qtd, phys_addr_t dma_addr)
{
  memset(qtd, 0, sizeof(*qtd));
  qtd->token = QTD_HALT;
  qtd->next_pointer_raw = EHCI_LIST_END;
  qtd->alt_pointer_raw = EHCI_LIST_END;
  INIT_LIST_HEAD(&qtd->chain_list);
  qtd->dma_addr = dma_addr;
}

inline qtd_t* allocate_qtd(ehci_hcd_t* ehci_hcd)
{
  phys_addr_t dma_addr;
  qtd_t* qtd = dma_pool_alloc(ehci_hcd->qtd_pool, &dma_addr);
  if(qtd == NULL) return NULL;
  initialise_qtd(qtd, dma_addr);
  return qtd;
}

/* If we cannot allocate a dummy qtd return NULL */
inline qh_t* allocate_qh(ehci_hcd_t* ehci_hcd)
{
  phys_addr_t dma_addr;
  qh_t* qh = dma_pool_alloc(ehci_hcd->qh_pool, &dma_addr);
  if(qh == NULL) return NULL;
  
  memset(qh, 0, sizeof(*qh));
  qh->state = QH_STATE_NOT_LINKED;
  INIT_LIST_HEAD(&qh->qtd_list);
  qh->dummy_qtd = allocate_qtd(ehci_hcd);
  if(qh->dummy_qtd == NULL) {
    dma_pool_free(ehci_hcd->qh_pool, qh, dma_addr);
    return NULL;
  }
  qh->dummy_qtd->ioc_called = TRUE; /* avoids calling ioc for dummy qtd */
  qh->previous = &qh->horizontalPointer;
  qh->dma_addr = dma_addr;
  return qh;
}

inline itd_t* allocate_itd(ehci_hcd_t* ehci_hcd)
{
  phys_addr_t dma_addr;
  itd_t* itd = dma_pool_alloc(ehci_hcd->itd_pool, &dma_addr);
  if(itd == NULL) return NULL;
  
  memset(itd, 0, sizeof(*itd));
  INIT_LIST_HEAD(&itd->chain_list);
  INIT_LIST_HEAD(&itd->uninserted_list);
  itd->dma_addr = dma_addr;
  return itd;
}

inline ehci_completion_element_t* allocate_ehci_completion_element()
{
  ehci_completion_element_t* element = kzalloc(sizeof(ehci_completion_element_t));
  if(element == NULL) return NULL;
  INIT_LIST_HEAD(&element->chain_list);
  return element;
}

inline void free_qh(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  dma_pool_free(ehci_hcd->qh_pool, qh, qh->dma_addr);
}

inline void free_qtd(ehci_hcd_t* ehci_hcd, qtd_t* qtd)
{
  list_del(&qtd->chain_list);
  dma_pool_free(ehci_hcd->qtd_pool, qtd, qtd->dma_addr);
}

inline void free_qtds(ehci_hcd_t* ehci_hcd, qtd_t** items, uint num_items)
{
  while(num_items--) {
    free_qtd(ehci_hcd, items[num_items]);
  }
}

inline void free_itd(ehci_hcd_t* ehci_hcd, itd_t* itd)
{
  list_del(&itd->chain_list);
  dma_pool_free(ehci_hcd->itd_pool, itd, itd->dma_addr);
}

inline void free_ehci_completion_element(ehci_completion_element_t* comp_element)
{
  list_del(&comp_element->chain_list);
  kfree(comp_element);
}

/* End of initialisation functions */




/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
