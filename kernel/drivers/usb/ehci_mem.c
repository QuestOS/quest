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

#include <drivers/usb/ehci.h>
#include <drivers/usb/ehci_mem.h>
#include <util/debug.h>

#define DEBUG_EHCI_MEM
#define DEBUG_EHCI_MEM_VERBOSE

#ifdef DEBUG_EHCI_MEM
#define DLOG(fmt,...) DLOG_PREFIX("EHCI",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#ifdef DEBUG_EHCI_MEM_VERBOSE
#define DLOGV(fmt,...) DLOG_PREFIX("EHCI",fmt,##__VA_ARGS__)
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



/* Start of functions related to qtd_t memory management*/

inline void
initialise_qtd(qtd_t* qtd)
{
  memset(qtd, 0, sizeof(*qtd));
  qtd->token = QTD_HALT;
  qtd->next_pointer_raw = EHCI_LIST_END;
  qtd->alt_pointer_raw = EHCI_LIST_END;
}

uint32_t
allocate_qtds(ehci_hcd_t* ehci_hcd, qtd_t** qtds, uint32_t num_qtd)
{
  DLOG("ehci_hcd : 0x%X", ehci_hcd);
  uint32_t  count       = 0;
  uint32_t  entry;
  uint32_t  i           = ehci_hcd->used_qtd_bitmap_size;
  uint32_t* used_bitmap = ehci_hcd->used_qtd_bitmap;
  qtd_t*    pool        = ehci_hcd->qtd_pool;
  
  while(i--) {
    if(used_bitmap[i] == INT_MAX) continue; /* all qtd at this bitmap
                                               entry are used */
    entry = EHCI_ELEMENTS_PER_BITMAP_ENTRY;
    while(entry--) {
      if(!(used_bitmap[i] & (1 << entry)) ) {
        used_bitmap[i] |= (1 << entry); /* Mark entry as allocated */
        qtds[count] = &pool[i * EHCI_ELEMENTS_PER_BITMAP_ENTRY + entry];
        initialise_qtd(qtds[count]);
        EHCI_MEM_ASSERT( !( ((uint32_t)qtds[count]) & 0x1F) );
	if(++count == num_qtd) return count;
      }
    }
  }
  return count;
}

inline qtd_t*
allocate_qtd(ehci_hcd_t* ehci_hcd)
{
  qtd_t* qtd;
  qtd_t** temp = &qtd;
  return allocate_qtds(ehci_hcd, temp, 1) ? qtd : NULL;
}

void
free_qtds(ehci_hcd_t* ehci_hcd, qtd_t** qtds, uint32_t num_qtd)
{
  uint32_t* used_bitmap = ehci_hcd->used_qtd_bitmap;
  qtd_t*    pool        = ehci_hcd->qtd_pool;
  
  while(num_qtd--) {
    uint32_t qtd_index = qtds[num_qtd] - pool;
    EHCI_MEM_ASSERT(&pool[qtd_index] == qtds[num_qtd]);
    
    /* Flip the one bit to zero */
    used_bitmap[qtd_index << BITMAP_INDEX_SHIFT] &=
      ~(1 << (qtd_index |  BITMAP_SUBINDEX_MASK));
  }
}

inline void
free_qtd(ehci_hcd_t* ehci_hcd, qtd_t* qtd)
{
  free_qtds(ehci_hcd, &qtd, 1);
}

/* End of functions related to qtd_t memory management */



/* Start of functions related to qh_t memory management */

/* If we cannot allocate a dummy qtd return false */
inline bool initialise_qh(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  memset(qh, 0, sizeof(*qh));
  qh->state = QH_STATE_NOT_LINKED;
  qh->dummy_qtd = allocate_qtd(ehci_hcd);
  return qh->dummy_qtd != NULL;
}

uint32_t allocate_qhs(ehci_hcd_t* ehci_hcd, qh_t** qhs, uint32_t num_qh)
{
  
  uint32_t  count       = 0;
  uint32_t  entry;
  uint32_t  i           = ehci_hcd->used_queue_head_bitmap_size;
  uint32_t* used_bitmap = ehci_hcd->used_queue_head_bitmap;
  qh_t*     pool        = ehci_hcd->queue_head_pool;

  
  DLOG("Size of qh_t 0x%X", sizeof(qh_t));
  while(i--) {
    /* if all queue heads at this bitmap entry are used */
    if(used_bitmap[i] == INT_MAX) continue;
    
    entry = EHCI_ELEMENTS_PER_BITMAP_ENTRY;
    while(entry--) {
      if(!(used_bitmap[i] & (1 << entry)) ) {
        used_bitmap[i] |= (1 << entry); /* Mark entry as allocated */
        qhs[count] = &pool[i * EHCI_ELEMENTS_PER_BITMAP_ENTRY + entry];
        DLOGV("qhs[count] = 0x%X", qhs[count]);
        EHCI_MEM_ASSERT( !( ((uint32_t)qhs[count]) & 0x1F) );
        
        /*
         * If we fail to initialise a qh, free the qh and return the
         * number of qh's successfully initialized
         */
        if(!initialise_qh(ehci_hcd, qhs[count])) {
          free_qh(ehci_hcd, qhs[count]);
          return count;
        }
        
        if(++count == num_qh) return count;
      }
    }
  } return count;
}

inline qh_t* allocate_qh(ehci_hcd_t* ehci_hcd)
{
  qh_t* qh;
  return allocate_qhs(ehci_hcd, &qh, 1) ? qh : NULL;
}

void free_qhs(ehci_hcd_t* ehci_hcd, qh_t** qhs, uint32_t num_qh)
{
  uint32_t* used_bitmap = ehci_hcd->used_queue_head_bitmap;
  qh_t*     pool        = ehci_hcd->queue_head_pool;
  
  while(num_qh--) {
    uint32_t qh_index = qhs[num_qh] - pool;
    EHCI_MEM_ASSERT(&pool[qh_index] == qhs[num_qh]);
    
    /* Flip the one bit to zero */
    used_bitmap[qh_index << BITMAP_INDEX_SHIFT] &=
      ~(1 << (qh_index |  BITMAP_SUBINDEX_MASK));
  }
}
 

inline void free_qh(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  free_qhs(ehci_hcd, &qh, 1);
}


/* End of functions related to qh_t memory management */

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
