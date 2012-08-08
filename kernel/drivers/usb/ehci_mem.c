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

inline bool
initialise_qtd(ehci_hcd_t* ehci_hcd, qtd_t* qtd)
{
  memset(qtd, 0, sizeof(*qtd));
  qtd->token = QTD_HALT;
  qtd->next_pointer_raw = EHCI_LIST_END;
  qtd->alt_pointer_raw = EHCI_LIST_END;
  INIT_LIST_HEAD(&qtd->chain_list);
  return TRUE;
}

/* If we cannot allocate a dummy qtd return false */
inline bool initialise_qh(ehci_hcd_t* ehci_hcd, qh_t* qh)
{
  memset(qh, 0, sizeof(*qh));
  qh->state = QH_STATE_NOT_LINKED;
  INIT_LIST_HEAD(&qh->qtd_list);
  qh->dummy_qtd = allocate_qtd(ehci_hcd);
  return qh->dummy_qtd != NULL;
}

inline bool initialise_itd(ehci_hcd_t* ehci_hcd, itd_t* itd)
{
  memset(itd, 0, sizeof(*itd));
  INIT_LIST_HEAD(&itd->chain_list);
  INIT_LIST_HEAD(&itd->uninserted_list);
  return TRUE;
}

inline bool initialise_ehci_completion_element(ehci_hcd_t* ehci_hcd,
                                               ehci_completion_element_t* element)
{
  memset(element, 0, sizeof(*element));
  INIT_LIST_HEAD(&element->chain_list);
  return TRUE;
}

/* End of initialisation functions */

/*
 * -- EM -- I know the following is ugly, but right now since all the
 * pools are statically declared and that is not the way it will
 * permanently be it does not matter.  Eventually the ehci memory
 * stuff will use the kernel heap/we will have a more unified dma pool
 * system similar to Linux and this can be redone to be nicer and
 * simpler since all these static pools will go away
 */

#define EHCI_RESOURCE_MEMORY_FUNCS(res)                                 \
  uint32_t allocate_##res##s(ehci_hcd_t* ehci_hcd, res##_t** items, uint32_t num) \
  {                                                                     \
    uint32_t  count       = 0;                                          \
    uint32_t  entry;                                                    \
    uint32_t  i           = ehci_hcd->used_##res##_bitmap_size;         \
    uint32_t* used_bitmap = ehci_hcd->used_##res##_bitmap;              \
    res##_t*  pool        = ehci_hcd->res##_pool;                       \
                                                                        \
    while(i--) {                                                        \
      if(used_bitmap[i] == INT_MAX) continue;                           \
                                                                        \
      entry = EHCI_ELEMENTS_PER_BITMAP_ENTRY;                           \
      while(entry--) {                                                  \
        if(!(used_bitmap[i] & (1 << entry)) ) {                         \
          used_bitmap[i] |= (1 << entry); /* Mark entry as allocated */ \
          items[count] = &pool[i * EHCI_ELEMENTS_PER_BITMAP_ENTRY + entry]; \
          if(!initialise_##res(ehci_hcd, items[count])) {               \
            free_##res(ehci_hcd, items[count]);                         \
            return count;                                               \
          }                                                             \
          EHCI_MEM_ASSERT( !( ((uint32_t)items[count]) & 0x1F) );       \
          if(++count == num) {                                          \
            return count;                                               \
          }                                                             \
        }                                                               \
      }                                                                 \
    }                                                                   \
    return count;                                                       \
  }                                                                     \
                                                                        \
  inline res##_t* allocate_##res(ehci_hcd_t* ehci_hcd)                  \
  {                                                                     \
    res##_t* temp;                                                      \
    return allocate_##res##s(ehci_hcd, &temp, 1) ? temp : NULL;         \
  }                                                                     \
                                                                        \
  void free_##res##s(ehci_hcd_t* ehci_hcd, res##_t** items, uint32_t num) \
  {                                                                     \
    uint32_t* used_bitmap = ehci_hcd->used_##res##_bitmap;              \
    res##_t*    pool        = ehci_hcd->res##_pool;                     \
                                                                        \
    while(num--) {                                                      \
      uint32_t index = items[num] - pool;                               \
      EHCI_MEM_ASSERT(&pool[index] == items[num]);                      \
                                                                        \
      /* Flip the one bit to zero */                                    \
      used_bitmap[index >> BITMAP_INDEX_SHIFT] &=                       \
        ~(1 << (index & BITMAP_SUBINDEX_MASK));                         \
    }                                                                   \
  }                                                                     \
                                                                        \
  inline void free_##res(ehci_hcd_t* ehci_hcd, res##_t* item)           \
  {                                                                     \
    free_##res##s(ehci_hcd, &item, 1);                                  \
  }
  

EHCI_RESOURCE_MEMORY_FUNCS(qtd)
EHCI_RESOURCE_MEMORY_FUNCS(itd)
EHCI_RESOURCE_MEMORY_FUNCS(qh)
EHCI_RESOURCE_MEMORY_FUNCS(ehci_completion_element)

#undef EHCI_RESOURCE_MEMORY_FUNCS



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
