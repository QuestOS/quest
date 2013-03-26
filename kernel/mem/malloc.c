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


#include "kernel.h"
#include "mem/mem.h"
#include "util/printf.h"

//#define DEBUG_MALLOC

#ifdef DEBUG_MALLOC
#define DLOG(fmt, ...) com1_printf(fmt, ##__VA_ARGS__)
#else
#define DLOG(fmt, ...) ;
#endif


static uint32  page_table_phys_addrs   [MALLOC_POOL_NUM_PAGE_TABLES];
static uint32* page_table_virtual_addrs[MALLOC_POOL_NUM_PAGE_TABLES];

bool init_malloc_pool_page_tables()
{
  /* The page tables here won't be carried over to the first process
     but we need to map them anyway because the kmalloc init function
     might call the map malloc page table related functions */
  uint32 cr3 = (uint32)get_pdbr();
  pgdir_entry_t* directory = map_virtual_page((cr3 & 0xFFFFF000) | 3);
  uint i;

  if(directory == NULL) {
    return FALSE;
  }

  for(i = 0; i < MALLOC_POOL_NUM_PAGE_TABLES; ++i) {
    uint32 phys_frame = alloc_phys_frame();
    void* map_result;
    if(phys_frame == 0xFFFFFFFF) {
      return FALSE;
    }
    page_table_phys_addrs[i] = directory[i + MALLOC_POOL_START_PAGE_TABLE].raw = phys_frame | 3;
    page_table_virtual_addrs[i] = map_result = map_virtual_page(phys_frame | 3);
    DLOG("page virt = 0x%p\n", map_result);
    
    if(map_result == NULL) {
      return FALSE;
    }
    memset(map_result, 0, 0x1000);
  }

  unmap_virtual_page(directory);
  return TRUE;
}

void map_malloc_page_tables(pgdir_entry_t* pageDir)
{
  int i;
  for(i = 0; i < MALLOC_POOL_NUM_PAGE_TABLES; ++i) {
    pageDir[i + MALLOC_POOL_START_PAGE_TABLE].raw = page_table_phys_addrs[i] | 3;
  }
}


/* Find free virtual page and map it to a corresponding physical frame
 *
 * Returns virtual address
 *
 */
void *
map_malloc_pool_virtual_page (uint32 phys_frame)
{
  uint i, j;
  void *va;

  for(j = 0; j < MALLOC_POOL_NUM_PAGE_TABLES; ++j) {
    uint32* page_table = page_table_virtual_addrs[j];
    for (i = 0; i < 0x400; i++)
      if (!page_table[i]) {       /* Free page */
        page_table[i] = phys_frame;
        DLOG("In %s, pt entry[%d] = 0x%X\n", __FUNCTION__, i, page_table[i]);
        va = (char *) ((MALLOC_POOL_START_PAGE_TABLE + j) * 0x400000) + (i << 12);
        
        /* Invalidate page in case it was cached in the TLB */
        invalidate_page (va);

        DLOG("mapped 0x%X\n", ((MALLOC_POOL_START_PAGE_TABLE + j) * 0x400000) + (i << 12));
        memset(va, 0, 4096);
        
        return va;
      }
  }
  
  return NULL;                  /* Invalid address */
}


/* Map non-contiguous physical memory to contiguous virtual memory */
void *
map_malloc_pool_virtual_pages (uint32 * phys_frames, uint32 count)
{
  uint i, j, k;
  void *va;

  if (count == 0)
    return NULL;

  /* -- EM -- Right now for simplicity we do not cross 4MB regions
     when trying to map multiple pages */
  for(k = 0; k < MALLOC_POOL_NUM_PAGE_TABLES; ++k) {
    uint32 *page_table = page_table_virtual_addrs[k];
    for (i = 0; i < 0x400 - count + 1; i++) {
      if (!page_table[i]) {       /* Free page */
        for (j = 0; j < count; j++) {
          if (page_table[i + j]) {
            /* Not enough pages in this window */
            i = i + j;
            goto keep_searching;
          }
        }
        
        for (j = 0; j < count; j++) {
          page_table[i + j] = phys_frames[j];
          
        }
        
        va = (char *) ((MALLOC_POOL_START_PAGE_TABLE + k) * 0x400000) + (i << 12);
        
        /* Invalidate page in case it was cached in the TLB */
        for (j = 0; j < count; j++) {
          invalidate_page (va + j * 0x1000);
          DLOG("mapped 0x%X\n", va + j * 0x1000);
          memset(va + j * 0x1000, 0, 4096);
          DLOG("In %s, pt entry[%d] = 0x%X\n", __FUNCTION__, i+j, page_table[i+j]);
        }
        
        return va;
      }
    keep_searching:
      ;
    }
  }
  
  return NULL;                  /* Invalid address */
}

/* Map contiguous physical memory to contiguous virtual memory */
void *
map_contiguous_malloc_pool_virtual_pages (uint32 phys_frame, uint32 count)
{
  uint i, j, k;
  void *va;

  if (count == 0)
    return NULL;

  /* -- EM -- Right now for simplicity we do not cross 4MB regions
     when trying to map multiple pages */
  for(k = 0; k < MALLOC_POOL_NUM_PAGE_TABLES; ++k) {
    uint32 *page_table = page_table_virtual_addrs[k];
    for (i = 0; i < 0x400 - count + 1; i++) {
      if (!page_table[i]) {       /* Free page */
        for (j = 0; j < count; j++) {
          if (page_table[i + j]) {
            /* Not enough pages in this window */
            i = i + j;
            goto keep_searching;
          }
        }
        
        for (j = 0; j < count; j++) {
          page_table[i + j] = (phys_frame + j * 0x1000);
        }
        
        va = (char *) ((MALLOC_POOL_START_PAGE_TABLE + k) * 0x400000) + (i << 12);
        
        /* Invalidate page in case it was cached in the TLB */
        for (j = 0; j < count; j++) {
          invalidate_page (va + j * 0x1000);
          memset(va + j * 0x1000, 0, 4096);
          DLOG("mapped 0x%X\n", va + j * 0x1000);
          DLOG("In %s, pt entry[%d] = 0x%X\n", __FUNCTION__, i+j, page_table[i+j]);
        }
        
        return va;
      }
    keep_searching:
      ;
    }
  }
  return NULL;                  /* Invalid address */
}


/*
 * Release previously mapped virtual page
 */
void
unmap_malloc_pool_virtual_page (void *virt_addr)
{
  uint i;
  for(i = 0; i < MALLOC_POOL_NUM_PAGE_TABLES; ++i) {

    if((0xFFC00000 & (uint32)virt_addr) == ((MALLOC_POOL_START_PAGE_TABLE + i) * 0x400000)) {
      uint32 *page_table = page_table_virtual_addrs[i];
    
      page_table[((uint32) virt_addr >> 12) & 0x3FF] = 0;
      
      /* Invalidate page in case it was cached in the TLB */
      invalidate_page (virt_addr);
      return;
    }
  }
  panic("Failed to unmap page in malloc pool");
}

void
unmap_malloc_pool_virtual_pages (void *virt_addr, uint32 count)
{
  uint j;
  for (j = 0; j < count; j++)
    unmap_malloc_pool_virtual_page (virt_addr + j * 0x1000);
}



/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */
