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

#include "types.h"
#include "arch/i386.h"
#include "mem/mem.h"
#include "kernel.h"
#include <util/printf.h>


//#define DEBUG_DMA_POOL


#ifdef DEBUG_DMA_POOL
#define DLOG(fmt,...) DLOG_PREFIX("dma-pool",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


static uint32  page_table_phys_addrs   [DMA_POOL_NUM_PAGE_TABLES];
static uint32* page_table_virtual_addrs[DMA_POOL_NUM_PAGE_TABLES];

static void* map_dma_pool_virtual_page (uint32 phys_frame)
{
  return map_pool_virtual_page(phys_frame, DMA_POOL_START_PAGE_TABLE,
                               DMA_POOL_NUM_PAGE_TABLES,
                               page_table_virtual_addrs);
};

bool init_dma_pool_page_tables()
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

  for(i = 0; i < DMA_POOL_NUM_PAGE_TABLES; ++i) {
    uint32 phys_frame = alloc_phys_frame();
    void* map_result;
    if(phys_frame == 0xFFFFFFFF) {
      return FALSE;
    }
    page_table_phys_addrs[i] = directory[i + DMA_POOL_START_PAGE_TABLE].raw = phys_frame | 3;
    page_table_virtual_addrs[i] = map_result = map_virtual_page(phys_frame | 3);
    DLOG("dir entry = %d, page virt = 0x%p, phys_frame = 0x%X",
         i + DMA_POOL_START_PAGE_TABLE, map_result, phys_frame);
    
    if(map_result == NULL) {
      return FALSE;
    }
    memset(map_result, 0, 0x1000);
  }

  unmap_virtual_page(directory);
  return TRUE;
}

void map_dma_page_tables(pgdir_entry_t* pageDir, uint32 offset)
{
  int i, j;
  for(i = 0; i < DMA_POOL_NUM_PAGE_TABLES; ++i) {
    uint32 phys = (page_table_phys_addrs[i] + offset) | 3;
    pageDir[i + DMA_POOL_START_PAGE_TABLE].raw = phys;
    if(offset) {
      pgtbl_entry_t* page_table = map_virtual_page(phys);
      for(j = 0; j < 1024; ++j) {
        if(page_table[j].raw && page_table[j].raw < offset) {
          page_table[j].raw = ((page_table[j].raw & 0xFFFFF000) + offset) | 3;
        }
      }
      unmap_virtual_page(page_table);
    }
  }
}


struct dma_pool* dma_pool_create(const char *name,
				 size_t size, size_t align, size_t boundary)
{
  dma_pool_t* pool = kmalloc(sizeof(dma_pool_t) + strlen(name) + 1);
  if(pool == NULL) return NULL;
  /* Put the name right after the dma_pool itself, avoid two calls to
     kmalloc */
  pool->name = ((char*)pool) + sizeof(dma_pool_t);
  strcpy(pool->name, name);
  /* -- EM -- Our bitmap for a page is only 32*DMA_PAGE_BITMAP_SIZE so
     if the object is too small for the bitmap we just treat it as
     bigger, fix this later */
  if(size < (4096 / (DMA_PAGE_BITMAP_SIZE * 32))) {
    size = 4096 / (DMA_PAGE_BITMAP_SIZE * 32);
  }

  if ((size % align) != 0) size = ALIGN(size, align);
  
  pool->size = size;
  pool->align = align;

  /* -- EM -- Fix this restriction later */
  if(size > 0x1000) {
    DLOG("Cannot handle dma object larger than a page right now");
    panic("Cannot handle dma object larger than a page right now");
  }
  
  /* -- EM -- Fix this restriction later */
  if(boundary != 0 && boundary != 0x1000) {
    DLOG("dma_pools only work if boundary is 0 or 0x1000");
    panic("dma_pools only work if boundary is 0 or 0x1000");
  }

  pool->objs_per_page = 0x1000 / size;
  pool->boundary = boundary;
  INIT_LIST_HEAD(&pool->dma_pages);

  DLOG("name = %s, size = %d, align = %d, boundary = %d",
       pool->name, pool->size, pool->align, pool->boundary);
  return pool;
}

void *dma_pool_alloc(struct dma_pool *pool, 
		     phys_addr_t *dma_handle)
{
  dma_page_t* page = NULL;
  dma_page_t* temp_page;
  int i;
  DLOG("Calling DMA alloc for pool %s", pool->name);
  list_for_each_entry(temp_page, &pool->dma_pages, chain) {
    DLOG("temp_page = 0x%p  pool = 0x%p", temp_page, pool);
    if(temp_page->obj_count < pool->objs_per_page) {
      page = temp_page;
      break;
    }
  }
  if(page == NULL) {
    page = kmalloc(sizeof(dma_page_t));
    if(page == NULL) {
      return NULL;
    }
    page->phys_addr = alloc_phys_frame();
    if(page->phys_addr == 0xFFFFFFFF) {
      kfree(page);
      return NULL;
    }
    page->virt_addr = map_dma_pool_virtual_page(page->phys_addr | 3);
    if(page->virt_addr == NULL) {
      free_phys_frame(page->phys_addr);
      kfree(page);
      return NULL;
    }
    INIT_LIST_HEAD(&page->chain);
    page->obj_count = 0;
    memset(page->bitmap, 0xFF, DMA_PAGE_BITMAP_SIZE * 4);
    list_add(&page->chain, &pool->dma_pages);
  }

  for(i = 0; i < pool->objs_per_page; ++i) {
    if(BITMAP_TST(page->bitmap, i)) {
      BITMAP_CLR(page->bitmap, i);
      page->obj_count++;
      *dma_handle = ((uint32_t)page->phys_addr) + pool->size * i;
      return &page->virt_addr[pool->size * i];
    }
  }
  DLOG("Error should never reach end of dma_pool_alloc");
#ifdef DEBUG_DMA_POOL
  panic("Error should never reach end of dma_pool_alloc");
#endif
  return NULL;
}

void dma_pool_free(struct dma_pool *pool, void *vaddr,
		   phys_addr_t addr)
{
  dma_page_t* page;
  list_for_each_entry(page, &pool->dma_pages, chain) {
    if((((uint)vaddr) & (~0xFFF)) == ((uint)page->virt_addr)) {
      BITMAP_SET(page->bitmap, ((((uint)vaddr) & (~0xFFF)) / pool->size));
      return;
    }
  }
  DLOG("Error should never reach end of dma_pool_free");
#ifdef DEBUG_DMA_POOL
  panic("Error should never reach end of dma_pool_free");
#endif
}

void dma_pool_destroy(struct dma_pool *pool)
{
  dma_page_t *page;
  
  list_for_each_entry_safe(page, &pool->dma_pages, chain) {
    kfree(page);
  }

  kfree(pool);
}





/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */
