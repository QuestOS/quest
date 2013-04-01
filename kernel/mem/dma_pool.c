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


#define DEBUG_DMA_POOL


#ifdef DEBUG_DMA_POOL
#define DLOG(fmt,...) DLOG_PREFIX("dma-pool",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif




struct dma_pool* dma_pool_create(const char *name,
				 size_t size, size_t align, size_t boundary)
{
  dma_pool_t* pool = kmalloc(sizeof(dma_pool_t) + strlen(name) + 1);
  if(pool == NULL) return NULL;
  /* Put the name right after the dma_pool itself, avoid two calls to
     kmalloc */
  pool->name = ((char*)pool) + sizeof(dma_pool_t);
  strcpy(pool->name, name);
  pool->size = size;
  pool->align = align;
  pool->boundary = boundary;
  INIT_LIST_HEAD(&pool->dma_pages);

  DLOG("dma_pool: name = %s, size = %d, align = %d, boundary = %d",
       pool->name, pool->size, pool->align, pool->boundary);
  return pool;
}

void *dma_pool_alloc(struct dma_pool *pool, 
		     phys_addr_t *dma_handle)
{
  return NULL;
}

void dma_pool_free(struct dma_pool *pool, void *vaddr,
		   phys_addr_t addr)
{
}

void dma_pool_destroy(struct dma_pool *pool)
{
  dma_page_t *page, *temp;
  
  list_for_each_entry_safe(page, temp, &pool->dma_pages, chain) {
    
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
