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


struct dma_pool* dma_pool_create(const char *name,
				 size_t size, size_t align, size_t alloc)
{
  return NULL;
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
}





/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */
