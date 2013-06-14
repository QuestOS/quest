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


#ifndef _MALLOC_H_
#define _MALLOC_H_

#include "types.h"
#include "arch/i386.h"
#include "mem/mem.h"


#define MALLOC_POOL_NUM_PAGE_TABLES ((uint32)5)

/* The malloc pool "grows" down, always ending at 1018 */

#define MALLOC_POOL_LAST_PAGE_TABLE ((uint32)1018)
#define MALLOC_POOL_START_PAGE_TABLE                                    \
  ((uint32)(MALLOC_POOL_LAST_PAGE_TABLE - MALLOC_POOL_NUM_PAGE_TABLES + 1))



void init_malloc (void);
void* kmalloc(uint32 size);
void kfree(void* ptr);

bool init_malloc_pool_page_tables();
void map_malloc_paging_structures(pgdir_entry_t* pageDir, uint32 offset);
bool malloc_uses_page_tables();

void* map_malloc_pool_virtual_page (uint32 phys_frame);
void* map_malloc_pool_virtual_pages (uint32 * phys_frames, uint32 count);
void* map_contiguous_malloc_pool_virtual_pages (uint32 phys_frame, uint32 count);
void unmap_malloc_pool_virtual_page (void *virt_addr);
void unmap_malloc_pool_virtual_pages (void *virt_addr, uint32 count);

static inline void* kzalloc(uint32 size) {
  void* temp = kmalloc(size);
  if(temp) {
    memset(temp, 0, size);
  }
  return temp;
}

#endif //_MALLOC_H_

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */
