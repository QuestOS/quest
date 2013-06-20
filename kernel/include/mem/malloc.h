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
#include "kernel.h"


#define MALLOC_POOL_NUM_DIR_ENTRIES ((uint32)5)

/* The malloc pool "grows" down, always ending at 1018 */

#define MALLOC_POOL_LAST_DIR_ENTRY ((uint32)1018)
#define MALLOC_POOL_START_DIR_ENTRY                                     \
  ((uint32)(MALLOC_POOL_LAST_DIR_ENTRY - MALLOC_POOL_NUM_DIR_ENTRIES + 1))



void init_malloc (void);
void* kmalloc(uint32 size);
void kfree(void* ptr);

bool init_malloc_pool_dir_entries();
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

static inline void* kmalloc_aligned(uint32 size, uint32 alignment, void** ptr_to_free)
{
  *ptr_to_free = kmalloc(size + alignment - 1);
  if(*ptr_to_free) {
    void* temp = (void*)(((uint)((u8*)(*ptr_to_free)+alignment-1)) & ~((uint32)(alignment-1)));
    return temp;
  }
  return NULL;
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
