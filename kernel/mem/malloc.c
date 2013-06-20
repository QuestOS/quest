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
//#define DEBUG_MALLOC_VERBOSE

#ifdef DEBUG_MALLOC
#define DLOG(fmt, ...) com1_printf(fmt, ##__VA_ARGS__)
#else
#define DLOG(fmt, ...) ;
#endif

#ifdef DEBUG_MALLOC_VERBOSE
#define DLOGV(fmt, ...) com1_printf(fmt, ##__VA_ARGS__)
#else
#define DLOGV(fmt, ...) ;
#endif

static uint32  page_dir_entries[MALLOC_POOL_NUM_DIR_ENTRIES];
static uint32* page_table_virtual_addrs[MALLOC_POOL_NUM_DIR_ENTRIES];

static bool using_page_tables = TRUE;

bool init_malloc_pool_dir_entries()
{
  /* The page tables here won't be carried over to the first process
     but we need to map them anyway because the kmalloc init function
     might call the map malloc page table related functions */
  uint32 cr3 = (uint32)get_pdbr();
  pgdir_entry_t* directory = map_virtual_page((cr3 & 0xFFFFF000) | 3);
  uint i;

  using_page_tables = malloc_uses_page_tables();

  if(directory == NULL) {
    return FALSE;
  }
  
  if(using_page_tables) {
    for(i = 0; i < MALLOC_POOL_NUM_DIR_ENTRIES; ++i) {
      uint32 phys_frame = alloc_phys_frame();
      void* map_result;
      if(phys_frame == 0xFFFFFFFF) {
        return FALSE;
      }
      page_dir_entries[i] = directory[i + MALLOC_POOL_START_DIR_ENTRY].raw = phys_frame | 3;
      page_table_virtual_addrs[i] = map_result = map_virtual_page(phys_frame | 3);
      DLOG("page virt = 0x%p, phys_frame = 0x%X\n", map_result, phys_frame);
      
      if(map_result == NULL) {
        return FALSE;
      }
      memset(map_result, 0, 0x1000);
    }
  }
  else {
    for(i = 0; i < MALLOC_POOL_NUM_DIR_ENTRIES; ++i) {
      uint32 phys_frame = alloc_phys_frames_aligned_on(1024, 0x400000);
      if(phys_frame == 0xFFFFFFFF) {
        return FALSE;
      }
      page_table_virtual_addrs[i] = NULL;
      page_dir_entries[i] = directory[i + MALLOC_POOL_START_DIR_ENTRY].raw = phys_frame | 0x83;
      
    }
  }

  unmap_virtual_page(directory);
  return TRUE;
}

void map_malloc_paging_structures(pgdir_entry_t* pageDir, uint32 offset)
{
  int i, j;
  for(i = 0; i < MALLOC_POOL_NUM_DIR_ENTRIES; ++i) {
    uint32 phys = (page_dir_entries[i] + offset) | 3;
    pageDir[i + MALLOC_POOL_START_DIR_ENTRY].raw = phys;
    if(offset && using_page_tables) {
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

void* map_malloc_pool_virtual_page (uint32 phys_frame)
{
  return using_page_tables ?
    map_pool_virtual_page(phys_frame, MALLOC_POOL_START_DIR_ENTRY,
                          MALLOC_POOL_NUM_DIR_ENTRIES,
                          page_table_virtual_addrs)
    : NULL;
}

void* map_malloc_pool_virtual_pages (uint32 * phys_frames, uint32 count)
{
  return using_page_tables ?
    map_pool_virtual_pages(phys_frames, count, MALLOC_POOL_START_DIR_ENTRY,
                           MALLOC_POOL_NUM_DIR_ENTRIES,
                           page_table_virtual_addrs)
    : NULL;
}

void* map_contiguous_malloc_pool_virtual_pages (uint32 phys_frame, uint32 count)
{
  return using_page_tables ?
    map_contiguous_pool_virtual_pages(phys_frame, count, MALLOC_POOL_START_DIR_ENTRY,
                                      MALLOC_POOL_NUM_DIR_ENTRIES,
                                      page_table_virtual_addrs)
    : NULL;
}

void unmap_malloc_pool_virtual_page (void *virt_addr)
{
  if(using_page_tables)
    unmap_pool_virtual_page(virt_addr, MALLOC_POOL_START_DIR_ENTRY,
                            MALLOC_POOL_NUM_DIR_ENTRIES,
                            page_table_virtual_addrs);
}

void unmap_malloc_pool_virtual_pages (void *virt_addr, uint32 count)
{
  if(using_page_tables)
    unmap_pool_virtual_pages(virt_addr, count, MALLOC_POOL_START_DIR_ENTRY,
                             MALLOC_POOL_NUM_DIR_ENTRIES,
                             page_table_virtual_addrs);
}





/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */
