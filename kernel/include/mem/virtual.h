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

#ifndef _VIRTUAL_H_
#define _VIRTUAL_H_
#include "types.h"
#include "util/cassert.h"

#define PGTBL_NUM_ENTRIES 0x400
#define PGDIR_NUM_ENTRIES 0x400
#define PGDIR_KERNEL_BEGIN 0x300 /* where shared kernel entries begin */
#define PGDIR_KERNEL_STACK 0x3FE /* per-process kernel stack is not shared */
#define BIGPAGE_SIZE_BITS 22
#define BIGPAGE_SIZE (1<<BIGPAGE_SIZE_BITS)
#define PAGE_SIZE_BITS 12
#define PAGE_SIZE (1<<PAGE_SIZE_BITS)
#define FRAMENUM_TO_FRAME(x) ((frame_t) ((x) << PAGE_SIZE_BITS))
#define FRAME_TO_FRAMENUM(x) ((framenum_t) ((x) >> PAGE_SIZE_BITS))
#define BIGFRAMENUM_TO_FRAME(x) ((frame_t) ((x) << BIGPAGE_SIZE_BITS))
#define FRAME_TO_BIGFRAMENUM(x) ((framenum_t) ((x) >> BIGPAGE_SIZE_BITS))

extern void *map_virtual_page (uint32 phys_frame);
extern void unmap_virtual_page (void *virt_addr);
extern void *map_virtual_pages (uint32 * phys_frames, uint32 count);
extern void *map_contiguous_virtual_pages (uint32 phys_frame, uint32 count);
extern bool map_virtual_page_to_addr(uint dir_entry_perm, uint32 phys_frame, addr_t virt_addr);
extern void unmap_virtual_pages (void *virt_addr, uint32 count);

void* map_pool_virtual_page (uint32 phys_frame, uint32 start_dir_entry, uint32 num_dir_entries,
                             uint32* page_table_virtual_addrs[]);
void* map_pool_virtual_pages (uint32 * phys_frames, uint32 count,
                              uint32 start_dir_entry, uint32 num_dir_entries,
                              uint32* page_table_virtual_addrs[]);
void* map_contiguous_pool_virtual_pages (uint32 phys_frame, uint32 count,
                                         uint32 start_dir_entry, uint32 num_dir_entries,
                                         uint32* page_table_virtual_addrs[]);
void unmap_pool_virtual_page (void *virt_addr, uint32 start_dir_entry, uint32 num_dir_entries,
                              uint32* page_table_virtual_addrs[]);
void unmap_pool_virtual_pages (void *virt_addr, uint32 count, uint32 start_dir_entry, uint32 num_dir_entries,
                               uint32* page_table_virtual_addrs[]);

extern void *get_phys_addr (void *virt_addr);


/* Searches the current process address space to find a free region to
   that can accommodate the request size, useful when you want to map
   something to a process address space in an arbitrary location */
extern void* find_free_virtual_region(size_t size);

typedef union {
  u32 raw;
  struct {
    uint offset:PAGE_SIZE_BITS;
    uint pgtbl_i:(BIGPAGE_SIZE_BITS - PAGE_SIZE_BITS);
    uint pgdir_i:(8*sizeof (u32) - BIGPAGE_SIZE_BITS);
  };
} linear_address_t PACKED;
CASSERT (sizeof (linear_address_t) == sizeof (u32), linear_address_t);

typedef union {
  uint32 raw;
  /* 4 kiB page */
  union {
    uint raw:12;
    struct {
      uint present:1;
      uint writeable:1;
      uint supervisor:1;
      uint write_through:1;
      uint cache_disabled:1;
      uint accessed:1;
      uint dirty:1;
      uint attribute_index:1;
      uint global_page:1;
      uint avail:3;
    };
  } flags;
  struct {
    uint __align:12;
    framenum_t framenum:20;
  };
} pgtbl_entry_t PACKED;
CASSERT (sizeof (pgtbl_entry_t) == sizeof (u32), pgtbl_entry_t);

typedef struct {
  frame_t table_pa;             /* table physical address */
  pgtbl_entry_t *table_va;      /* table virtual address */
  uint8 *starting_va;           /* first virtual address */
} pgtbl_t;

typedef union {
  uint32 raw;

  /* 4 KiB page */
  union {
    union {
      uint32 raw:12;
      struct {
        uint present:1;
        uint writeable:1;
        uint supervisor:1;
        uint write_through:1;
        uint cache_disabled:1;
        uint accessed:1;
        uint avail1:1;
        uint page_size:1;
        uint global_page:1;     /* ignored */
        uint avail3:3;
      };
    } flags;
    struct {
      uint __align:12;
      framenum_t table_framenum:20;
    };
  };

  /* 4 MiB page */
  union {
    union {
      uint32 raw:22;
      struct {
        uint present:1;
        uint writeable:1;
        uint supervisor:1;
        uint write_through:1;
        uint cache_disabled:1;
        uint accessed:1;
        uint dirty:1;
        uint page_size:1;
        uint global_page:1;
        uint avail3:3;
        uint attribute_index:1;
        uint reserved:9;
      };
    } flags_4k;
    struct {
      uint __align_4k:22;
      framenum_t framenum:10;   /* 4 MiB frame number */
    };
  };
} pgdir_entry_t PACKED;
CASSERT (sizeof (pgdir_entry_t) == sizeof (u32), pgdir_entry_t);

typedef struct {
  frame_t dir_pa;               /* directory physical address */
  pgdir_entry_t *dir_va;        /* directory virtual address */
} pgdir_t;


/* precondition: dir has valid VA, PA 
 * postcondition: return has valid VA, PA
 * failure result is (0, 0) */
pgdir_t clone_page_directory (pgdir_t dir);
pgtbl_t clone_page_table (pgtbl_t tbl);
/* precondition: dir PA and VA are valid, va is aligned */
/* postcondition: returned frame is aligned */
/* failure: -1 */
frame_t pgdir_get_frame (pgdir_t dir, void *va);
/* Obtains the physical address of a virtual address in the given
 * page directory. */
/* precondition: dir PA and VA are valid */
/* failure: -1 */
phys_addr_t pgdir_get_phys_addr (pgdir_t dir, void *va);

#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
