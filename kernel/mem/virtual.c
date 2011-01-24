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

#include "arch/i386.h"
#include "kernel.h"
#include "types.h"
#include "mem/physical.h"


extern uint32 _kernelstart;


/* Find free virtual page and map it to a corresponding physical frame
 *
 * Returns virtual address
 *
 */
void *
map_virtual_page (uint32 phys_frame)
{

  uint32 *page_table = (uint32 *) KERN_PGT;
  int i;
  void *va;

  for (i = 0; i < 0x400; i++)
    if (!page_table[i]) {       /* Free page */
      page_table[i] = phys_frame;

      va = (char *) &_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      invalidate_page (va);

      return va;
    }

  return NULL;                  /* Invalid address */
}

/* Map contiguous physical to virtual memory */
void *
map_contiguous_virtual_pages (uint32 phys_frame, uint32 count)
{
  uint32 *page_table = (uint32 *) KERN_PGT;
  int i, j;
  void *va;

  if (count == 0)
    return NULL;

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
        page_table[i + j] = phys_frame + j * 0x1000;
      }

      va = (char *) &_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      for (j = 0; j < count; j++) {
        invalidate_page (va + j * 0x1000);
      }

      return va;
    }
  keep_searching:
    ;
  }

  return NULL;                  /* Invalid address */
}

/* Map non-contiguous physical memory to contiguous virtual memory */
void *
map_virtual_pages (uint32 * phys_frames, uint32 count)
{
  uint32 *page_table = (uint32 *) KERN_PGT;
  int i, j;
  void *va;

  if (count == 0)
    return NULL;

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

      va = (char *) &_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      for (j = 0; j < count; j++) {
        invalidate_page (va + j * 0x1000);
      }

      return va;
    }
  keep_searching:
    ;
  }

  return NULL;                  /* Invalid address */
}


/*
 * Release previously mapped virtual page
 */
void
unmap_virtual_page (void *virt_addr)
{

  uint32 *page_table = (uint32 *) KERN_PGT;

  page_table[((uint32) virt_addr >> 12) & 0x3FF] = 0;

  /* Invalidate page in case it was cached in the TLB */
  invalidate_page (virt_addr);
}

void
unmap_virtual_pages (void *virt_addr, uint32 count)
{
  int j;
  for (j = 0; j < count; j++)
    unmap_virtual_page (virt_addr + j * 0x1000);
}


void *
get_phys_addr (void *virt_addr)
{

  void *pa;
  uint32 phys_frame;
  uint32 va = (uint32) virt_addr;

  uint32 phys_pdbr = (uint32) get_pdbr (), phys_ptbr;
  uint32 *virt_pdbr, *virt_ptbr;

  virt_pdbr = map_virtual_page (phys_pdbr | 3);
  phys_ptbr = (virt_pdbr[va >> 22] & 0xFFFFF000);
  virt_ptbr = map_virtual_page (phys_ptbr | 3);
  phys_frame = virt_ptbr[(va >> 12) & 0x3FF] & 0xFFFFF000;
  pa = (void *) (phys_frame + (va & 0x00000FFF));
  unmap_virtual_page (virt_ptbr);
  unmap_virtual_page (virt_pdbr);

  return pa;
}


/* ************************************************** */

typedef uint frame_t;

typedef union {
  uint32 raw;
  /* 4 kiB page */
  struct {
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
      } PACKED;
    } flags;
    frame_t frame:20;           /* frame physical address */
  } PACKED;
} pgtbl_entry_t;

typedef struct {
  pgtbl_entry_t *table_va;      /* table virtual address */
  frame_t table_pa;             /* table physical address */
  uint8 *starting_va;           /* first virtual address */
} pgtbl_t;

#define PGTBL_NUM_ENTRIES 0x400

void *
map_virtual_page_pt (pgtbl_entry_t entry, pgtbl_t *tbl)
{
  uint i;
  void *va;

  /* perhaps use a "primitive" mapping facility for this */
  if (tbl->table_va == NULL) return NULL;

  for (i = 0; i < PGTBL_NUM_ENTRIES; i++)
    if (!tbl->table_va[i].flags.present) { /* Free page */
      tbl->table_va[i].raw = entry.raw;

      va = (void *) &tbl->starting_va[i << 12];

      /* Invalidate page in case it was cached in the TLB */
      invalidate_page (va);

      return va;
    }

  return NULL;                  /* Invalid address */
}

typedef union {
  uint32 raw;

  /* 4 KiB page */
  struct {
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
      } PACKED;
    } flags;
    frame_t table_pa:20;        /* page table physical address */
  } PACKED;

  /* 4 MiB page */
  struct {
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
      } PACKED;
    } flags;
    frame_t frame:10;           /* frame physical address */
  } PACKED;
} pgdir_entry_t;

typedef pgdir_entry_t *pgdir_t;
#define PGDIR_NUM_ENTRIES 0x400

/* precondition: entry is valid
 * postcondition: tbl->table_pa and starting_va are valid */
bool
map_pgdir_entry (/* in */ pgdir_t dir,
                 /* in */ bool from_top,
                 /* in */ pgdir_entry_t entry,
                 /* out */ pgtbl_t *tbl)
{
  uint i;

  tbl->table_pa = entry.table_pa;
  tbl->table_va = NULL;

  if (from_top) {
    for (i=PGDIR_NUM_ENTRIES; i>=0; i--) {
      if (!dir[i].flags.present) { /* Free entry */
        dir[i] = entry;
        tbl->starting_va = (uint8 *) (i << 22);
        return TRUE;
      }
    } 
  } else {
    for (i=0; i<PGDIR_NUM_ENTRIES; i++) {
      if (!dir[i].flags.present) { /* Free entry */
        dir[i] = entry;
        tbl->starting_va = (uint8 *) (i << 22);
        return TRUE;
      }
    }
  }

  return FALSE;
}

/* for now */
#define _prim_map_virtual_page map_virtual_page
#define _prim_unmap_virtual_page unmap_virtual_page


/* precondition: all of tbl is valid */
pgtbl_t
clone_page_table (pgtbl_t tbl)
{
  pgtbl_t new_tbl;
  uint i;

  new_tbl.table_pa = alloc_phys_frame ();
  if (new_tbl.table_pa == -1)
    goto abort;
  new_tbl.table_va = _prim_map_virtual_page (new_tbl.table_pa | 3);
  if (new_tbl.table_va == NULL)
    goto abort_tbl_pa;
  new_tbl.starting_va = tbl.starting_va;

  memset (new_tbl.table_va, 0, PGTBL_NUM_ENTRIES * sizeof (pgtbl_entry_t));

  for (i=0; i<PGTBL_NUM_ENTRIES; i++) {
    if (tbl.table_va[i].flags.present) {
      new_tbl.table_va[i].raw = tbl.table_va[i].raw;
    }
  }

  return new_tbl;

 abort_tbl_pa:
  free_phys_frame (new_tbl.table_pa);
 abort:
  new_tbl.table_pa = -1;
  return new_tbl;
}

pgdir_t
clone_page_directory (pgdir_t dir)
{
  frame_t new_pgd_pa;
  pgdir_t new_dir;
  uint i;

  new_pgd_pa = alloc_phys_frame ();

  if (new_pgd_pa == -1)
    goto abort;

  new_dir = _prim_map_virtual_page (new_pgd_pa | 3);
  if (new_dir == NULL)
    goto abort_pgd_pa;

  memset (new_dir, 0, PGDIR_NUM_ENTRIES * sizeof (pgdir_entry_t));

  /* run through dir and make copies of tables */
  for (i=0; i<PGDIR_NUM_ENTRIES; i++) {
    if (dir[i].flags.present) {
      if (dir[i].flags.page_size) {
        /* 4 MiB page */
        new_dir[i].raw = dir[i].raw;
      } else {
        /* Page table */
        pgtbl_t tbl, new_tbl;

        /* setup a pgtbl struct with physical and virtual addresses of
         * the existing page table */
        tbl.table_pa = dir[i].table_pa;
        tbl.table_va = _prim_map_virtual_page (tbl.table_pa | 3);
        if (tbl.table_va == NULL)
          goto abort_pgd_va;
        tbl.starting_va = (uint8 *) (i << 22);

        new_tbl = clone_page_table (tbl);

        _prim_unmap_virtual_page (tbl.table_va);
        _prim_unmap_virtual_page (new_tbl.table_va);

        if (new_tbl.table_pa == -1)
          goto abort_pgd_va;

        /* setup new directory entry with flags and new table address */
        new_dir[i].flags = dir[i].flags;
        new_dir[i].table_pa = new_tbl.table_pa;
      }
    }
  }

  return new_dir;

 abort_pgd_va:
  _prim_unmap_virtual_page (new_dir);
 abort_pgd_pa:
  free_phys_frame (new_pgd_pa);
 abort:
  return NULL;  
}


/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
