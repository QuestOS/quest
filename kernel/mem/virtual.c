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

#include "arch/i386.h"
#include "kernel.h"
#include "types.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "util/printf.h"

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
  if (virt_pdbr[va >> 22] & 0x80) {
    /* 4MB page */
    pa = (void *) (phys_ptbr + (va & 0x003FFFFF));
  } else {
    virt_ptbr = map_virtual_page (phys_ptbr | 3);
    phys_frame = virt_ptbr[(va >> 12) & 0x3FF] & 0xFFFFF000;
    pa = (void *) (phys_frame + (va & 0x00000FFF));
    unmap_virtual_page (virt_ptbr);
  }
  unmap_virtual_page (virt_pdbr);

  return pa;
}


/* ************************************************** */

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

/* precondition: entry is valid and dir has valid VA
 * postcondition: tbl->table_pa and starting_va are valid */
bool
map_pgdir_entry (/* in */ pgdir_t dir,
                 /* in */ bool from_top,
                 /* in */ pgdir_entry_t entry,
                 /* out */ pgtbl_t *tbl)
{
  uint i;
  pgdir_entry_t *dir_va = dir.dir_va;

  tbl->table_pa = FRAMENUM_TO_FRAME (entry.table_framenum);
  tbl->table_va = NULL;

  if (from_top) {
    for (i=PGDIR_NUM_ENTRIES; i>=0; i--) {
      if (!dir_va[i].flags.present) { /* Free entry */
        dir_va[i] = entry;
        tbl->starting_va = (uint8 *) (i << 22);
        return TRUE;
      }
    } 
  } else {
    for (i=0; i<PGDIR_NUM_ENTRIES; i++) {
      if (!dir_va[i].flags.present) { /* Free entry */
        dir_va[i] = entry;
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

/* precondition: dir PA and VA are valid, va is aligned */
/* postcondition: returned frame is aligned */
/* failure: -1 */
frame_t
pgdir_get_frame (pgdir_t dir, void *va)
{
  linear_address_t la; la.raw = (u32) va;
  pgdir_entry_t *entry = &dir.dir_va[la.pgdir_i];

  if (!entry->flags.present)
    goto abort;

  if (entry->flags.page_size) {
    /* big page */
    return (frame_t) BIGFRAMENUM_TO_FRAME (entry->framenum);
  } else {
    /* regular page */
    pgtbl_entry_t *table = _prim_map_virtual_page (FRAMENUM_TO_FRAME (entry->table_framenum) | 3);
    if (table == NULL)
      goto abort;
    if (!table[la.pgtbl_i].flags.present) {
      _prim_unmap_virtual_page (table);
      goto abort;
    } else {
      frame_t frame = FRAMENUM_TO_FRAME (table[la.pgtbl_i].framenum);
      _prim_unmap_virtual_page (table);
      return frame;
    }
  }

 abort:
  return (frame_t) -1;
}

/* Obtains the physical address of a virtual address in the given
 * page directory. */
/* precondition: dir PA and VA are valid */
/* failure: -1 */
phys_addr_t
pgdir_get_phys_addr (pgdir_t dir, void *va)
{
  linear_address_t la; la.raw = (u32) va;
  frame_t frame = pgdir_get_frame (dir, (void *) (((u32) va) & (~(PAGE_SIZE-1))));
  if (frame == -1)
    return -1;
  return frame + la.offset;
}

/* Clone the contents of a page table, copying data. */
/* failure result is (-1, 0) */
/* precondition: all of tbl is valid */
/* postcondition: new physical and virtual address are valid in returned pgtbl */
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
      frame_t new_frame = alloc_phys_frame ();
      frame_t old_frame = FRAMENUM_TO_FRAME (tbl.table_va[i].framenum);

      /* temporarily map frames */
      void *old_page_tmp = _prim_map_virtual_page (old_frame | 3);
      if (old_page_tmp == NULL)
        goto abort_tbl_va;
      void *new_page_tmp = _prim_map_virtual_page (new_frame | 3);
      if (new_page_tmp == NULL) {
        _prim_unmap_virtual_page (old_page_tmp);
        goto abort_tbl_va;
      }

      /* copy contents of old frame to new frame */
      memcpy (new_page_tmp, old_page_tmp, PAGE_SIZE);

      /* setup new page table entry */
      new_tbl.table_va[i].flags.raw = tbl.table_va[i].flags.raw;
      new_tbl.table_va[i].framenum = FRAME_TO_FRAMENUM (new_frame);

      _prim_unmap_virtual_page (old_page_tmp);
      _prim_unmap_virtual_page (new_page_tmp);
    }
  }

  return new_tbl;

 abort_tbl_va:
  _prim_unmap_virtual_page (new_tbl.table_va);
 abort_tbl_pa:
  free_phys_frame (new_tbl.table_pa);
 abort:
  new_tbl.table_pa = -1;
  new_tbl.table_va = NULL;
  return new_tbl;
}

/* Clone an entire address space making copies of data where
 * appropriate (e.g. userspace and kernel stack). */

/* precondition: dir has valid VA, PA 
 * postcondition: return has valid VA, PA
 * failure result is (-1, 0) */
pgdir_t
clone_page_directory (pgdir_t dir)
{
  frame_t new_pgd_pa;
  pgdir_t new_dir;
  uint i;

  new_pgd_pa = alloc_phys_frame ();

  if (new_pgd_pa == -1)
    goto abort;

  new_dir.dir_pa = new_pgd_pa;
  new_dir.dir_va = _prim_map_virtual_page (new_pgd_pa | 3);
  if (new_dir.dir_va == NULL)
    goto abort_pgd_pa;

  memset (new_dir.dir_va, 0, PGDIR_NUM_ENTRIES * sizeof (pgdir_entry_t));

  /* run through dir and make copies of tables */
  for (i=0; i<PGDIR_NUM_ENTRIES; i++) {
    if (dir.dir_va[i].flags.present) {
      if (i >= PGDIR_KERNEL_BEGIN && i != PGDIR_KERNEL_STACK) {
        /* shared kernel-space */
        new_dir.dir_va[i].raw = dir.dir_va[i].raw;
      } else if (dir.dir_va[i].flags.page_size) {
        /* clone 4 MiB page */
        //panic ("userspace 4 MiB pages not supported");
        new_dir.dir_va[i].raw = (dir.dir_va[i].framenum << 22) + 0x83;
        //new_dir.dir_va[i].raw = dir.dir_va[i].raw;
        //com1_printf ("Copy frame 0x%X: 0x%X\n", i, new_dir.dir_va[i].raw);
      } else {
        /* clone a page table */
        pgtbl_t tbl, new_tbl;

        /* setup a pgtbl struct with physical and virtual addresses of
         * the existing page table */
        tbl.table_pa = FRAMENUM_TO_FRAME (dir.dir_va[i].table_framenum);
        tbl.table_va = _prim_map_virtual_page (tbl.table_pa | 3);
        if (tbl.table_va == NULL)
          goto abort_pgd_va;
        tbl.starting_va = (uint8 *) (i << 22);

        new_tbl = clone_page_table (tbl);

        _prim_unmap_virtual_page (tbl.table_va);

        if (new_tbl.table_pa == -1)
          goto abort_pgd_va;

        _prim_unmap_virtual_page (new_tbl.table_va);

        /* setup new directory entry with flags and new table address */
        new_dir.dir_va[i].flags = dir.dir_va[i].flags;
        new_dir.dir_va[i].table_framenum = FRAME_TO_FRAMENUM (new_tbl.table_pa);
      }
    }
  }

  return new_dir;

 abort_pgd_va:
  _prim_unmap_virtual_page (new_dir.dir_va);
 abort_pgd_pa:
  free_phys_frame (new_pgd_pa);
 abort:
  new_dir.dir_va = NULL;
  new_dir.dir_pa = -1;
  return new_dir;
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
