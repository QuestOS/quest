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

#include "kernel.h"
#include "mem/mem.h"
#include "vm/shm.h"
#include "smp/spinlock.h"

/*
 * This is a copy of power-of-2 memory allocator from mem/pow2.c.
 * This is the version for SeQuest and allocates memory from the
 * shared memory region so that all the sandboxes can use.
 *
 * All the shared drivers and other shared system components should
 * use this allocator for memory allocation.
 */

/* power-of-2 memory allocator works with blocks in increasing sizes
 * in powers of 2, from 2^SHM_POW2_MIN_POW to 2^SHM_POW2_MAX_POW */

#define SHM_POW2_MIN_POW 5
#define SHM_POW2_MAX_POW 16
/* NB: the value of SHM_POW2_MAX_POW has to be smaller than 2^SHM_POW2_MIN_POW
 * because it needs to be able to fit within the SHM_POW2_MASK_POW.  Also
 * keep in mind it needs to be able to find a contiguous virtual
 * address range every time it allocates one of these large
 * blocks. */

#define SHM_POW2_MIN_SIZE (1<<SHM_POW2_MIN_POW)
#define SHM_POW2_MAX_SIZE (1<<SHM_POW2_MAX_POW)

/* Length of the central header table */
#define SHM_POW2_TABLE_LEN ((SHM_POW2_MAX_POW - SHM_POW2_MIN_POW)+1)

/* How many frames to allocate for a max-size block: */
#define SHM_POW2_MAX_POW_FRAMES (1 << (SHM_POW2_MAX_POW - 12))

/* Mask the index from a descriptor: */
#define SHM_POW2_MASK_POW ((1<<SHM_POW2_MIN_POW)-1)

struct _SHM_POW2_HEADER
{
  struct _SHM_POW2_HEADER *next;
  uint32 count;
  uint8 *ptrs[0];
} PACKED;
typedef struct _SHM_POW2_HEADER SHM_POW2_HEADER;
#define POW2_MAX_COUNT ((0x1000 - sizeof(SHM_POW2_HEADER)) >> 2)

static SHM_POW2_HEADER *shm_pow2_table[SHM_POW2_TABLE_LEN];

static uint32 *shm_pow2_used_table; /* array of descriptors:
                                 * pointer | index */
static uint32 shm_pow2_used_count, shm_pow2_used_table_pages;

static spinlock * shm_pow2_lock = NULL;

static void
shm_pow2_add_free_block (uint8 * ptr, uint8 index)
{
  SHM_POW2_HEADER *hdr = shm_pow2_table[index - SHM_POW2_MIN_POW];

  for (;;) {
    if (hdr->count < POW2_MAX_COUNT) {
      /* There is room in the current header's block */
      hdr->ptrs[hdr->count++] = ptr;
      return;
    } else {
      /* There is not enough room */
      if (hdr->next) {
        /* Chase the next pointer */
        hdr = hdr->next;
      } else {
        /* End of the list -- make a new header */
        hdr->next = map_virtual_page (shm_alloc_phys_frame () | 3);
        memset (hdr->next, 0, 0x1000);
        hdr = hdr->next;
      }
    }
  }
}

/* Trying to avoid making the frame-size of shm_pow2_get_free_block any
 * larger than it is -- because of recursion -- and this is used in a
 * re-entrantly safe fashion. */
static uint32 shm_pow2_tmp_phys_frames[SHM_POW2_MAX_POW_FRAMES];

static uint8 *
shm_pow2_get_free_block (uint8 index)
{
  SHM_POW2_HEADER *hdr = shm_pow2_table[index - SHM_POW2_MIN_POW], *prev = NULL;
  uint8 *ptr;

  for (;;) {
    if (hdr->count == 0) {
      /* no free blocks -- get one */
      /* hdr->count can only be 0 for first in chain */
      if (index < SHM_POW2_MAX_POW) {
        uint8 *ptr1, *ptr2;

        /* Recursive call: 32 bytes per frame, 11 calls deep, 352
         * bytes of stack worst-case.  Should be acceptable, for
         * now. */
        ptr1 = shm_pow2_get_free_block (index + 1);
        ptr2 = ptr1 + (1 << (index));

        shm_pow2_add_free_block (ptr1, index);

        /* It is safe to return ptr2 here because if there were no
         * free blocks upon entering this section of code, then there
         * is no need to check 'prev' and possibly zero its next
         * pointer. */
        return ptr2;
      } else {
        /* grab new pages */
        int i;
        for (i = 0; i < SHM_POW2_MAX_POW_FRAMES; i++)
          shm_pow2_tmp_phys_frames[i] = shm_alloc_phys_frame () | 3;
        return map_virtual_pages (shm_pow2_tmp_phys_frames, SHM_POW2_MAX_POW_FRAMES);
      }
    } else if (hdr->count < POW2_MAX_COUNT || hdr->next == NULL) {
      /* There are free blocks ready to go */
      ptr = hdr->ptrs[--hdr->count];
      if (prev && hdr->count == 0) {
        /* We followed a next pointer to get here. */
        uint32 frame = (uint32) get_phys_addr ((void *) hdr);
        prev->next = NULL;
        unmap_virtual_page ((void *) hdr);
        shm_free_phys_frame (frame);
      }
      return ptr;
    } else {
      /* hdr->count == POW2_MAX_COUNT && hdr->next != NULL */
      /* chase next pointer and try to use that header instead */
      prev = hdr;
      hdr = hdr->next;
    }
  }
}

static void
shm_pow2_insert_used_table (uint8 * ptr, uint8 index)
{
  if (shm_pow2_used_count >= (shm_pow2_used_table_pages * 0x400)) {
    uint32 count = shm_pow2_used_table_pages + 1;
    uint32 frames = shm_alloc_phys_frames (count), old_frames;
    void *virt = map_contiguous_virtual_pages (frames | 3, count);
    memcpy (virt, shm_pow2_used_table, sizeof (uint32) * shm_pow2_used_count);
    old_frames = (uint32) get_phys_addr (shm_pow2_used_table);
    unmap_virtual_pages ((void *) shm_pow2_used_table, shm_pow2_used_table_pages);
    shm_free_phys_frames (old_frames, shm_pow2_used_table_pages);
    shm_pow2_used_table = (uint32 *) virt;
    shm_pow2_used_table_pages = count;
  }
  shm_pow2_used_table[shm_pow2_used_count++] = (uint32) ptr | (uint32) index;
}

static int
shm_pow2_remove_used_table (uint8 * ptr, uint8 * index)
{
  int i;
  for (i = 0; i < shm_pow2_used_count; i++) {
    if ((uint32) ptr == (shm_pow2_used_table[i] & (~SHM_POW2_MASK_POW))) {
      /* found it */
      *index = shm_pow2_used_table[i] & SHM_POW2_MASK_POW;
      shm_pow2_used_table[i] = shm_pow2_used_table[--shm_pow2_used_count];
      shm_pow2_used_table[shm_pow2_used_count] = 0;
      return 0;
    }
  }
  return -1;
}

static uint8
pow2_compute_index (uint32 size)
{
  int i;
  if (size <= SHM_POW2_MIN_SIZE)
    return SHM_POW2_MIN_POW;
  else if (size >= SHM_POW2_MAX_SIZE)
    return SHM_POW2_MAX_POW;
  else {
    size--;
    /* bit scan reverse -- find most significant set bit */
    asm volatile ("bsrl %1,%0":"=r" (i):"r" (size));
    return (i + 1);
  }
}

int
shm_pow2_alloc (uint32 size, uint8 ** ptr)
{
  uint8 index = pow2_compute_index (size);
  spinlock_lock (shm_pow2_lock);
  *ptr = shm_pow2_get_free_block (index);
  shm_pow2_insert_used_table (*ptr, index);
  spinlock_unlock (shm_pow2_lock);
  memset (*ptr, 0, size);
  return index;
}

void
shm_pow2_free (uint8 * ptr)
{
  uint8 index;
  spinlock_lock (shm_pow2_lock);
  if (shm_pow2_remove_used_table (ptr, &index) == 0) {
    shm_pow2_add_free_block (ptr, index);
  }
  spinlock_unlock (shm_pow2_lock);
}

void
shm_pow2_init (void)
{
  int i;
  uint32 phy_frame = 0;

  for (i = 0; i < SHM_POW2_TABLE_LEN; i++) {
    phy_frame = shm_alloc_phys_frame ();
    if (phy_frame == -1) {
      panic ("spow2: physical memory allocation failed!");
    }
    shm_pow2_table[i] = map_virtual_page (phy_frame | 3);
    memset (shm_pow2_table[i], 0, 0x1000);
  }

  phy_frame = shm_alloc_phys_frame ();
  if (phy_frame == -1) {
    panic ("spow2: used table physical memory allocation failed!");
  }
  shm_pow2_used_table = map_virtual_page (phy_frame | 3);
  memset (shm_pow2_used_table, 0, 0x1000);
  shm_pow2_used_count = 0;
  shm_pow2_used_table_pages = 1;
  shm_pow2_lock = shm_alloc_drv_lock ();
  spinlock_init (shm_pow2_lock);
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
