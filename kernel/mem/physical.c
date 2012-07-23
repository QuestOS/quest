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

#include "mem/physical.h"
#include "kernel.h"

/* Declare space for bitmap (physical) memory usage table.
 * PHYS_INDEX_MAX entries of 32-bit integers each for a 4K page => 4GB
 * memory limit when PHYS_INDEX_MAX=32768
 */
uint32 mm_table[PHYS_INDEX_MAX] __attribute__ ((aligned (4096)));
uint32 mm_limit;                /* Actual physical page limit */
uint32 mm_begin = 0;

/* Find free page in mm_table 
 *
 * Returns physical address rather than virtual, since we we don't
 * want user-level pages mapped into kernel page tables in all cases
 */
uint32
alloc_phys_frame (void)
{

  int i;

  for (i = mm_begin; i < mm_limit; i++)
    if (BITMAP_TST (mm_table, i)) {     /* Free page */
      BITMAP_CLR (mm_table, i);
      return (i << 12);         /* physical byte address of free page/frame */
    }

  return -1;                    /* Error -- no free page? */
}

/*
 * Allocate free physical page from high address.
 */
uint32
alloc_phys_frame_high (void)
{

  int i;

  for (i = mm_limit - 1; i >= mm_begin; i--) {
    if (BITMAP_TST (mm_table, i)) {
      BITMAP_CLR (mm_table, i);
      return (i << 12);
    }
  }

  return -1;
}

uint32
alloc_phys_frames (uint32 count)
{

  int i, j;

  for (i = mm_begin; i < mm_limit - count + 1; i++) {
    for (j = 0; j < count; j++) {
      if (!BITMAP_TST (mm_table, i + j)) {      /* Is not free page? */
        i = i + j;
        goto keep_searching;
      }
    }
    /* found window: */
    for (j = 0; j < count; j++) {
      BITMAP_CLR (mm_table, i + j);
    }
    return (i << 12);           /* physical byte address of free frames */
  keep_searching:
    ;
  }
  return -1;                    /* Error -- no free page? */
}

void
free_phys_frame (uint32 frame)
{
  BITMAP_SET (mm_table, frame >> 12);
}

void
free_phys_frames (uint32 frame, uint32 count)
{
  int i;
  frame >>= 12;
  for (i = 0; i < count; i++)
    BITMAP_SET (mm_table, frame + i);
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
