#include "arch/i386.h"
#include "kernel.h"
#include "types.h"

/* Declare space for bitmap (physical) memory usage table.
 * PHYS_INDEX_MAX entries of 32-bit integers each for a 4K page => 4GB
 * memory limit when PHYS_INDEX_MAX=32768
 */
uint32 mm_table[PHYS_INDEX_MAX] __attribute__ ((aligned (4096)));
uint32 mm_limit;                /* Actual physical page limit */

/* Find free page in mm_table 
 *
 * Returns physical address rather than virtual, since we we don't
 * want user-level pages mapped into kernel page tables in all cases
 */
uint32
alloc_phys_frame (void)
{

  int i;

  for (i = 0; i < mm_limit; i++)
    if (BITMAP_TST (mm_table, i)) {     /* Free page */
      BITMAP_CLR (mm_table, i);
      return (i << 12);         /* physical byte address of free page/frame */
    }

  return -1;                    /* Error -- no free page? */
}

uint32
alloc_phys_frames (uint32 count)
{

  int i, j;

  for (i = 0; i < mm_limit - count + 1; i++) {
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
