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

#include "vm/shm.h"
#include "vm/ept.h"
#include "vm/spow2.h"
#include "kernel.h"
#include "mem/virtual.h"
#include "util/printf.h"
#include "smp/apic.h"

#define DEBUG_SHM    1
#if DEBUG_SHM > 0
#define DLOG(fmt,...) DLOG_PREFIX("shm",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

shm_info *shm = NULL;

/* Per-Sandbox shared memory initialization flag.
 * If this is true, the global lock can be used for
 * inter-sandbox synchronization. */
bool shm_initialized = FALSE;

/* Per-Sandbox virtual screen buffer status */
bool shm_screen_initialized = FALSE;
/* Per-Sandbox virtual screen buffer */
char * shm_screen = NULL;
/* An ugly hack... So that we know when to reset the coordinates */
bool shm_screen_first = FALSE;

uint32 shm_limit, shm_begin;

/*
 * Initialize per-sandbox virtual screen buffer for separated screen
 * output. This is called in shm_init after the shared memory area is
 * initialized.
 */
static void
shm_screen_init (uint32 cpu)
{
  if (cpu >= SHM_MAX_SCREEN) {
    logger_printf ("CPU %d: Virtual screen not initialized.\n", cpu);
    return;
  }

  shm->virtual_display.screen[cpu] = (char *) shm_alloc_phys_frame ();
  shm_screen = map_virtual_page ((uint32) (shm->virtual_display.screen[cpu]) | 3);
  memset (shm_screen, 0, 0x1000);

  /* Backup screen buffer for Bootstrap Processor */
  if (cpu == 0) {
    memcpy (shm_screen, pchVideo, 0x1000);
  }

  logger_printf ("CPU %d: Virtual screen initialized.\n", cpu);
  shm_screen_initialized = TRUE;
  shm_screen_first = TRUE;
  shm->virtual_display.cursor[cpu].x = -1;
  shm->virtual_display.cursor[cpu].y = -1;
}

/*
 * shm_init should be called sequentially by each sandbox kernel.
 * Also, this function should be called after the kernel fork unless
 * called in bootstrap processor. (which is in vmx_init_mem function).
 */
void
shm_init (uint32 cpu)
{
  int i;

  if (sizeof (shm_info) > PHYS_PAGE_SIZE) {
    logger_printf ("Shared memory shm_info structure is larger than 1 page!\n");
    logger_printf ("Shared memory initialization failed.\n");
    return;
  }

  shm_limit = PHYS_SHARED_MEM_HIGH >> 12;
  shm_begin = (PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12;

  /* Unmap the shared info page mapped for Bootstrap Processor.
   * It will be remapped in the new kernel space. */
  if (shm != NULL) {
    unmap_virtual_page (shm);
  }

  shm = (shm_info*) map_virtual_page (SHARED_MEM_INFO_PAGE | 3);

  /* Initialization done only once on Bootstrap Proc */
  if (cpu == 0) {
    /* Initialize the shared global spin locks */
    spinlock_init (&(shm->shm_lock));
    spinlock_init (&(shm->logger_lock));
    spinlock_init (&(shm->global_lock));

    for (i = 0; i < NUM_DRV_LOCKS; i++) {
      spinlock_init (&(shm->driver_lock[i]));
      BITMAP_SET (shm->driver_lock_table, i);
    }

    /* Mark all the pages in shared area as available */
    for (i = shm_begin; i < shm_limit; i++) {
      SHM_BITMAP_SET (shm->shm_table, i);
    }
    /* Mark the info page as occupied */
    SHM_BITMAP_CLR (shm->shm_table, SHARED_MEM_INFO_PAGE >> 12);
    /* Set the magic to notify others that this area is initialized */
    shm->magic = SHM_MAGIC;
    shm->num_sandbox = 0;
    shm->virtual_display.cur_screen = 0;
    shm->bsp_booted = FALSE;
    DLOG ("Shared memory system initialized:");
    DLOG ("  Total Allocatable Pages = %d", (SHARED_MEM_SIZE >> 12) - 1);
  }

  spinlock_lock (&(shm->shm_lock));
  shm->num_sandbox++;
  spinlock_unlock (&(shm->shm_lock));

  if (shm->magic != SHM_MAGIC) {
    logger_printf ("shm_info structure is not initialized.\n");
    return;
  }

  shm_initialized = TRUE;

  shm_screen_init (cpu);

  if (cpu == 0) {
    shm_pow2_init ();
  }
}

spinlock*
shm_alloc_drv_lock (void)
{
  int i;

  if (!shm_initialized) {
    logger_printf ("shm_alloc_drv_lock: Shared memory is not initialized!\n");
    return NULL;
  } else {
    spinlock_lock (&(shm->shm_lock));
    for (i = 0; i < NUM_DRV_LOCKS; i++) {
      if (BITMAP_TST (shm->driver_lock_table, i)) {
        BITMAP_CLR (shm->driver_lock_table, i);
        spinlock_unlock (&(shm->shm_lock));
        return &(shm->driver_lock[i]);
      }
    }
    spinlock_unlock (&(shm->shm_lock));
  }

  return NULL;
}

void
shm_free_drv_lock (spinlock* lock)
{
  int i;

  if (!shm_initialized) {
    logger_printf ("shm_free_drv_lock: Shared memory is not initialized!\n");
    return;
  } else {
    spinlock_lock (&(shm->shm_lock));
    for (i = 0; i < NUM_DRV_LOCKS; i++) {
      /* Compare the virtual address should be enough here */
      if ((uint32)lock == (uint32)(&(shm->driver_lock[i]))) {
        BITMAP_SET (shm->driver_lock_table, i);
        spinlock_unlock (&(shm->shm_lock));
        return;
      }
    }
    spinlock_unlock (&(shm->shm_lock));
    logger_printf ("shm_free_drv_lock: Lock not found!\n");
  }
}

uint32
shm_alloc_phys_frame (void)
{
  int i;

  if (!shm_initialized) {
    logger_printf ("shm_alloc_phys_frame: Shared memory is not initialized!\n");
    return -1;
  } else {
    spinlock_lock (&(shm->shm_lock));
    for (i = shm_begin; i < shm_limit; i++)
      if (SHM_BITMAP_TST (shm->shm_table, i)) {
        SHM_BITMAP_CLR (shm->shm_table, i);
        spinlock_unlock (&(shm->shm_lock));
        return (i << 12);
      }
    spinlock_unlock (&(shm->shm_lock));
  }

  return -1;
}

uint32
shm_alloc_phys_frames (uint32 count)
{
  int i, j;

  if (!shm_initialized) {
    logger_printf ("shm_alloc_phys_frames: Shared memory is not initialized!\n");
    return -1;
  } else {
    spinlock_lock (&(shm->shm_lock));

    for (i = shm_begin; i < shm_limit - count + 1; i++) {
      for (j = 0; j < count; j++) {
        if (!SHM_BITMAP_TST (shm->shm_table, i + j)) {      /* Is not free page? */
          i = i + j;
          goto keep_searching;
        }
      }
      /* found window: */
      for (j = 0; j < count; j++) {
        SHM_BITMAP_CLR (shm->shm_table, i + j);
      }
      spinlock_unlock (&(shm->shm_lock));
      return (i << 12);           /* physical byte address of free frames */
      keep_searching:
      ;
    }

    spinlock_unlock (&(shm->shm_lock));
  }

  return -1;
}

void
shm_free_phys_frame (uint32 frame)
{
  if (!shm_initialized) {
    logger_printf ("shm_free_phys_frame: Shared memory is not initialized!\n");
    return;
  } else {
    spinlock_lock (&(shm->shm_lock));
    SHM_BITMAP_SET (shm->shm_table, frame >> 12);
    spinlock_unlock (&(shm->shm_lock));
  }
}

void
shm_free_phys_frames (uint32 frame, uint32 count)
{
  int i;

  frame >>= 12;

  if (!shm_initialized) {
    logger_printf ("shm_free_phys_frames: Shared memory is not initialized!\n");
    return;
  } else {
    spinlock_lock (&(shm->shm_lock));
    for (i = 0; i < count; i++)
      SHM_BITMAP_SET (shm->shm_table, frame + i);
    spinlock_unlock (&(shm->shm_lock));
  }
}

/* vi: set et sw=2 sts=2: */
