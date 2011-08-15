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

#ifndef _SHM_H_
#define _SHM_H_

#include <types.h>
#include <kernel.h>

/* The number of allocatable locks for shared drivers */
/* The number is counted in groups of 32 locks */
#define DRV_LOCK_INDEX          0x1
/* DRV_LOCK_INDEX * 32 is the actual # of locks */
#define NUM_DRV_LOCKS           (DRV_LOCK_INDEX << 5)
#define PHYS_SHARED_MEM_HIGH    0xC0000000
#define SHARED_MEM_SIZE         0x00400000
/* SHARED_MEM_INDEX_MAX 32-bit integers bitmap for SHARED_MEM_SIZE of memory */
#define SHARED_MEM_INDEX_MAX    (SHARED_MEM_SIZE >> 17)
/* Last page in shared memory area reserved for meta info and locks, etc */
#define SHARED_MEM_INFO_PAGE    (PHYS_SHARED_MEM_HIGH - 0x1000)
#define SHM_MAGIC               0xCAFEBABE

#define SHM_MAX_SCREEN          0x08

typedef struct _cursor {
  int x;
  int y;
} cursor_t;

typedef struct _display_t {
  /* The physical address of virtual screen buffer */
  char * screen[SHM_MAX_SCREEN];
  /* Keep track of shell cursor */
  cursor_t cursor[SHM_MAX_SCREEN];
  /* Current active output */
  uint32 cur_screen;
} display_t;

typedef struct _shm_info {
  uint32 magic;
  spinlock shm_lock;
  spinlock logger_lock;
  spinlock global_lock;
  spinlock driver_lock[NUM_DRV_LOCKS];
  uint32 driver_lock_table[DRV_LOCK_INDEX];
  uint32 shm_table[SHARED_MEM_INDEX_MAX];
  display_t virtual_display;
  uint32 num_sandbox;
} shm_info;

/*
 * Bitmap operations for physical shared memory bitmap.
 */
#define SHM_BITMAP_SET(table,index)                                                 \
    ((table)[((index) - ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12))>>5] |=    \
    (1 << (((index) - ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12)) & 31)))

#define SHM_BITMAP_CLR(table,index)                                                 \
    ((table)[((index) - ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12))>>5] &=    \
    ~(1 << (((index) - ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12)) & 31)))

#define SHM_BITMAP_TST(table,index)                                                 \
    ((table)[((index) - ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12))>>5] &     \
    (1 << (((index) - ((PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE) >> 12)) & 31)))

extern shm_info *shm;
extern bool shm_initialized;
extern bool shm_screen_initialized;
extern bool shm_screen_first;
extern char * shm_screen;
extern void shm_init (uint32);
extern uint32 shm_alloc_phys_frame (void);
extern void shm_free_phys_frame (uint32);
extern spinlock * shm_alloc_drv_lock (void);
extern void shm_free_drv_lock (spinlock *);

#endif

