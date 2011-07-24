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

#define PHYS_SHARED_MEM_HIGH    0xC0000000
#define SHARED_MEM_SIZE         0x00400000
/* SHARED_MEM_INDEX_MAX 32-bit integers bitmap for SHARED_MEM_SIZE of memory */
#define SHARED_MEM_INDEX_MAX    (SHARED_MEM_SIZE >> 17)
/* Last page in shared memory area reserved for meta info and locks, etc */
#define SHARED_MEM_INFO_PAGE    (PHYS_SHARED_MEM_HIGH - 0x1000)

typedef struct _shm_info {
  uint32 magic;
  spinlock shm_lock;
  spinlock logger_lock;
  spinlock global_lock;
  uint32 shm_table[SHARED_MEM_INDEX_MAX];
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
extern void shm_init (uint32);
extern uint32 shm_alloc_phys_frame (void);
extern void shm_free_phys_frame (uint32);

#endif

