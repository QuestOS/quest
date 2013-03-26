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

#ifndef _SPOW2_H_
#define _SPOW2_H_
#include "types.h"
#include "vm/shm.h"

/* For some drivers like NIC, we need a relatively large pool of consistent
 * virtual memory space for shared data structures like TX and RX ring
 * descriptors. This means that the virtual addresses for them should be the
 * same for all the sandboxes in order to make the sharing more efficient.
 * And the size of the pool is bigger than our kernel virtual address space.
 * So, here, we have to do this hack of statically allocating an 12MB virtual
 * and physical memory space for this purpose.
 *
 * This pool is only used by the shared memory power-of-two allocator for
 * shared component memory management in Quest-V. We might need to think of
 * a better way of solving this problem in the future when we rewrite the
 * memory allocator.
 */

/* Size of the pool: 12MB by default */
#define SHARED_MEM_POOL_SIZE          0x00C00000

/* Starting physical address of this pool. Let's put it together with shared memory. */
/* See shm.h for detail about the location. */
#define PHY_SHARED_MEM_POOL_START   \
    (PHYS_SHARED_MEM_HIGH - SHARED_MEM_SIZE - SHARED_MEM_POOL_SIZE)

/* SHARED_MEM_POOL_INDEX_MAX 32-bit integers bitmap for SHARED_MEM_POOL_SIZE of memory */
#define SHARED_MEM_POOL_INDEX_MAX     (SHARED_MEM_POOL_SIZE >> 17)

#ifndef __ASSEMBLER__

#define SPOW2_BITMAP_SET(table,index)                                \
    ((table)[((index) - (PHY_SHARED_MEM_POOL_START >> 12))>>5] |=    \
    (1 << (((index) - (PHY_SHARED_MEM_POOL_START >> 12)) & 31)))

#define SPOW2_BITMAP_CLR(table,index)                                \
    ((table)[((index) - (PHY_SHARED_MEM_POOL_START >> 12))>>5] &=    \
    ~(1 << (((index) - (PHY_SHARED_MEM_POOL_START >> 12)) & 31)))

#define SPOW2_BITMAP_TST(table,index)                                \
    ((table)[((index) - (PHY_SHARED_MEM_POOL_START >> 12))>>5] &     \
    (1 << (((index) - (PHY_SHARED_MEM_POOL_START >> 12)) & 31)))

void shm_kmalloc_init (void);
void* shm_kmalloc(uint32_t size);
void shm_kfree(void* ptr);

#endif /* __ASSEMBLER__ */

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
