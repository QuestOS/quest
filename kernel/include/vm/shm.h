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

#ifndef _SHM_H_
#define _SHM_H_

#include <types.h>
#include <kernel.h>
#include <vm/ept.h>

/* The number of allocatable locks for shared drivers */
/* The number is counted in groups of 32 locks */
#define DRV_LOCK_INDEX          0x1
/* DRV_LOCK_INDEX * 32 is the actual # of locks */
#define NUM_DRV_LOCKS           (DRV_LOCK_INDEX << 5)
/* Change this to 0xC0000000 if possible. */
#define PHYS_SHARED_MEM_HIGH    0xA0000000
#define SHARED_MEM_SIZE         0x00800000
/* SHARED_MEM_INDEX_MAX 32-bit integers bitmap for SHARED_MEM_SIZE of memory */
#define SHARED_MEM_INDEX_MAX    (SHARED_MEM_SIZE >> 17)
/* Last page in shared memory area reserved for meta info and locks, etc */
#define SHARED_MEM_INFO_PAGE    (PHYS_SHARED_MEM_HIGH - 0x1000)
#define SHM_MAGIC               0xCAFEBABE

#define SHM_MAX_SCREEN          0x08
#define SHM_MAX_SANDBOX         0x08

/* The start (high, grows down) physical address of private channels b/w sandboxes */
#define PHYS_PRIV_CHANNEL_HIGH  (PHYS_SHARED_MEM_HIGH - ((1 + SHM_MAX_SCREEN) << 12))
/* Number of total private channels needed */
#define NUM_PRIV_CHANNELS       ((SHM_MAX_SANDBOX * (SHM_MAX_SANDBOX - 1)) >> 1)
/* Index of channel between x and y (x and y cannot be equal!) */
/* x and y starts from 0 (core id) and CHANNEL_INDEX start from *1* (not 0) */
/* Given x < y, index = y + ((2n -3)x - x^2)/2 */
#define CHANNEL_INDEX(x, y)                                         \
  ((((x < y) ? (((SHM_MAX_SANDBOX << 1) - 3) * x - x * x) :         \
               (((SHM_MAX_SANDBOX << 1) - 3) * y - y * y)) >> 1) +  \
  ((x < y) ? y : x))
/* Physical address of channel between x and y (x and y cannot be equal!) */
#define CHANNEL_ADDR(x, y)  (PHYS_PRIV_CHANNEL_HIGH - (CHANNEL_INDEX(x, y) << 12))

/* These are the macro used by user space programs in the vshm_map
   syscall.  These match with the EPT equivalents but that is just
   because it works with x86, these should be separate for future
   architectures that might have different bitmaps */
#define VSHM_NO_ACCESS        0x0
#define VSHM_READ_ACCESS      0x1
#define VSHM_WRITE_ACCESS     0x2
#define VSHM_EXEC_ACCESS      0x4
#define VSHM_ALL_ACCESS       0x7

#define VSHM_CREATE           0x80000000

#ifndef __ASSEMBLER__

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
  /* A flag shows whether this structure is properly initialized */
  uint32 magic;
  spinlock shm_lock;
  spinlock logger_lock;
  spinlock global_lock;
  /* Allocatable shared driver locks */
  spinlock driver_lock[NUM_DRV_LOCKS];
  uint32 driver_lock_table[DRV_LOCK_INDEX];
  /* Physical memory bitmap for shared memory */
  uint32 shm_table[SHARED_MEM_INDEX_MAX];
  display_t virtual_display;
  uint32 num_sandbox;
  /* The migration request queue storing the physical address of its first quest_tss */
  void * migration_queue[SHM_MAX_SANDBOX];
  /* Time stamp counter value of other sandbox used to fix scheduling time */
  uint64 remote_tsc[SHM_MAX_SANDBOX];
  /* If TRUE, local tsc is leading. If FALSE, local tsc is falling behind */
  bool remote_tsc_diff[SHM_MAX_SANDBOX];
  /* If TRUE, local EPT tables have been setup. */
  bool ept_initialized[SHM_MAX_SANDBOX];
  /* Used to muffle network output of a certain sandbox to implement "hot" backup */
  bool network_transmit_enabled[SHM_MAX_SANDBOX];
  bool bsp_booted;
} shm_info;

#define POOL_SIZE_IN_PAGES 10
#define NUM_POOLS_PER_SANDBOX 4

typedef struct {
  bool new_pool;
  uint vshm_key;
  uint pool_id;
  uint permissions;
  phys_addr_t start_phys_addr;
  uint bitmap;
} new_shared_memory_arena_msg_t;

CASSERT(POOL_SIZE_IN_PAGES <= (sizeof(uint) * 8), vshm_pool_size)

typedef enum {
  ISBM_NO_MESSAGE = 0,
  ISBM_NEW_SHARED_MEMORY_ARENA,
  ISBM_TEST
} isb_msg_type_t;

bool send_intersandbox_msg(isb_msg_type_t msg_type, uint target_sandbox,
			   void* msg, size_t size);

static inline void
initialise_new_shared_memory_arena_msg(new_shared_memory_arena_msg_t* msg,
				       bool new_pool, uint vshm_key,
				       uint pool_id, uint permissions,
				       phys_addr_t start_phys_addr,
				       uint bitmap)
{
  msg->new_pool = new_pool;
  msg->vshm_key = vshm_key;
  msg->pool_id = pool_id;
  msg->permissions = permissions;
  msg->start_phys_addr = start_phys_addr;
  msg->bitmap = bitmap;
}

typedef struct {
  phys_addr_t start_addr;
  uint permissions;
  uint keys_map[POOL_SIZE_IN_PAGES];
  uint used_pages;
  bool created_locally;
} shm_pool_t;

typedef struct {
  shm_pool_t pools[SHM_MAX_SANDBOX][NUM_POOLS_PER_SANDBOX];
  void* priv_write_regions[SHM_MAX_SANDBOX];
  void* priv_read_regions[SHM_MAX_SANDBOX];
} shm_comm_t;

extern shm_comm_t shm_comm;

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

#define shm_alloc_phys_frame() shm_alloc_phys_frame_perm (EPT_ALL_ACCESS)
#define shm_alloc_phys_frames(c) shm_alloc_phys_frames_perm (c, EPT_ALL_ACCESS)

extern shm_info *shm;
extern bool shm_initialized;
extern bool shm_screen_initialized;
extern bool shm_screen_first;
extern char * shm_screen;
extern void shm_init (uint32);

/* Initialise shared memory pools to be blank */
void init_shared_memory_pools(void);

/*
 * Allocate one frame of shared memory for calling sandbox. The permission of this
 * frame (set in EPT) will be set according to the permission given for local sandbox.
 * Allowed permissions are any combination of: EPT_NO_ACCESS, EPT_READ_ACCESS,
 * EPT_WRITE_ACCESS and EPT_ALL_ACCESS.
 */
extern uint32 shm_alloc_phys_frame_perm (uint8);
/*
 * Same as shm_alloc_phys_frame_perm except this function allows the calling sandbox
 * to allocate multiple contiguous frames.
 */
extern uint32 shm_alloc_phys_frames_perm (uint32, uint8);
extern void shm_free_phys_frame (uint32);
extern void shm_free_phys_frames (uint32, uint32);
/*
 * This function sets the permission of a given frame in shared memory. No allocation
 * is done for the caller. Caller must use shm_alloc_phys_frame/frames_perm before
 * changing permissions.
 */
extern void shm_set_ept_permission (uint32, uint32, uint8);
extern spinlock * shm_alloc_drv_lock (void);
extern void shm_free_drv_lock (spinlock *);

#endif /* __ASSEMBLER__ */

#endif

