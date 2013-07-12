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

#include "vm/shm.h"
#include "vm/ept.h"
#include "vm/vmx.h"
#include "vm/spow2.h"
#include "kernel.h"
#include "mem/virtual.h"
#include "util/printf.h"
#include "smp/apic.h"
#include "sched/vcpu.h"
#include "sched/sched.h"

#define DEBUG_SHM    1
#if DEBUG_SHM > 0
#define DLOG(fmt,...) DLOG_PREFIX("shm",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

shm_info *shm = NULL;

shm_comm_t shm_comm;

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

  /* Pre-allocated virtual screen page per cpu */
  shm->virtual_display.screen[cpu] = (char *) (PHYS_SHARED_MEM_HIGH - 0x1000 * (cpu + 2));
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
    memset (shm, 0, 0x1000);
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
    /* Mark all virtual screen page as occupied */
    for (i = 0; i < SHM_MAX_SCREEN; i++) {
      SHM_BITMAP_CLR (shm->shm_table, (SHARED_MEM_INFO_PAGE >> 12) - i -1);
    }
    /* Mark all private channel pages as occupied */
    for (i = 0; i < NUM_PRIV_CHANNELS; i++) {
      SHM_BITMAP_CLR (shm->shm_table, (PHYS_PRIV_CHANNEL_HIGH >> 12) - i - 1);
    }
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

  shm->network_transmit_enabled[cpu] = FALSE;
  shm_initialized = TRUE;

  shm_screen_init (cpu);

  if (cpu == 0) {
    shm_kmalloc_init ();
  }
}


void process_isbm(isb_msg_type_t* msg_start, uint src_sandbox)
{
  switch(*msg_start) {
  case ISBM_NEW_SHARED_MEMORY_ARENA:
    {
      int i;
      new_shared_memory_arena_msg_t* nsma_msg = (new_shared_memory_arena_msg_t*)(msg_start+1);
      shm_pool_t *pool = &shm_comm.pools[src_sandbox][nsma_msg->pool_id];
      int count = popcount(nsma_msg->bitmap);
      if(nsma_msg->new_pool) {
        if(pool->start_addr) {
          /* -- EM -- The sending sandbox has tried to create a pool
             that already exists, this could happen if two sandbox
             tried to create pools to each other at the same time,
             assuming it won't happen for now */
          DLOG("Tried to create a new pool where one already exists");
          return;
        }
        if(count > POOL_SIZE_IN_PAGES) {
          /* -- EM -- Requested more pages that are available */
          return;
        }
        pool->start_addr = nsma_msg->start_phys_addr;
        pool->permissions = nsma_msg->permissions;
        pool->created_locally = FALSE;
        pool->used_pages = 0;
      }
      
      if(pool->used_pages + count > POOL_SIZE_IN_PAGES) {
        /* -- EM -- Requested more pages that are available */
        return;
      }

      for(i = 0; i < POOL_SIZE_IN_PAGES; ++i) {
        if( ((1 << i) & nsma_msg->bitmap) && pool->keys_map[i] ){
          /* -- EM -- Requested a page labeled to another arena */
          return;
        }
      }
    
      pool->used_pages += count;

      for(i = 0; i < POOL_SIZE_IN_PAGES; ++i) {
        if((1 << i) & nsma_msg->bitmap) {
          pool->keys_map[i] = nsma_msg->vshm_key;
        }
      }
    }
    break;
    
  case ISBM_TEST:
    {
      char* msg = (char*)(msg_start+1);
      msg[30] = 0;              /* Force the string to have an end */
      logger_printf("Got msg from sandbox %u: %s\n", src_sandbox, msg);
    }
    break;
    
  case ISBM_NO_MESSAGE:
    break;
  }
}

int isbm_communcation_thread_stack[1024] ALIGNED(0x1000);

void isbm_communcation_thread(void)
{
  #define ISBM_COMM_THREAD_SLEEP_INTERVAL 50000
  int i;
  uint pcpu_id = get_pcpu_id();
  isb_msg_type_t* reading_arena;

  unlock_kernel();
  sti();
  
  while(1) {
    
    for(i = 0; i < SHM_MAX_SANDBOX; ++i) {
      if(i != pcpu_id) {
        reading_arena = shm_comm.priv_read_regions[i];
        
	if(*reading_arena != ISBM_NO_MESSAGE) {
	  cli();
	  lock_kernel();
	  process_isbm(reading_arena, i);
          *reading_arena = ISBM_NO_MESSAGE;
	  unlock_kernel();
	  sti();
	}
      }
    }
    cli();
    lock_kernel();
    sched_usleep(ISBM_COMM_THREAD_SLEEP_INTERVAL);
    unlock_kernel();
    sti();
  }
}

void init_shared_memory_pools(void)
{
  int i;
  uint pcpu_id = get_pcpu_id();
  int vcpu_id = create_main_vcpu(10, 1000, NULL);
  if(vcpu_id < 0) {
    DLOG("Failed to create ipc hog vcpu");
    return;
  }

  
  
  memset(&shm_comm, 0, sizeof(shm_comm));

  for(i = 0; i < SHM_MAX_SANDBOX; ++i) {
    if(i != pcpu_id) {
      DLOG("Setting up channel between %d and %d", i, pcpu_id);
      DLOG("Phys addr = 0x%X", CHANNEL_ADDR(i, pcpu_id));
      DLOG("channel index = %d", CHANNEL_INDEX(i, pcpu_id));
      void* arena = map_virtual_page(CHANNEL_ADDR(i, pcpu_id) | 3);
      if(!arena) {
        com1_printf("Failed to initialise shared memory communication channels\n");
        panic("Failed to initialise shared memory communication channels");
      }
      
      memset(arena, 0, 0x1000);
      
      if(i < pcpu_id) {
        shm_comm.priv_write_regions[i] = arena;
        shm_comm.priv_read_regions[i] = ((char*)arena) + (0x1000 / 2);
      }
      else {
        shm_comm.priv_read_regions[i] = arena;
        shm_comm.priv_write_regions[i] = ((char*)arena) + (0x1000 / 2);
      }
    }
  }

  create_kernel_thread_vcpu_args ((u32) isbm_communcation_thread,
				  (u32) &isbm_communcation_thread_stack[1023],
                                  "ISBM Communication Thread", vcpu_id, TRUE, 0);

  return;
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

static struct _set_ept_param {
  uint32 phys_frame;
  uint32 count;
  uint8 perm;
} set_ept_param;

void
shm_set_ept_permission (uint32 phys_frame, uint32 count, uint8 perm)
{
  extern void * vm_exit_input_param;

  set_ept_param.phys_frame = phys_frame;
  set_ept_param.count = count;
  set_ept_param.perm = perm;
  vm_exit_input_param = (void *) &set_ept_param;
  vm_exit (VM_EXIT_REASON_SET_EPT);
}

uint32
shm_alloc_phys_frame_perm (uint8 perm)
{
  int i;

  if (!shm_initialized) {
    logger_printf ("shm_alloc_phys_frame_perm: Shared memory is not initialized!\n");
    return -1;
  } else {
    spinlock_lock (&(shm->shm_lock));
    for (i = shm_begin; i < shm_limit; i++)
      if (SHM_BITMAP_TST (shm->shm_table, i)) {
        SHM_BITMAP_CLR (shm->shm_table, i);
        spinlock_unlock (&(shm->shm_lock));
        if (shm->ept_initialized[get_pcpu_id ()]) {
          shm_set_ept_permission (i << 12, 1, perm);
        }
        return (i << 12);
      }
    spinlock_unlock (&(shm->shm_lock));
  }

  DLOG ("shm_alloc_phys_frame_perm failed!");
  return -1;
}

uint32
shm_alloc_phys_frames_perm (uint32 count, uint8 perm)
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
      if (shm->ept_initialized[get_pcpu_id ()]) {
        shm_set_ept_permission (i << 12, count, perm);
      }
      return (i << 12);           /* physical byte address of free frames */
      keep_searching:
      ;
    }

    spinlock_unlock (&(shm->shm_lock));
  }

  DLOG ("shm_alloc_phys_frames failed!");
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

//#define ISBM_USE_IPI

bool send_intersandbox_msg(isb_msg_type_t msg_type, uint target_sandbox,
			   void* msg, size_t size)
{
#ifdef ISBM_USE_IPI
  return FALSE;
#else
  int attempt_count = 20;
  uint sender_sandbox = get_pcpu_id();
  isb_msg_type_t* arena;
  if( (sender_sandbox == target_sandbox) ||
      (size + sizeof(isb_msg_type_t) > 0x1000 / 2) ) return FALSE;
  
  arena = shm_comm.priv_write_regions[target_sandbox];
  
  while(attempt_count--) {
    if(*arena == ISBM_NO_MESSAGE) {
      *arena = msg_type;
      memcpy(arena+1, msg, size);
      break;
    }
    else {
      sched_usleep(1000 * 50);
    }
  }

  return attempt_count >= 0;
  
#endif
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
