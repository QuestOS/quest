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
#include "vm/vmx.h"
#include "vm/ept.h"
#include "vm/shm.h"
#include "kernel.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "util/printf.h"
#include "smp/apic.h"
#include "arch/i386.h"
#include "sched/sched.h"
#include "vm/shdr.h"
#include "vm/migration.h"
#include "sched/sched.h"
#include "sched/vcpu.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"

//#define DEBUG_MIGRATION

#ifdef DEBUG_MIGRATION
#define DLOG(fmt,...) DLOG_PREFIX("Migration",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

extern bool sleepqueue_detach (task_id);
extern void sleepqueue_append (task_id);

void *
detach_task (task_id tid)
{
  quest_tss * tss = lookup_TSS (tid);
  if (!tss) return NULL;
  vcpu * v = vcpu_lookup (tss->cpu);
  if (!v) return NULL;
  if (!vcpu_in_runqueue (v, tid)) {
    DLOG ("task 0x%X not in vcpu %d run queue", tid, tss->cpu);
    /* tid is current task in its VCPU? */
    if (v->tr == tid) {
      /* If yes, we need to check the sleep queue. This is probably the only
       * reason why this case could occur. The task has to be removed from the
       * sleep queue of current sandbox and inserted into the sleep queue of
       * the remote sandbox with the timestamp properly fixed.
       */
      DLOG ("task 0x%X is VCPU %d's current task", tid, tss->cpu);
      if (v->runqueue == 0) {
        v->runnable = FALSE;
      }
      /* Detach the task from local sleep queue if necessary. If task is not
       * in the sleep queue, we set the sleep time to 0 so that the destination
       * sandbox knows it won't need to put it to its sleep queue.
       */
      if (!sleepqueue_detach (tid)) tss->time = 0;
      return get_phys_addr ((void *) tss);
    }
    return NULL;
  } else {
    /* Remove migrating quest_tss from its VCPU run queue */
    vcpu_remove_from_runqueue (v, tid);
    /* Is the VCPU run queue empty? */
    if (v->runqueue == 0) {
      if ((v->tr == 0) || (v->tr == tid)) {
        v->runnable = FALSE;
      }
    }
    /* Return the physical address of the detached quest_tss */
    return get_phys_addr ((void *) tss);
  }
  return NULL;
}

quest_tss *
pull_quest_tss (void * phy_tss)
{
  int cpu = get_pcpu_id ();
  quest_tss * new_tss = NULL;
  quest_tss * target_tss = map_virtual_page (((uint32) phy_tss) | 3);
  if (!target_tss) return NULL;

  DLOG ("Duplicating quest_tss:");
  DLOG ("  name: %s, task_id: 0x%X, affinity: %d, CR3: 0x%X",
        target_tss->name, target_tss->tid, target_tss->sandbox_affinity, target_tss->CR3);

  new_tss = alloc_quest_tss ();
  if (!new_tss) {
    DLOG ("alloc_quest_tss failed!");
    goto abort;
  }
  memcpy (new_tss, target_tss, sizeof (quest_tss));

  /* Fix sleep time if necessary */
  if (new_tss->time) {
    if (shm->remote_tsc_diff[cpu]) {
      /* Local TSC is faster */
      new_tss->time += shm->remote_tsc[cpu];
    } else {
      /* Local TSC is slower */
      new_tss->time -= shm->remote_tsc[cpu];
    }
  }

  DLOG ("Duplicated quest_tss:");
  DLOG ("  name: %s, task_id: 0x%X, affinity: %d, CR3: 0x%X",
        new_tss->name, new_tss->tid, new_tss->sandbox_affinity, new_tss->CR3);

  unmap_virtual_page (target_tss);
  return new_tss;

abort:
  unmap_virtual_page (target_tss);
  return NULL;
}

int
attach_task (quest_tss * new_tss)
{
  uint64 now;

  /* Add new quest_tss to per-sandbox quest_tss list */
  tss_add_tail (new_tss);
  /* TODO: Find a VCPU or create a new one for the process */
  /* Let's do a wakeup or sleep for now without considering VCPU */
  RDTSC (now);
  if (new_tss->time == 0 || now >= new_tss->time) {
    /* Not sleeping or sleep expires already. Wake up directly. */
    DLOG ("Waking up task in destination sandbox");
    wakeup (new_tss->tid);
    new_tss->time = 0;
  } else {
    /* The migrating task is still sleeping */
    DLOG ("Adding task in destination sandbox sleep queue");
    sleepqueue_append (new_tss->tid);
  }

#ifdef DEBUG_MIGRATION
  check_copied_threads ();
#endif

  return 1;
}

pgdir_t
remote_clone_page_directory (pgdir_t dir)
{
  frame_t new_pgd_pa;
  pgdir_t new_dir, shell_dir;
  uint i;
  extern task_id shell_tss;
  quest_tss * stss = lookup_TSS (shell_tss);
  if (!stss) goto abort;
  shell_dir.dir_pa = stss->CR3;
  shell_dir.dir_va = map_virtual_page (shell_dir.dir_pa | 3);
  if (!shell_dir.dir_va) goto abort;

  new_pgd_pa = alloc_phys_frame ();

  if (new_pgd_pa == -1)
    goto abort;

  new_dir.dir_pa = new_pgd_pa;
  new_dir.dir_va = map_virtual_page (new_pgd_pa | 3);
  if (new_dir.dir_va == NULL)
    goto abort_pgd_pa;

  memset (new_dir.dir_va, 0, PGDIR_NUM_ENTRIES * sizeof (pgdir_entry_t));

  /* run through dir and make copies of tables */
  for (i=0; i<PGDIR_NUM_ENTRIES; i++) {
    if (dir.dir_va[i].flags.present) {
      if (i >= PGDIR_KERNEL_BEGIN && i != PGDIR_KERNEL_STACK) {
        /* Replace kernel mappings with destination sandbox kernel */
        /* To make things easier, let's use local shell's kernel mappings directly. */
        /* This is what fork does anyway:-) */
        new_dir.dir_va[i].raw = shell_dir.dir_va[i].raw;
      } else if (dir.dir_va[i].flags.page_size) {
        /* clone 4 MiB page */
        /* --??-- Assume 4 MiB page is always for shared memory, no relocation */
        //panic ("userspace 4 MiB pages not supported");
        new_dir.dir_va[i].raw = (dir.dir_va[i].framenum << 22) + 0x83;
      } else {
        /* clone a page table */
        pgtbl_t tbl, new_tbl;

        /* setup a pgtbl struct with physical and virtual addresses of
         * the existing page table */
        tbl.table_pa = FRAMENUM_TO_FRAME (dir.dir_va[i].table_framenum);
        tbl.table_va = map_virtual_page (tbl.table_pa | 3);
        if (tbl.table_va == NULL)
          goto abort_pgd_va;
        tbl.starting_va = (uint8 *) (i << 22);

        new_tbl = clone_page_table (tbl);

        unmap_virtual_page (tbl.table_va);

        if (new_tbl.table_pa == -1)
          goto abort_pgd_va;

        unmap_virtual_page (new_tbl.table_va);

        /* setup new directory entry with flags and new table address */
        new_dir.dir_va[i].flags = dir.dir_va[i].flags;
        new_dir.dir_va[i].table_framenum = FRAME_TO_FRAMENUM (new_tbl.table_pa);
      }
    }
  }

  return new_dir;

 abort_pgd_va:
  unmap_virtual_page (new_dir.dir_va);
 abort_pgd_pa:
  free_phys_frame (new_pgd_pa);
  unmap_virtual_page (shell_dir.dir_va);
 abort:
  new_dir.dir_va = NULL;
  new_dir.dir_pa = -1;
  return new_dir;
}

void
destroy_task (task_id tid)
{
  quest_tss * tss = lookup_TSS (tid);
  uint32 * virt_addr = NULL;
  uint32 * tmp_page = NULL;
  task_id waiter = 0;
  int i, j;
  if (!tss) return;

  /* Reclaim resources */
  virt_addr = map_virtual_page (tss->CR3 | 3);
  if (!virt_addr) goto abort;

  /* Free user-level virtual address space */
  for (i = 0; i < 1023; i++) {
    if (virt_addr[i]            /* Free page directory entry */
        &&!(virt_addr[i] & 0x80)) {     /* and not 4MB page */
      tmp_page = map_virtual_page (virt_addr[i] | 3);
      for (j = 0; j < 1024; j++) {
        if (tmp_page[j]) {      /* Free frame */
          if ((j < 0x200) || (j > 0x20F) || i) {        /* --??-- Skip releasing
                                                           video memory */
            BITMAP_SET (mm_table, tmp_page[j] >> 12);
          }
        }
      }
      unmap_virtual_page (tmp_page);
      BITMAP_SET (mm_table, virt_addr[i] >> 12);
    }
  }

  BITMAP_SET (mm_table, (uint32) tss->CR3 >> 12);    /* Free up page for page directory */
  unmap_virtual_page (virt_addr);

  for (i = 3; i < MAX_FD; i++) {
    if (tss->fd_table[i].entry) {
      switch (tss->fd_table[i].type) {
        case FD_TYPE_UDP :
          udp_remove ((struct udp_pcb *) tss->fd_table[i].entry);
          break;
        case FD_TYPE_TCP :
          if (tcp_close ((struct tcp_pcb *) tss->fd_table[i].entry) != ERR_OK) {
            logger_printf ("TCP PCB close failed in exit\n");
          }
          break;
        default :
          break;
      }
    }
  }

  /* All tasks waiting for us now belong on the runqueue. */
  while ((waiter = queue_remove_head (&tss->waitqueue)))
    wakeup (waiter);

 abort:
  /* Remove quest_tss */
  tss_remove (tss->tid);
  free_quest_tss (tss);

  return;
}

int
request_migration (int sandbox)
{
  int cpu = get_pcpu_id ();
  uint64 now;
  if (cpu == sandbox) {
    DLOG ("Ignored migration request to local sandbox %d", cpu);
    return 0;
  }
  /* Put current TSC into remote_tsc of "sandbox" */
  /* This is used to fix time difference. */
  /* TODO: Also add IPI overhead into the tsc? */
  RDTSC (now);
  shm->remote_tsc[sandbox] = now;
  return LAPIC_send_ipi (0x1 << sandbox,
                         LAPIC_ICR_LEVELASSERT
                         | LAPIC_ICR_DM_LOGICAL
                         | MIGRATION_RECV_REQ_VECTOR);
}

extern void * vm_exit_input_param;
extern void * vm_exit_return_val;

/* 
 * Trap into monitor and do the address space duplication. The newly
 * created process quest_tss will be returned.
 */
static quest_tss *
trap_and_migrate (void * phy_tss)
{
  vm_exit_input_param = phy_tss;
  vm_exit (VM_EXIT_REASON_MIGRATION);
  return (quest_tss *) vm_exit_return_val;
}

static uint32
receive_migration_request (uint8 vector)
{
  quest_tss * new_tss = NULL;
  int cpu = 0;
  uint64 now;

  /* --YL-- This is ugly. Let's think of some trick later. For now, do it the
   * save way for easier debugging.
   */

  /* Get the tsc difference at this time between remote and local sandboxes. We
   * need to minimize the cycles spent doing this offset calculation or somehow
   * compensate for it by considering the overhead in the final calculation.
   */
  RDTSC (now);
  cpu = get_pcpu_id ();
  DLOG ("Local TSC: 0x%llX Remote TSC: 0x%llX", now, shm->remote_tsc[cpu]);

  if (now <= shm->remote_tsc[cpu]) {
    shm->remote_tsc[cpu] -= now;
    shm->remote_tsc_diff[cpu] = FALSE;
  } else {
    shm->remote_tsc[cpu] = now - shm->remote_tsc[cpu];
    shm->remote_tsc_diff[cpu] = TRUE;
  }

  DLOG ("TSC Diff: 0x%llx Flag: %d",
        shm->remote_tsc[cpu], shm->remote_tsc_diff[cpu]);

  DLOG ("Received migration request in sandbox kernel %d!", cpu);
  /* What is the request on the queue? */
  if (shm->migration_queue[cpu]) {
    DLOG ("Process quest_tss to be migrated: 0x%X", (uint32) shm->migration_queue[cpu]);
    /* TODO: Wake up the migration thread here */
    /* Trap to monitor now for convenience */
    new_tss = trap_and_migrate (shm->migration_queue[cpu]);
    if (new_tss) {
      DLOG ("New quest_tss:");
      DLOG ("  name: %s, task_id: 0x%X, affinity: %d, CR3: 0x%X",
            new_tss->name, new_tss->tid, new_tss->sandbox_affinity, new_tss->CR3);
      /* Add new (migrated) process to local sandbox scheduler */
      if (!attach_task (new_tss)) {
        DLOG ("Attaching task failed!");
        /* Destroy task */
        destroy_task (new_tss->tid);
      }
    } else {
      DLOG ("trap_and_migrate failed!");
      return 0;
    }
  } else {
    /* No request in queue! */
    DLOG ("No request found in migration queue!");
    return 0;
  }

  return 0;
}

static uint32
receive_cleanup_request (uint8 vector)
{
  int cpu = get_pcpu_id ();
  DLOG ("Received migration cleanup request in sandbox kernel %d!", cpu);

  return 0;
}

/*
 * migration_init should be called by EACH sandbox kernel to setup
 * migration threads and register IPI handlers etc.
 */
bool
migration_init (void)
{
  int cpu = get_pcpu_id ();

  if (vector_used (MIGRATION_RECV_REQ_VECTOR)) {
    DLOG ("Interrupt vector %d has been used in sandbox %d. IPI handler registration failed!",
          MIGRATION_RECV_REQ_VECTOR, cpu);
  } else {
    set_vector_handler (MIGRATION_RECV_REQ_VECTOR, &receive_migration_request);
  }

  if (vector_used (MIGRATION_CLEANUP_VECTOR)) {
    DLOG ("Interrupt vector %d has been used in sandbox %d. IPI handler registration failed!",
          MIGRATION_CLEANUP_VECTOR, cpu);
  } else {
    set_vector_handler (MIGRATION_CLEANUP_VECTOR, &receive_cleanup_request);
  }


  DLOG ("Migration subsystem initialized in sandbox kernel %d", cpu);

  return TRUE;
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
