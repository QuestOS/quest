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

#include "arch/i386.h"
#include "kernel.h"
#include "smp/spinlock.h"
#include "mem/malloc.h"
#include "mem/virtual.h"
#include "mem/physical.h"
#include "sched/proc.h"
#ifdef USE_VMX
#include "vm/ept.h"
#include "vm/shm.h"
#endif

extern descriptor idt[];

quest_tss init_tss = {
  .next_tss = NULL,
  .prev_tss = NULL,
  .tid = 0
};

void
tss_add_head (quest_tss * new_tss)
{
  quest_tss * old_head = init_tss.next_tss;

  if (old_head == NULL) {
    init_tss.next_tss = new_tss;
    init_tss.prev_tss = new_tss;
    new_tss->next_tss = &init_tss;
    new_tss->prev_tss = &init_tss;
  } else {
    init_tss.next_tss = new_tss;
    new_tss->next_tss = old_head;
    new_tss->prev_tss = &init_tss;
    old_head->prev_tss = new_tss;
  }
}

void
tss_add_tail (quest_tss * new_tss)
{
  quest_tss * old_tail = init_tss.prev_tss;

  if (old_tail == NULL) {
    init_tss.next_tss = new_tss;
    init_tss.prev_tss = new_tss;
    new_tss->next_tss = &init_tss;
    new_tss->prev_tss = &init_tss;
  } else {
    init_tss.prev_tss = new_tss;
    new_tss->next_tss = &init_tss;
    new_tss->prev_tss = old_tail;
    old_tail->next_tss = new_tss;
  }
}

void
tss_remove (task_id tid)
{
  quest_tss * t;

  for (t = &init_tss; ((t = t->next_tss) != &init_tss);) {
    if (t->tid == tid) {
      (t->prev_tss)->next_tss = t->next_tss;
      (t->next_tss)->prev_tss = t->prev_tss;
      return;
    }
  }

  return;
}

quest_tss *
alloc_quest_tss ()
{
  // TODO
  // Use kernel memory allocator later.
  // For now, one page for a quest_tss.
  quest_tss *pTSS;
  uint32 pa;

  pa = alloc_phys_frame ();
  if (pa == -1) return NULL;
  pTSS = map_virtual_page (pa + 3);
  memset (pTSS, 0, sizeof (quest_tss));

  return pTSS;
}

void
free_quest_tss (quest_tss * tss)
{
  memset (tss, 0, sizeof (quest_tss));
  uint32 pa = (uint32) get_phys_addr (tss);
  unmap_virtual_page (tss);
  free_phys_frame (pa);
}

task_id
new_task_id ()
{
  // TODO
  // Assign a new task id. To be implemented later.
  static uint16 index = 1;
  uint32 cpu = get_pcpu_id ();
  index++;
  return (uint32) ((cpu << 16) | index);
}

/* Retrieve the pointer to the TSS structure for the task */
extern quest_tss *
lookup_TSS (task_id tid)
{
  quest_tss * t;

  for (t = &init_tss; ((t = t->next_tss) != &init_tss);) {
    if (t->tid == tid) return t;
  }

  return NULL;
}

/* Duplicate parent TSS -- used with fork */
task_id
duplicate_TSS (uint32 ebp,
               uint32 *esp,
               uint32 child_eip,
               uint32 child_ebp,
               uint32 child_esp,
               uint32 child_eflags, 
               uint32 child_directory)
{
  quest_tss *pTSS;

  /* Note, we rely on page being initialised to 0 since EAX contains
   * return value for child
   */
  pTSS = alloc_quest_tss ();

  /* Clear virtual page before use. */
  memset (pTSS, 0, 4096);

  //logger_printf ("duplicate_TSS: pTSS=%p i=0x%x esp=%p ebp=%p\n",
  //               pTSS, i << 3,
  //               child_esp, child_ebp);

  tss_add_tail (pTSS);
  
  pTSS->CR3 = (u32) child_directory;
  /* The child will begin running at the specified EIP */
  pTSS->initial_EIP = child_eip;

  /* modify stack in child space */
  linear_address_t esp_la; esp_la.raw = child_esp;
  pgdir_t child_pgdir;
  child_pgdir.dir_pa = child_directory;
  child_pgdir.dir_va = map_virtual_page (child_directory | 3);
  if (child_pgdir.dir_va == NULL)
    panic ("child_pgdir: out of memory");
  frame_t esp_frame = pgdir_get_frame (child_pgdir, (void *) (child_esp & (~0xFFF)));
  u32 *esp_virt = map_virtual_page (esp_frame | 3);
  if (esp_virt == NULL)
    panic ("esp_virt: out of memory");
  esp_virt[esp_la.offset >> 2] = pTSS->initial_EIP;
  unmap_virtual_page (child_pgdir.dir_va);
  unmap_virtual_page (esp_virt);

  pTSS->EFLAGS = child_eflags & 0xFFFFBFFF;   /* Disable NT flag */
  pTSS->ESP = child_esp;
  pTSS->EBP = child_ebp;
  pTSS->tid = new_task_id ();
  save_fpu_state(pTSS->fpu_state);
  pTSS->sandbox_affinity = get_pcpu_id ();
  pTSS->machine_affinity = 0;

  semaphore_init (&pTSS->Msem, 1, 0);

  pTSS->cpu = 0xFF;
  
  /* Return the index into the GDT for the segment */
  return pTSS->tid;
}

task_id
alloc_idle_TSS (int cpu_num)
{
  quest_tss *pTSS = (quest_tss *) (&idleTSS[cpu_num]);
  void idle_task (void);
  char * name = "idle thread";

  tss_add_tail (pTSS);

  u32 *stk = map_virtual_page (alloc_phys_frame () | 3);

#ifdef USE_VMX
  pTSS->CR3 = (u32) (((u32) get_pdbr ()) + cpu_num * SANDBOX_KERN_OFFSET);
#else
  pTSS->CR3 = (u32) get_pdbr ();
#endif
  pTSS->initial_EIP = (u32) & idle_task;
  stk[1023] = pTSS->initial_EIP;
  pTSS->EFLAGS = F_1 | F_IOPL0;

  pTSS->ESP = (u32) &stk[1023];
  pTSS->EBP = pTSS->ESP;
  pTSS->tid = new_task_id ();
  pTSS->sandbox_affinity = cpu_num;
  pTSS->machine_affinity = 0;
  memcpy (pTSS->name, name, strlen (name));
  pTSS->name[strlen (name)] = '\0';

  /* Return the index into the GDT for the segment */
  return pTSS->tid;
}

/* Allocate a basic TSS */
task_id
alloc_TSS (void *pPageDirectory, void *pEntry, int mod_num)
{
  quest_tss *pTSS = (quest_tss *) ul_tss[mod_num];

  tss_add_tail (pTSS);

  pTSS->CR3 = (u32) pPageDirectory;
  pTSS->initial_EIP = (u32) pEntry;

  if (mod_num != 1)
    pTSS->EFLAGS = F_1 | F_IF | F_IOPL0;
  else
    pTSS->EFLAGS = F_1 | F_IF | F_IOPL;       /* Give terminal server access to
                                               * screen memory */

  pTSS->ESP = USER_STACK_START - 100;
  pTSS->EBP = USER_STACK_START - 100;
  pTSS->tid = new_task_id ();
  pTSS->sandbox_affinity = get_pcpu_id ();
  pTSS->machine_affinity = 0;

  semaphore_init (&pTSS->Msem, 1, 0);

  /* Return the index into the GDT for the segment */
  return pTSS->tid;
}


/* -- EM -- This should probably be put someplace else but for now
      this is good enough */

fd_table_file_entry_t* alloc_fd_table_file_entry(char* pathname)
{
  fd_table_file_entry_t* res;
  res = kmalloc(sizeof(fd_table_file_entry_t));

  if(!res) {
    return NULL;
  }

  res->pathname = kmalloc(strlen(pathname) + 1);

  if(!res->pathname) {
    kfree(res);
    return NULL;
  }

  strcpy(res->pathname, pathname);
  return res;
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
