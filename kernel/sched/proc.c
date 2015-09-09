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
#include "sched/vcpu.h"
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
tss_remove (quest_tss *t)
{
  (t->prev_tss)->next_tss = t->next_tss;
  (t->next_tss)->prev_tss = t->prev_tss;

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

quest_tss *
alloc_thread_TSS (uint32_t child_directory,
                  uint32_t eip,
                  uint32_t child_eip,
                  uint32_t child_eflags,
                  uint32_t usr_stk,
                  uint32_t parent_esp,
                  uint32_t parent_ebp,
                  quest_tss * mTSS)
{
  quest_tss * pTSS = NULL;
  linear_address_t esp_la;
  pgdir_t child_pgdir;
  frame_t esp_frame;
  uint32_t klStack = 0;
  uint32_t * virt_pgd = NULL, * esp_virt = NULL;

  /* Note, we rely on page being initialised to 0 since EAX contains
   * return value for child
   */
  if (!(pTSS = alloc_quest_tss ())) {
    com1_printf ("alloc_quest_tss failed!\n");
    return NULL;
  }

  /* Clear virtual page before use. */
  memset (pTSS, 0, 4096);

  tss_add_tail (pTSS);

  virt_pgd = map_virtual_page (child_directory | 0x3);
  if (!virt_pgd) {
    com1_printf ("map_virtual_page failed in alloc_thread_TSS!\n");
    return NULL;
  }

  pTSS->CR3 = (u32) child_directory;
  /* The child will begin running at the specified EIP */
  pTSS->initial_EIP = child_eip;

  /* Find a kernel stack for new thread */
  klStack = find_and_map_kernel_level_stack (virt_pgd);
  if (klStack == -1) {
    com1_printf ("Kernel stack allocation failed!\n");
    return NULL;
  }
  //com1_printf ("Found new kernel stack: 0x%X\n", klStack);
  //com1_printf ("Parent ESP=0x%X\n", parent_esp);
  /* Copy kernel stack */
  memcpy ((void *) klStack, (void *) (parent_esp & 0xFFFFF000), 4096);
  /* 4 bytes are needed for kernel return address. Used to "return" from switch_to. */
  pTSS->ESP = klStack + (parent_esp & 0xFFF) - 4;
  pTSS->EBP = klStack + (parent_ebp & 0xFFF);

  /* modify stack in child space (patch up "kernel" return address -- call 1f) */
  esp_la.raw = pTSS->ESP;
  child_pgdir.dir_pa = child_directory;
  child_pgdir.dir_va = (pgdir_entry_t *) virt_pgd;
  esp_frame = pgdir_get_frame (child_pgdir, (void *) (pTSS->ESP & (~0xFFF)));
  esp_virt = map_virtual_page (esp_frame | 3);
  if (esp_virt == NULL) panic ("esp_virt: out of memory");
  esp_virt[esp_la.offset >> 2] = eip;

#if 0
  com1_printf ("Dump copied kernel stack:\n");
  com1_printf ("  esp_virt[1023]=0x%x\n", esp_virt[1023]);
  com1_printf ("  esp_virt[1022]=0x%x\n", esp_virt[1022]);
  com1_printf ("  esp_virt[1021]=0x%x\n", esp_virt[1021]);
  com1_printf ("  esp_virt[1020]=0x%x\n", esp_virt[1020]);
  com1_printf ("  esp_virt[1019]=0x%x\n", esp_virt[1019]);
  com1_printf ("  esp_virt[1018]=0x%x\n", esp_virt[1018]);
  com1_printf ("  esp_virt[1017]=0x%x\n", esp_virt[1017]);
#endif

  /* Patch up kernel stack for "user" return address */
  esp_virt[1023] = 0x23;                   /* SS selector */
  esp_virt[1022] = usr_stk - 100;          /* ESP3 -- User stack */
  esp_virt[1021] = F_1 | F_IF | 0x3000;    /* EFLAGS */
  esp_virt[1020] = 0x1B;                   /* CS selector */
  esp_virt[1019] = child_eip;              /* EIP -- User EIP */
  esp_virt[1018] = 0x00230023;             /* FS/GS selectors */
  esp_virt[1017] = 0x00230023;             /* DS/ES selectors */

  /* child_pgdir.dir_va is virt_pgd */
  unmap_virtual_page (child_pgdir.dir_va);
  unmap_virtual_page (esp_virt);

  pTSS->EFLAGS = child_eflags & 0xFFFFBFFF;   /* Disable NT flag */
  /* Inherit priority from main thread */
  pTSS->priority = mTSS->priority;
  pTSS->tid = new_task_id ();
  /* Set parent tid to main thread tid */
  pTSS->ptid = mTSS->tid;
  /* Increase thread count. Child num_threads is not changed. */
  pTSS->num_threads = mTSS->num_threads++;
  /* Remember user level stack location */
  pTSS->ulStack = usr_stk;
#ifdef NO_FPU
  /* Do nothing */
#else
  save_fpu_and_mmx_state(pTSS->fpu_state);
#endif
  pTSS->sandbox_affinity = get_pcpu_id ();
  pTSS->machine_affinity = 0;

  semaphore_init (&pTSS->Msem, 1, 0);

  pTSS->cpu = BEST_EFFORT_VCPU;
  
  return pTSS;
}

/* Duplicate parent TSS -- used with fork */
quest_tss *
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
  pTSS->ptid = pTSS->tid;
  pTSS->num_threads = 1;
  pTSS->ulStack = USER_STACK_START;
#ifdef NO_FPU
  /* Do nothing */
#else
  save_fpu_and_mmx_state(pTSS->fpu_state);
#endif
  pTSS->sandbox_affinity = get_pcpu_id ();
  pTSS->machine_affinity = 0;

  semaphore_init (&pTSS->Msem, 1, 0);

  pTSS->cpu = BEST_EFFORT_VCPU;
  
  return pTSS;
}

quest_tss *
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
  pTSS->ptid = pTSS->tid;
  pTSS->num_threads = 1;
  pTSS->ulStack = USER_STACK_START;
  pTSS->sandbox_affinity = cpu_num;
  pTSS->machine_affinity = 0;
  memcpy (pTSS->name, name, strlen (name));
  pTSS->name[strlen (name)] = '\0';

  return pTSS;
}

/* Allocate a basic TSS */
quest_tss *
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
  pTSS->ptid = pTSS->tid;
  pTSS->num_threads = 1;
  pTSS->ulStack = USER_STACK_START;
  pTSS->sandbox_affinity = get_pcpu_id ();
  pTSS->machine_affinity = 0;

  semaphore_init (&pTSS->Msem, 1, 0);

  return pTSS;
}


/* -- EM -- This should probably be put someplace else but for now
      this is good enough */

fd_table_file_entry_t* alloc_fd_table_file_entry(char* pathname, size_t file_length)
{
  fd_table_file_entry_t* res;
  res = kmalloc(sizeof(fd_table_file_entry_t));

  if(!res) {
    return NULL;
  }

  res->file_length = file_length;
  res->pathname = kmalloc(strlen(pathname) + 1);

  if(!res->pathname) {
    kfree(res);
    return NULL;
  }

#ifdef ONE_FILE_READ
  res->file_buffer = kmalloc(file_length);
  if(!res->file_buffer) {
    kfree(res->pathname);
    kfree(res);
    return NULL;
  }
#else
  res->file_buffer = NULL;
#endif
  

  strcpy(res->pathname, pathname);
  return res;
}

void free_fd_table_file_entry(fd_table_file_entry_t* entry)
{
  if(entry->file_buffer) kfree(entry->file_buffer);
  kfree(entry->pathname);
  kfree(entry);
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
