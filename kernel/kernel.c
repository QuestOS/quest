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
#include "util/printf.h"
#include "util/screen.h"
#include "util/debug.h"
#include "mem/virtual.h"
#include "mem/physical.h"
#include "sched/sched.h"
#include "vm/ept.h"
#include "sched/proc.h"
#include "sched/vcpu.h"
#include "arch/i386-percpu.h"
#include "arch/i386-mtrr.h"

static spinlock kernel_lock ALIGNED(LOCK_ALIGNMENT) = SPINLOCK_INIT;

/* Declare space for a stack */
uint32 ul_stack[NR_MODS][1024] ALIGNED (0x1000);

/* Declare space for a task state segment */
uint32 ul_tss[NR_MODS][1024] ALIGNED (0x1000);

/* Declare space for a page directory */
uint32 pg_dir[NR_MODS][1024] ALIGNED (0x1000);

/* Declare space for a page table */
uint32 pg_table[NR_MODS][1024] ALIGNED (0x1000);

/* Declare space for per process kernel stack */
uint32 kl_stack[NR_MODS][1024] ALIGNED (0x1000);

/* Declare space for a page table mappings for kernel stacks */
uint32 kls_pg_table[NR_MODS][1024] ALIGNED (0x1000);

/* Declare space for a page table for the user stack (only used if different than pg_dir table */
uint32 uls_pg_table[NR_MODS][1024] ALIGNED(0x1000);

/* Each CPU gets an IDLE task -- something to do when nothing else */
quest_tss idleTSS[MAX_CPUS] ALIGNED(0x1000);
task_id idleTSS_selector[MAX_CPUS];

/* Each CPU gets a CPU TSS for sw task switching */
tss cpuTSS[MAX_CPUS];
uint16 cpuTSS_selector[MAX_CPUS];

char *pchVideo = (char *) KERN_SCR;

/* PANIC!!!!! */
void panic (char *sz)
{
  print ("kernel panic: ");
  print (sz);

  cli ();
  hlt ();
}

/* Global kernel lock */
void
lock_kernel (void)
{
  spinlock_lock (&kernel_lock);
}

void
unlock_kernel (void)
{
  spinlock_unlock (&kernel_lock);
}

bool
update_CPU_TSS (uint32_t esp0)
{
  int cpu = get_pcpu_id ();
  tss * tss = &cpuTSS[cpu];
  if (!tss) panic ("update_CPU_TSS failed");
  tss->ulESP0 = esp0;
  return TRUE;
}

bool
kern_stk_pte_free (uint32_t pte)
{
  if ((pte == 0) || ((pte & KERN_STK_PTE_FREE_FLAG) == KERN_STK_PTE_FREE_FLAG))
    return TRUE;
  else
    return FALSE;
}

/*
 * Find an empty kernel stack of 4KB and map it in plPageDirectory.
 * This function is used by _rfork to create user threads.
 */
uint32_t
find_and_map_kernel_level_stack (uint32_t * plPageDirectory)
{
  int i = 0;
  uint32_t * plPageTable = NULL;
  uint32_t phys_frame = 0, stk_frame = 0;

  if (!plPageDirectory[KERN_STK >> 22]) {
    /* This really shouldn't happen */
    return -1;
  } else {
    plPageTable = map_virtual_page ((plPageDirectory[KERN_STK >> 22] & 0xFFFFF000) | 0x3);
    if (!plPageTable) {
      com1_printf ("map_virtual_page failed in find_kernel_level_stack!\n");
      return -1;
    }

    for (i = 0; i < 1024; i++) {
      if (kern_stk_pte_free (plPageTable[i])) {
        /* Found a free kernel stack */
        phys_frame = alloc_phys_frame ();
        if (phys_frame == -1) {
          com1_printf ("alloc_phys_frame failed in find_and_map_kernel_level_stack!\n");
          break;
        }

        plPageTable[i] = (phys_frame | 3);
        unmap_virtual_page (plPageTable);
        stk_frame = 0xFF800000 + (i << 12);
        invalidate_page ((void *) stk_frame);
        return stk_frame;
      }
    }

    unmap_virtual_page (plPageTable);
  }

  return -1;
}

void
free_kernel_level_stack (uint32_t * plPageDirectory, uint32_t stack_addr)
{
  int index = 0;
  uint32_t * plPageTable = NULL;
  uint32_t phys_frame = 0;

  if (!plPageDirectory[KERN_STK >> 22]) {
    /* This really shouldn't happen */
    return;
  } else {
    plPageTable = map_virtual_page ((plPageDirectory[KERN_STK >> 22] & 0xFFFFF000) | 0x3);
    if (!plPageTable) {
      panic ("map_virtual_page failed in free_kernel_level_stack!\n");
    }
    /* Get page table index for stack_addr */
    index = ((stack_addr >> 12) & 0x3FF);
    if (plPageTable[index]) {
      phys_frame = plPageTable[index] & 0xFFFFF000;
      /* Free physical memory */
      free_phys_frame (phys_frame);
      /*
       * Set entry to "free". We cannot unmap kernel stack at this point.
       * It is still used. Only the physical memory is freed. We have to
       * guarantee the rest of thread exit path does not involve physical
       * memory allocation.
       */
      plPageTable[index] |= KERN_STK_PTE_FREE_FLAG;
    } else {
      com1_printf ("Warning: tried to free 0x%X, but it is not mapped!\n", stack_addr);
    }
    unmap_virtual_page (plPageTable);
  }

  return;
}

uint32_t
find_user_level_stack (uint32_t * plPageDirectory)
{
  int pg_dir_index, pg_tbl_index;
  uint32_t * plPageTable;
  uint32_t i = 0, usr_stk = 0;

  /* Search begins at virtual address USER_STACK_START */
  for (i = 0; i < MAX_THREADS; i++) {
    usr_stk = USER_STACK_START - (USER_STACK_SIZE << 12) * i;
    get_pg_dir_and_table_indices((void *)(usr_stk - 1), &pg_dir_index, &pg_tbl_index);
    if (!plPageDirectory[pg_dir_index]) {
      /* Not present */
      return usr_stk;
    } else {
      plPageTable = map_virtual_page ((plPageDirectory[pg_dir_index] & 0xFFFFF000) | 0x3);
      if (!plPageTable) {
        com1_printf ("map_virtual_page failed in find_user_level_stack!\n");
        return -1;
      }
      if (!plPageTable[pg_tbl_index]) {
        /* Found available stack */
        unmap_virtual_page (plPageTable);
        return usr_stk;
      }
      unmap_virtual_page (plPageTable);
    }
  }

  /* Cannot find available user stack */
  com1_printf ("Cannot find available user stack!\n");

  return -1;
}

void map_user_level_stack(uint32_t* plPageDirectory, void* start_addr,
                          int num_frames, uint32_t* frames, bool invalidate_pages)
{
  int pg_dir_index, pg_tbl_index;
  uint32* plPageTable;
  get_pg_dir_and_table_indices(start_addr - 1, &pg_dir_index, &pg_tbl_index);
  if(!plPageDirectory[pg_dir_index]) {
    /* Not present */
    plPageDirectory[pg_dir_index] = alloc_phys_frame() | 7;
    plPageTable = map_virtual_page (plPageDirectory[pg_dir_index]);
    memset (plPageTable, 0, 0x1000);
  }
  else {
    plPageTable = map_virtual_page (plPageDirectory[pg_dir_index]);
  }
  
  
  while(num_frames--) {
    if(pg_tbl_index < 0) {
      pg_tbl_index = 1023;
      unmap_virtual_page(plPageTable);
      pg_dir_index--;
      if(pg_dir_index < 0) {
        com1_printf("Attempted to map a stack that was larger than its starting position");
        panic("Attempted to map a stack that was larger than its starting position");
      }
      if(!plPageDirectory[pg_dir_index]) {
        /* Not present */
        plPageDirectory[pg_dir_index] = alloc_phys_frame() | 7;
        plPageTable = map_virtual_page (plPageDirectory[pg_dir_index]);
        memset (plPageTable, 0, 0x1000);
      }
      else {
        plPageTable = map_virtual_page (plPageDirectory[pg_dir_index]);
      }
    }

    
    plPageTable[pg_tbl_index] = frames[num_frames] | 7;
    invalidate_page((void*)((pg_tbl_index) << 12) + ((1 << 22) * pg_dir_index));
    pg_tbl_index--;
  }

  unmap_virtual_page(plPageTable);
}

void
free_user_level_stack (uint32_t * plPageDirectory, uint32_t stack_addr, int num_frames)
{
  int pg_dir_index, pg_tbl_index;
  uint32_t * plPageTable = NULL;
  uint32_t phys_frame = 0;
  get_pg_dir_and_table_indices((void *) (stack_addr - 1), &pg_dir_index, &pg_tbl_index);

  if(!plPageDirectory[pg_dir_index]) {
    com1_printf ("Warning: tried to free unmapped user stack (0x%X)!\n", stack_addr);
    return;
  } else {
    plPageTable = map_virtual_page ((plPageDirectory[pg_dir_index] & 0xFFFFF000) | 0x03);
    if (!plPageTable) panic ("free_user_level_stack: Out of memory!");
  }

  while (num_frames--) {
    if (pg_tbl_index < 0) {
      pg_tbl_index = 1023;
      unmap_virtual_page(plPageTable);
      pg_dir_index--;
      if(pg_dir_index < 0) {
        panic ("Attempted to free a stack larger than its starting position");
      }
      if(!plPageDirectory[pg_dir_index]) {
        /* Not present */
        com1_printf ("Warning: tried to free unmapped user stack (0x%X)!\n", stack_addr);
        return;
      }
      plPageTable = map_virtual_page ((plPageDirectory[pg_dir_index] & 0xFFFFF000) | 0x3);
      if (!plPageTable) panic ("free_user_level_stack: Out of memory!");
    }

    if (plPageTable[pg_tbl_index]) {
      phys_frame = plPageTable[pg_tbl_index] & 0xFFFFF000;
      /* Free physical memory */
      free_phys_frame (phys_frame);
      /* Set entry to 0 */
      plPageTable[pg_tbl_index] = 0;
      invalidate_page((void*)((pg_tbl_index) << 12) + ((1 << 22) * pg_dir_index));
    }
    pg_tbl_index--;
  }

  unmap_virtual_page (plPageTable);
}

extern void *
lookup_GDT_selector (uint16 selector)
{

  descriptor *ad = (descriptor *) KERN_GDT;

  return (void *) (ad[selector >> 3].pBase0 |
                   (ad[selector >> 3].pBase1 << 16) |
                   (ad[selector >> 3].pBase2 << 24));
}

extern void
get_GDT_descriptor (uint16 selector, descriptor *ad)
{
  *ad = ((descriptor *) KERN_GDT)[selector >> 3];
}

/* Idle loop for CPU IDLE task */
void
idle_task (void)
{
  unlock_kernel ();
  sti ();                       /* when we initially jump here, IF=0 */
  for (;;) {
    asm volatile ("pause");
  }
}

/* Mask out every IDT entry. */
void
disable_idt (void)
{
  uint16 len = KERN_IDT_LEN;
  idt_descriptor *ptr = (idt_descriptor *) KERN_IDT;
  uint16 i;

  for (i = 0; i < (len >> 3); i++) {
    if (ptr[i].pBase0)
      ptr[i].fPresent = 0;
  }
}

/* Unmask every valid IDT entry. */
void
enable_idt (void)
{
  uint16 len = KERN_IDT_LEN;
  idt_descriptor *ptr = (idt_descriptor *) KERN_IDT;
  uint16 i;

  for (i = 0; i < (len >> 3); i++) {
    if (ptr[i].pBase0)
      ptr[i].fPresent = 1;
  }
}

/* Unmask an IDT entry. */
void
enable_idt_entry (uint16 i)
{
  idt_descriptor *ptr = (idt_descriptor *) KERN_IDT;
  if (ptr[i].pBase0)
    ptr[i].fPresent = 1;
}

/* Setup an IDT entry given the address of a function and an
 * appropriate descriptor privilege level. */
void
set_idt_descriptor_by_addr (uint8 n, void *addr, uint8 dpl)
{
  idt_descriptor *ptr = (idt_descriptor *) KERN_IDT;

  ptr[n].fPresent = 0;          /* disable */
  ptr[n].pBase1 = ((uint32) addr & 0xFFFF0000) >> 16;
  ptr[n].pBase0 = ((uint32) addr & 0x0000FFFF);
  ptr[n].pSeg = 0x08;
  ptr[n].fZero0 = 0;
  ptr[n].fZero1 = 0;
  ptr[n].fReserved = 0;
  ptr[n].fType = 0x6;
  ptr[n].f32bit = 1;
  ptr[n].uDPL = dpl;
  ptr[n].fPresent = 1;          /* re-enable */
}

/* Read an IDT entry into the pointer. */
void
get_idt_descriptor (uint8 n, idt_descriptor * d)
{
  idt_descriptor *ptr = (idt_descriptor *) KERN_IDT;

  *d = ptr[n];
}

/* Write an IDT entry from a given entry. */
void
set_idt_descriptor (uint8 n, idt_descriptor * d)
{
  idt_descriptor *ptr = (idt_descriptor *) KERN_IDT;

  ptr[n] = *d;
}

static bool kernel_threads_running = FALSE;
static task_id kernel_thread_waitq = 0;

task_id
create_kernel_thread_vcpu_args (uint eip, uint esp, const char * name,
                                u16 vcpu, bool run, uint n, ...)
{
  task_id pid;
  uint32 eflags;
  quest_tss * tss;
  extern u32 *pgd;              /* original page-global dir */
#ifdef USE_VMX
  int cpu = 0;
  cpu = get_pcpu_id ();
  void *page_dir = (void*) (((u32) &pgd) + SANDBOX_KERN_OFFSET * cpu);
#else
  void *page_dir = &pgd;
#endif
  uint *stack, i, c;
  va_list args;

  if(!vcpu_lookup(vcpu) && (vcpu != BEST_EFFORT_VCPU)) {
    logger_printf("Failed to create kernel thread\n");
    return 0;
  }

  asm volatile ("pushfl\n" "pop %0\n":"=r" (eflags):);

  stack = (uint *) (esp - sizeof (void *) * (n + 1));

  /* place arguments on the stack (i386-specific) */
  va_start (args, n);
  for (i=0; i<n; i++)
    stack[i+1] = va_arg (args, uint);
  va_end (args);

  stack[0] = (uint) exit_kernel_thread; /* set return address */
  esp -= sizeof (void *) * (n + 2);

  /* start kernel thread */
  pid = duplicate_TSS (0, NULL,
                       eip, 0, esp,
                       eflags, (uint32) page_dir);

  tss = lookup_TSS (pid);
  tss->priority = 0x1f;
  if (vcpu != 0xFFFF) {
    tss->cpu = vcpu;
  }
  if (name) {
    c = strlen (name);
    if (c > 31) c = 31;
    memcpy (tss->name, name, c);
    tss->name[c] = '\0';
  }

  if (run) {
    if (kernel_threads_running)
      wakeup (pid);
    else
      queue_append (&kernel_thread_waitq, pid);
  }

  return pid;
}

task_id
start_kernel_thread (uint eip, uint esp, const char * name)
{
  return create_kernel_thread_args (eip, esp, name, TRUE, 0);
}

void
begin_kernel_threads (void)
{
  if (kernel_threads_running == FALSE) {
    wakeup_queue (&kernel_thread_waitq);
    kernel_threads_running = TRUE;
  }
}

void
exit_kernel_thread (void)
{
  uint8 LAPIC_get_physical_ID (void);
  quest_tss *tss;
  task_id tid;
  task_id waiter;

  //for (;;)
  //  sched_usleep (1000000);

  tid = str ();
  tss = lookup_TSS (tid);

  /* All tasks waiting for us now belong on the runqueue. */
  while ((waiter = queue_remove_head (&tss->waitqueue)))
    wakeup (waiter);

  tss_remove (tid);
  free_quest_tss (tss);

  /* clear current task */
  ltr (0);

  schedule ();

  panic ("exit_kernel_thread: unreachable");
}

/* 
 * check_copied_threads checks threads created in BSP sandbox and copied
 * to other sandboxes during vm_mem_init. These threads will be created
 * with a task_id indicating its source sandbox as 0 and affinity to 0.
 * Shell and idle thread should be fix when they were created.
 * 
 * Three different groups need to be checked: (1) The global queue for all
 * quest_tss headed by init_tss (2) The wait queue for all newly created
 * kernel thread headed by kernel_thread_waitq (3) VCPU run queues
 */
void
check_copied_threads (void)
{
  uint cpu = get_pcpu_id ();
  quest_tss * t;
  task_id q = 0;
  vcpu * queue = NULL;
  logger_printf ("Checking threads in sandbox %d\n", cpu);
  logger_printf ("Current Task: 0x%X\n", str ());

  /* Check global (per-sandbox) queue headed by init_tss */
  logger_printf ("Checking threads in global queue...\n");
  for (t = &init_tss; ((t = t->next_tss) != &init_tss);) {
    logger_printf ("  name: %s, task_id: 0x%X, affinity: %d, vcpu: %d\n",
                   t->name, t->tid, t->sandbox_affinity, t->cpu);
  }

  /* Check wait queue headed by kernel_thread_waitq */
  logger_printf ("Checking threads in kernel_thread_waitq...\n");
  q = kernel_thread_waitq;
  while (q) {
    t = lookup_TSS (q);
    logger_printf ("  name: %s, task_id: 0x%X, affinity: %d, vcpu: %d\n",
                   t->name, t->tid, t->sandbox_affinity, t->cpu);
    q = t->next;
  }

  /* Check VCPU queues */
  logger_printf ("Checking VCPU run queues...\n");
  queue = percpu_read (vcpu_queue);
  /* Iterate VCPU queue */
  while (queue) {
    q = queue->runqueue;
    if (queue->tr) {
      logger_printf ("  vcpu 0x%X has current task:\n", queue);
      t = lookup_TSS (queue->tr);
      logger_printf ("    name: %s, task_id: 0x%X, affinity: %d, vcpu: 0x%X (%d)\n",
                     t->name, t->tid, t->sandbox_affinity, queue, t->cpu);
    }
    while (q) {
      t = lookup_TSS (q);
      logger_printf ("  name: %s, task_id: 0x%X, affinity: %d, vcpu: 0x%X (%d)\n",
                     t->name, t->tid, t->sandbox_affinity, queue, t->cpu);
      q = t->next;
    }
    queue = queue->next;
  }

  return;
}

void
disable_all_cache (void)
{  
  u64 mtrr_def_type = rdmsr (IA32_MTRR_DEF_TYPE);

  /* Intel Manual 3, section 11.5.3 Preventing Caching */
  /* Step 1 & 2 */
  /* Disable Cache */
  asm volatile ("push   %%eax\n\t"
                "wbinvd\n\t"
                "movl   %%cr0, %%eax\n\t"
                "orl    $0x40000000, %%eax\n\t"  /* CD set to 1 */
                "andl   $0xDFFFFFFF, %%eax\n\t"  /* Clear NW */
                "movl   %%eax, %%cr0\n\t"
                "pop    %%eax\n\t"
                "wbinvd":::"eax");

  /* Step 3 */

  /* Memory Types That Can Be Encoded in MTRRs    */
  /* -------------------------------------------- */
  /* | Uncacheable (UC)       |  00H            | */
  /* | Write Combining (WC)   |  01H            | */
  /* | Reserved*              |  02H            | */
  /* | Reserved*              |  03H            | */
  /* | Write-through (WT)     |  04H            | */
  /* | Write-protected (WP)   |  05H            | */
  /* | Writeback (WB)         |  06H            | */
  /* | Reserved*              |  7H through FFH | */
  /* -------------------------------------------- */

  /* Disable MTRRs and set default memory type to UC */
  wrmsr (IA32_MTRR_DEF_TYPE, mtrr_def_type & 0xFFFFFFFFFFFFF300L);

#if 0
  /* Set all MTRRs to UC type. This shouldn't be necessary. */
  u64 mtrr_cap = rdmsr (IA32_MTRRCAP);
  u8 num_var_reg = (u8) mtrr_cap;
  bool fix_supported = (mtrr_cap >> 8) & 0x01;
  u64 var_base, var_mask;
  int i = 0;

  for (i=0; i < num_var_reg; i++) {
    var_base = rdmsr (IA32_MTRR_PHYS_BASE (i));
    var_mask = rdmsr (IA32_MTRR_PHYS_MASK (i));
    wrmsr (IA32_MTRR_PHYS_BASE (i), var_base & 0xFFFFFFFFFFFFFF00L);
    wrmsr (IA32_MTRR_PHYS_MASK (i), var_mask);
  }

  if (fix_supported) {
    wrmsr (IA32_MTRR_FIX64K_00000, 0L);
    wrmsr (IA32_MTRR_FIX16K_80000, 0L);
    wrmsr (IA32_MTRR_FIX16K_A0000, 0L);
    wrmsr (IA32_MTRR_FIX4K_C0000, 0l);
    wrmsr (IA32_MTRR_FIX4K_C8000, 0l);
    wrmsr (IA32_MTRR_FIX4K_D0000, 0l);
    wrmsr (IA32_MTRR_FIX4K_D8000, 0l);
    wrmsr (IA32_MTRR_FIX4K_E0000, 0l);
    wrmsr (IA32_MTRR_FIX4K_E8000, 0l);
    wrmsr (IA32_MTRR_FIX4K_F0000, 0l);
    wrmsr (IA32_MTRR_FIX4K_F8000, 0l);
  }
#endif

  asm volatile ("wbinvd");

  return;
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
