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

/* Each CPU gets an IDLE task -- something to do when nothing else */
quest_tss idleTSS[MAX_CPUS];
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
create_kernel_thread_args (uint eip, uint esp, const char * name, bool run, uint n, ...)
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

  for (;;)
    sched_usleep (1000000);

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

#ifdef USE_VMX
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
  logger_printf ("Fixing threads in sandbox %d\n", cpu);

  /* Check global (per-sandbox) queue headed by init_tss */
  logger_printf ("Checking threads in global queue...\n");
  for (t = &init_tss; ((t = t->next_tss) != &init_tss);) {
    logger_printf ("  name: %s, task_id: 0x%X, affinity: %d\n",
                   t->name, t->tid, t->sandbox_affinity);
  }

  /* Check wait queue headed by kernel_thread_waitq */
  logger_printf ("Checking threads in kernel_thread_waitq...\n");
  q = kernel_thread_waitq;
  while (q) {
    t = lookup_TSS (q);
    logger_printf ("  name: %s, task_id: 0x%X, affinity: %d\n",
                   t->name, t->tid, t->sandbox_affinity);
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
      logger_printf ("    name: %s, task_id: 0x%X, affinity: %d, vcpu: 0x%X\n",
                     t->name, t->tid, t->sandbox_affinity, queue);
    }
    while (q) {
      t = lookup_TSS (q);
      logger_printf ("  name: %s, task_id: 0x%X, affinity: %d, vcpu: 0x%X\n",
                     t->name, t->tid, t->sandbox_affinity, queue);
      q = t->next;
    }
    queue = queue->next;
  }

  return;
}
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
