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

static spinlock kernel_lock = SPINLOCK_INIT;

/* Declare space for a stack */
uint32 ul_stack[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a task state segment */
uint32 ul_tss[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page directory */
uint32 pg_dir[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table */
uint32 pg_table[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for per process kernel stack */
uint32 kl_stack[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table mappings for kernel stacks */
uint32 kls_pg_table[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a dummy TSS -- used for kernel switch_to/jmp_gate
   semantics */
tss dummyTSS;

/* This is a global index into the GDT for a dummyTSS */
uint16 dummyTSS_selector;

/* Each CPU gets an IDLE task -- something to do when nothing else */
tss idleTSS[MAX_CPUS];
uint16 idleTSS_selector[MAX_CPUS];

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

/* Retrieve the pointer to the TSS structure for the task by
 * reconstructing it from the GDT descriptor entry. */
extern quest_tss *
lookup_TSS (uint16 selector)
{

  descriptor *ad = (descriptor *) KERN_GDT;

  return (quest_tss *) (ad[selector >> 3].pBase0 |
                        (ad[selector >> 3].pBase1 << 16) |
                        (ad[selector >> 3].pBase2 << 24));
}

/* Idle loop for CPU IDLE task */
void
idle_task (void)
{
  unlock_kernel ();
  sti ();                       /* when we initially jump here, IF=0 */
  for (;;) {
    asm volatile ("hlt");
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
start_kernel_thread (uint eip, uint esp)
{
  task_id pid;
  uint32 eflags;
  void *page_dir = get_pdbr ();

  asm volatile ("pushfl\n" "pop %0\n":"=r" (eflags):);

  /* start kernel thread */
  pid = duplicate_TSS (0, NULL,
                       eip, 0, esp,
                       eflags, (uint32) page_dir);

  if (kernel_threads_running)
    wakeup (pid);
  else
    queue_append (&kernel_thread_waitq, pid);

  return pid;
}

void
begin_kernel_threads (void)
{
  if (kernel_threads_running == FALSE) {
    wakeup_queue (&kernel_thread_waitq);
    kernel_threads_running = TRUE;
  }
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
