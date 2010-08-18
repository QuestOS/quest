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
#include "arch/i386-percpu.h"
#include "kernel.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/debug.h"
#include "sched/sched.h"

//#define DEBUG_SCHED
#ifdef DEBUG_SCHED
#define DLOG(fmt,...) DLOG_PREFIX("sched",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

uint16 runqueue[MAX_PRIO_QUEUES];
uint16 waitqueue[MAX_PRIO_QUEUES];      /* For tasks having expired
                                           their current quanta */
static uint32 runq_bitmap[(MAX_PRIO_QUEUES + 31) / 32];

static int bitmap_find_first_set (uint32 *table, uint32 limit);

#define preserve_segment(next)                                  \
  {                                                             \
    tss *tssp = (tss *)lookup_TSS (next);                       \
    u16 sel;                                                    \
    asm volatile ("movw %%"PER_CPU_SEG_STR", %0":"=r" (sel));   \
    tssp->usFS = sel;                                           \
  }

#define switch_to(next) do { preserve_segment (next); jmp_gate (next); } while (0)

extern void
queue_append (uint16 * queue, uint16 selector)
{

  quest_tss *tssp;

  /* NB: This code assumes atomic execution, and therefore cannot be
     called with interrupts enabled. */

  if (*queue) {
    if (*queue == selector)
      return;                   /* already on queue */

    for (tssp = lookup_TSS (*queue); tssp->next; tssp = lookup_TSS (tssp->next))
      if (tssp->next == selector)
        /* already on queue */
        return;

    /* add to end of queue */
    tssp->next = selector;

  } else
    *queue = selector;

  tssp = lookup_TSS (selector);
  tssp->next = 0;

}


extern void
runqueue_append (uint32 prio, uint16 selector)
{
  DLOG ("runqueue_append(%x, %x)", prio, selector);

  queue_append (&runqueue[prio], selector);

  BITMAP_SET (runq_bitmap, prio);
}

extern uint16
queue_remove_head (uint16 * queue)
{

  quest_tss *tssp;
  uint16 head;

  /* NB: This code assumes atomic execution, and therefore cannot be
     called with interrupts enabled. */

  if (!(head = *queue))
    return 0;

  tssp = lookup_TSS (head);

  *queue = tssp->next;

  return head;
}

extern void
sprr_wakeup (uint16 selector)
{
  quest_tss *tssp;
  tssp = lookup_TSS (selector);
  runqueue_append (tssp->priority, selector);
}

extern void
wakeup_queue (uint16 * q)
{
  uint16 head;

  while ((head = queue_remove_head (q)))
    wakeup (head);
}

uint8 sched_enabled = 0;

/* Pick from the highest priority non-empty queue */
extern void
sprr_schedule (void)
{

  uint16 next;
  unsigned int prio;

  if ((prio = bitmap_find_first_set (runq_bitmap, MAX_PRIO_QUEUES)) != -1) {    /* Got a task to execute */

    next = queue_remove_head (&runqueue[prio]);
    if (!runqueue[prio])
      BITMAP_CLR (runq_bitmap, prio);

#ifdef DEBUG_SCHED
    DLOG ("CPU %x: switching to task: %x runqueue(%x):",
          LAPIC_get_physical_ID (), next, prio);
    {                           /* print runqueue to com1 */
      quest_tss *tssp;
      int sel = runqueue[prio];
      while (sel) {
        tssp = lookup_TSS (sel);
        logger_printf (" %x", sel);
        sel = tssp->next;
      }
    }
    com1_putc ('\n');
#endif

    if (next == str ()) {
      /* no task switch required */
      return;
    }

    switch_to (next);
  } else {                      /* Replenish timeslices for expired
                                   tasks */

    /*
     * If a task calls schedule() and is selected from the runqueue,
     * then it must be switched out.  Go to IDLE task if nothing else.
     */
    uint8 phys_id = LAPIC_get_physical_ID ();
    uint16 idle_sel = idleTSS_selector[phys_id];

    DLOG ("CPU %x: idling", phys_id);

    /* Only switch tasks to IDLE if we are not already running IDLE. */
    if (str () != idle_sel)
      switch_to (idle_sel);
  }
}

/* NB: If limit is not a multiple of the system word size then all bits in
   table beyond limit must be set to zero */
static int
bitmap_find_first_set (uint32 *table, uint32 limit)
{

  int i;

  for (i = 0; i < (limit >> 5); i++)
    if (table[i])
      return ffs (table[i]) + (i << 5);

  return -1;
}

/* ************************************************** */
#ifdef MPQ

/* global queue (for unbound tasks) */

static u16 mpq_global_runqueue = 0;

/* per-cpu queues */

DEF_PER_CPU (u16, mpq_runqueue);
INIT_PER_CPU (mpq_runqueue) {
  percpu_write (mpq_runqueue, 0);
}
DEF_PER_CPU (u16, mpq_idle_task);
INIT_PER_CPU (mpq_idle_task) {
  percpu_write (mpq_idle_task, idleTSS_selector[LAPIC_get_physical_ID ()]);
}

extern void
mpq_wakeup (uint16 selector)
{
#ifdef DEBUG_SCHED
  //logger_printf ("runqueue_append(%x, %x)\n", prio, selector);
#endif

  quest_tss *tssp = lookup_TSS (selector);

  if (tssp->cpu == 0xFF) {
    queue_append (&mpq_global_runqueue, selector);
  } else {
    if (tssp->cpu != LAPIC_get_physical_ID ()) {
      queue_append (percpu_pointer (tssp->cpu, mpq_runqueue), selector);
    } else {
      u16 q = percpu_read (mpq_runqueue);
      queue_append (&q, selector);
      percpu_write (mpq_runqueue, q);
    }
  }
}

extern void
mpq_schedule (void)
{
  u16 q = percpu_read (mpq_runqueue), next;
  if (q) {
    next = queue_remove_head (&q);
    percpu_write (mpq_runqueue, q);
    DLOG ("cpu %d running task 0x%x", (u32) LAPIC_get_physical_ID (), next);
  } else if (mpq_global_runqueue) {
    next = queue_remove_head (&mpq_global_runqueue);
    quest_tss *tssp = lookup_TSS (next);
    tssp->cpu = LAPIC_get_physical_ID ();
    DLOG ("binding task 0x%x to cpu %d", next, tssp->cpu);
  } else {
    next = percpu_read (mpq_idle_task);
  }
  if (str () == next)
    return;
  else
    switch_to (next);
}

#endif
/* ************************************************** */

/* Hooks for scheduler */

#if defined(SPRR) || (!defined(MPQ) && !defined(VCPU))
void (*schedule) (void) = sprr_schedule;
void (*wakeup) (uint16) = sprr_wakeup;
#elif defined(MPQ)
void (*schedule) (void) = mpq_schedule;
void (*wakeup) (uint16) = mpq_wakeup;
#elif defined(VCPU)
extern void vcpu_schedule (void);
extern void vcpu_wakeup (u16);
void (*schedule) (void) = vcpu_schedule;
void (*wakeup) (uint16) = vcpu_wakeup;
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
