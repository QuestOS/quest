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

/* ************************************************** */

#include "sched/vcpu.h"
#include "sched/sched.h"
#include "arch/i386-percpu.h"
#include "util/perfmon.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "smp/spinlock.h"
#include "util/debug.h"

//#define DEBUG_VCPU

#ifdef DEBUG_VCPU
#define DLOG(fmt,...) DLOG_PREFIX("vcpu",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define NUM_VCPUS 8
static vcpu vcpus[NUM_VCPUS] ALIGNED (VCPU_ALIGNMENT);

vcpu *
vcpu_lookup (int i)
{
  if (0 <= i && i < NUM_VCPUS)
    return &vcpus[i];
  return NULL;
}

void
vcpu_lock (vcpu *vcpu)
{
  spinlock_lock (&vcpu->lock);
}

void
vcpu_unlock (vcpu *vcpu)
{
  spinlock_unlock (&vcpu->lock);
}

/* locked functions */

bool
vcpu_in_runqueue (vcpu *vcpu, task_id task)
{
  task_id i = vcpu->runqueue;

  while (i != 0) {
    if (task == i) return TRUE;
    i = lookup_TSS (i)->next;
  }
  return task == i;
}

void
vcpu_remove_from_runqueue (vcpu *vcpu, task_id task)
{
  task_id *q = &vcpu->runqueue;
  while (*q != 0) {
    if (*q == task) {
      *q = lookup_TSS (*q)->next;
      return;
    }
    q = &lookup_TSS (*q)->next;
  }
}

void
vcpu_internal_schedule (vcpu *vcpu)
{
  u64 now;
  RDTSC (now);

  if (vcpu->next_schedule == 0 || vcpu->next_schedule <= now)
    goto sched;

  if (vcpu_in_runqueue (vcpu, vcpu->tr) == TRUE) {
    /* keep vcpu->tr running, remove from runqueue */
    vcpu_remove_from_runqueue (vcpu, vcpu->tr);
  } else
    goto sched;

  return;

 sched:
  /* round-robin */
  vcpu->tr = queue_remove_head (&vcpu->runqueue);
  vcpu->next_schedule = now + vcpu->quantum;
}

void
vcpu_runqueue_append (vcpu *vcpu, task_id task)
{
  queue_append (&vcpu->runqueue, task);
}

#define preserve_segment(next)                                  \
  {                                                             \
    tss *tssp = (tss *)lookup_TSS (next);                       \
    u16 sel;                                                    \
    asm volatile ("movw %%"PER_CPU_SEG_STR", %0":"=r" (sel));   \
    tssp->usFS = sel;                                           \
  }

#define switch_to(next) software_context_switch (next)

void
vcpu_switch_to (vcpu *vcpu)
{
  switch_to (vcpu->tr);
}

bool
vcpu_is_idle (vcpu *vcpu)
{
  return vcpu->tr == 0;
}

void
vcpu_queue_append (vcpu **queue, vcpu *vcpu)
{
  while (*queue) {
    if (*queue == vcpu)
      /* already on queue */
      return;
    queue = &((*queue)->next);
  }
  vcpu->next = NULL;
  *queue = vcpu;
}

vcpu *
vcpu_queue_remove_head (vcpu **queue)
{
  vcpu *vcpu = NULL;
  if (*queue) {
    vcpu = *queue;
    *queue = (*queue)->next;
  }
  return vcpu;
}

/* ************************************************** */

DEF_PER_CPU (vcpu *, vcpu_queue);
INIT_PER_CPU (vcpu_queue) {
  percpu_write (vcpu_queue, NULL);
}

DEF_PER_CPU (vcpu *, vcpu_current);
INIT_PER_CPU (vcpu_current) {
  percpu_write (vcpu_current, NULL);
}

DEF_PER_CPU (task_id, vcpu_idle_task);
INIT_PER_CPU (vcpu_idle_task) {
  percpu_write (vcpu_idle_task, 0);
}

/* end of timeslice */
static void
vcpu_acnt_before_switch (vcpu *vcpu)
{
  static int tick = 0;
  u64 now;
  int i;

  RDTSC (now);

  if (vcpu->prev_tsc)
    vcpu->timestamps_counted += now - vcpu->prev_tsc;

  for (i=0; i<2; i++) {
    u64 value = perfmon_pmc_read (i);
    if (vcpu->prev_pmc[i])
      vcpu->pmc_total[i] += value - vcpu->prev_pmc[i];
  }

  if (tick++ > 1021) {
    for (i=0; i<NUM_VCPUS; i++) {
      vcpu = &vcpus[i];
      logger_printf ("vcpu=%p (%d) tsc=0x%llX pmc[0]=0x%llX pmc[1]=0x%llX\n",
                     vcpu, vcpu->cpu,
                     vcpu->timestamps_counted,
                     vcpu->pmc_total[0],
                     vcpu->pmc_total[1]);
      tick = 0;
    }
  }
}

/* beginning of timeslice */
static void
vcpu_acnt_after_switch (vcpu *vcpu)
{
  u64 now;
  int i;

  RDTSC (now);
  vcpu->prev_tsc = now;

  for (i=0; i<2; i++) {
    u64 value = perfmon_pmc_read (i);
    vcpu->prev_pmc[i] = value;
  }
}

extern void
vcpu_rr_schedule (void)
{
  task_id next = 0;
  vcpu
    *queue = percpu_read (vcpu_queue),
    *cur   = percpu_read (vcpu_current),
    *vcpu  = NULL;

  if (cur)
    /* handle end-of-timeslice accounting */
    vcpu_acnt_before_switch (cur);
  if (queue) {
    /* get next vcpu from queue */
    vcpu = vcpu_queue_remove_head (&queue);
    /* perform 2nd-level scheduling */
    vcpu_internal_schedule (vcpu);
    next = vcpu->tr;
    /* if vcpu still has a runqueue, put it back on 1st-level queue */
    if (vcpu->runqueue)
      vcpu_queue_append (&queue, vcpu);
    percpu_write (vcpu_queue, queue);
    percpu_write (vcpu_current, vcpu);
    DLOG ("vcpu_schedule: pcpu=%d vcpu=%p vcpu->tr=0x%x ->runqueue=0x%x next=0x%x",
          LAPIC_get_physical_ID (), vcpu, vcpu->tr, vcpu->runqueue, next);
  }
  if (vcpu)
    /* handle beginning-of-timeslice accounting */
    vcpu_acnt_after_switch (vcpu);
  if (next == 0) {
    /* no task selected, go idle */
    next = percpu_read (vcpu_idle_task);
    percpu_write (vcpu_current, NULL);
  }
  if (next == 0) {
    /* workaround: vcpu_idle_task was not initialized yet */
    next = idleTSS_selector[LAPIC_get_physical_ID ()];
    percpu_write (vcpu_idle_task, next);
  }

  /* switch to new task or continue running same task */
  if (str () == next)
    return;
  else
    switch_to (next);
}

extern void
vcpu_rr_wakeup (task_id task)
{
  DLOG ("vcpu_wakeup (0x%x), cpu=%d", task, LAPIC_get_physical_ID ());
  quest_tss *tssp = lookup_TSS (task);
  static int next_vcpu_binding = 0;

  if (tssp->cpu == 0xFF) {
    tssp->cpu = next_vcpu_binding;
    next_vcpu_binding++;
    if (next_vcpu_binding >= NUM_VCPUS)
      next_vcpu_binding = 0;
  }

  vcpu *vcpu = vcpu_lookup (tssp->cpu);

  /* put task on vcpu runqueue (2nd level) */
  vcpu_runqueue_append (vcpu, task);

  /* put vcpu on pcpu queue (1st level) */
  vcpu_queue_append (percpu_pointer (vcpu->cpu, vcpu_queue), vcpu);
}

extern void
vcpu_init (void)
{
  DLOG ("init num_vcpus=%d num_cpus=%d", NUM_VCPUS, mp_num_cpus);
  memset (vcpus, 0, sizeof(vcpus));

  int cpu_i=0, vcpu_i;
  vcpu *vcpu;

  /* distribute VCPUs across PCPUs */
  for (vcpu_i=0; vcpu_i<NUM_VCPUS; vcpu_i++) {
    vcpu = vcpu_lookup (vcpu_i);
    vcpu->cpu = cpu_i++;
    if (cpu_i >= mp_num_cpus)
      cpu_i = 0;
    vcpu->quantum = div_u64_u32_u32 (tsc_freq, QUANTUM_HZ);
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
