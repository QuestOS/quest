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
#include "arch/i386-div64.h"
#include "util/perfmon.h"
#include "util/cpuid.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "smp/spinlock.h"
#include "util/debug.h"
#include "util/printf.h"
#include "mem/pow2.h"

//#define DEBUG_VCPU
//#define DUMP_STATS_VERBOSE
//#define CHECK_INVARIANTS

#ifdef DEBUG_VCPU
#define DLOG(fmt,...) DLOG_PREFIX("vcpu",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

u32 tsc_freq_msec;
u64 vcpu_init_time;

struct vcpu_params { vcpu_type type; u32 C, T; iovcpu_class class; };
static struct vcpu_params init_params[] = {
  { MAIN_VCPU, 1, 3 },
  { MAIN_VCPU, 1, 5 },
  { MAIN_VCPU, 2, 10 },
  { MAIN_VCPU, 1, 50 },
  { IO_VCPU, 1, 50 },
};
#define NUM_VCPUS (sizeof (init_params) / sizeof (struct vcpu_params))
static vcpu vcpus[NUM_VCPUS] ALIGNED (VCPU_ALIGNMENT);

extern uint
lowest_priority_vcpu (void)
{
  uint i, n=0, T=0;
  for (i=0; i<NUM_VCPUS; i++) {
    if (init_params[i].type == MAIN_VCPU && init_params[i].T >= T) {
      T = init_params[i].T;
      n = i;
    }
  }
  return n;
}

vcpu *
vcpu_lookup (int i)
{
  if (0 <= i && i < NUM_VCPUS)
    return &vcpus[i];
  return NULL;
}

int
vcpu_index (vcpu *v)
{
  return (((uint) v) - ((uint) &vcpus)) / sizeof (vcpu);
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
  u64 now = vcpu->virtual_tsc;

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

/* task accounting */
static void
vcpu_acnt_end_timeslice (vcpu *vcpu)
{
  u64 now;
  int i;

  RDTSC (now);

  if (vcpu->prev_tsc) {
    vcpu->timestamps_counted += now - vcpu->prev_tsc;
    vcpu->virtual_tsc += now - vcpu->prev_tsc;
  }

  for (i=0; i<2; i++) {
    u64 value = perfmon_pmc_read (i);
    if (vcpu->prev_pmc[i])
      vcpu->pmc_total[i] += value - vcpu->prev_pmc[i];
  }
}

static void
vcpu_acnt_begin_timeslice (vcpu *vcpu)
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
    vcpu_acnt_end_timeslice (cur);
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
    vcpu_acnt_begin_timeslice (vcpu);
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
  static int next_vcpu_binding = 1;

  if (tssp->cpu == 0xFF) {
    do {
      tssp->cpu = next_vcpu_binding;
      next_vcpu_binding++;
      if (next_vcpu_binding >= NUM_VCPUS)
        next_vcpu_binding = 0;
    } while  (vcpu_lookup (tssp->cpu)->type != MAIN_VCPU);
    logger_printf ("vcpu: task 0x%x now bound to vcpu=%d\n", task, tssp->cpu);
  }

  vcpu *vcpu = vcpu_lookup (tssp->cpu);

  /* put task on vcpu runqueue (2nd level) */
  vcpu_runqueue_append (vcpu, task);

  /* put vcpu on pcpu queue (1st level) */
  vcpu_queue_append (percpu_pointer (vcpu->cpu, vcpu_queue), vcpu);

  if (!vcpu->runnable && !vcpu->running && vcpu->hooks->unblock)
    vcpu->hooks->unblock (vcpu);
  vcpu->runnable = TRUE;
}

/* ************************************************** */

DEF_PER_CPU (u64, pcpu_tprev);
INIT_PER_CPU (pcpu_tprev) {
  percpu_write64 (pcpu_tprev, 0LL);
}
DEF_PER_CPU (s64, pcpu_overhead);
INIT_PER_CPU (pcpu_overhead) {
  percpu_write64 (pcpu_overhead, 0LL);
}
DEF_PER_CPU (u64, pcpu_idle_time);
INIT_PER_CPU (pcpu_idle_time) {
  percpu_write64 (pcpu_idle_time, 0LL);
}
DEF_PER_CPU (u64, pcpu_idle_prev_tsc);
INIT_PER_CPU (pcpu_idle_prev_tsc) {
  percpu_write64 (pcpu_idle_prev_tsc, 0LL);
}
DEF_PER_CPU (u32, pcpu_sched_time);
INIT_PER_CPU (pcpu_sched_time) {
  percpu_write (pcpu_sched_time, 0);
}

static void
idle_time_acnt_begin ()
{
  u64 now;

  RDTSC (now);
  percpu_write64 (pcpu_idle_prev_tsc, now);
}

static void
idle_time_acnt_end ()
{
  u64 idle_time = percpu_read64 (pcpu_idle_time);
  u64 idle_prev_tsc = percpu_read64 (pcpu_idle_prev_tsc);
  u64 now;

  RDTSC (now);

  if (idle_prev_tsc)
    idle_time += now - idle_prev_tsc;
  percpu_write64 (pcpu_idle_time, idle_time);
}

static inline u32
compute_percentage (u64 overall, u64 usage)
{
  u64 res = div64_64 (usage * 1000, overall);
  u16 whole, frac;

  whole = ((u16) res) / 10;
  frac = ((u16) res) - whole * 10;
  return (((u32) whole) << 16) | (u32) frac;
}

extern void
vcpu_dump_stats (void)
{
  int i;
#ifdef DUMP_STATS_VERBOSE
  vcpu *cur = percpu_read (vcpu_current);
#endif
  s64 overhead = percpu_read64 (pcpu_overhead);
  u64 idle_time = percpu_read64 (pcpu_idle_time);
  u64 sum = idle_time;
  u32 stime = percpu_read (pcpu_sched_time);
  extern u32 uhci_sample_bps (void);
  extern u32 atapi_sample_bps (void);
  u32 uhci_bps = uhci_sample_bps ();
  u32 atapi_bps = atapi_sample_bps ();
  logger_printf ("vcpu_dump_stats overhead=0x%llX"
                 " sched=0x%X uhci_bps=%d atapi_bps=%d\n",
                 overhead, stime, uhci_bps, atapi_bps);
#ifdef DUMP_STATS_VERBOSE
  logger_printf ("idle tsc=0x%llX%s\n", idle_time, (cur==NULL ? " (*)" : ""));
#endif

  percpu_write64 (pcpu_idle_time, 0LL);
  percpu_write (pcpu_sched_time, 0);

  for (i=0; i<NUM_VCPUS; i++) {
    vcpu *vcpu = &vcpus[i];
#ifdef DUMP_STATS_VERBOSE
    if (vcpu->type == IO_VCPU) {
      logger_printf ("vcpu=%d pcpu=%d tsc=0x%llX pmc[0]=0x%llX pmc[1]=0x%llX%s\n",
                     i, vcpu->cpu,
                     vcpu->timestamps_counted,
                     vcpu->pmc_total[0],
                     vcpu->pmc_total[1],
                     (vcpu == cur ? " (*)" : ""));
      logger_printf ("  b=0x%llX overhead=0x%llX delta=0x%llX usage=0x%X\n",
                     vcpu->b, vcpu->sched_overhead, vcpu->prev_delta,
                     vcpu->prev_usage);
    }
#endif
    sum += vcpu->timestamps_counted;
  }
  u64 now; RDTSC (now);
  now -= vcpu_init_time;
  RDTSC (vcpu_init_time);
  u32 res = compute_percentage (now, idle_time);
  logger_printf (" idle=%02d.%d\n", res >> 16, res & 0xFF);
  for (i=0; i<NUM_VCPUS; i++) {
    vcpu *vcpu = &vcpus[i];
    res = compute_percentage (now, vcpu->timestamps_counted);
    vcpu->timestamps_counted = 0;
    logger_printf (" V%02d=%02d.%d %d", i, res >> 16, res & 0xFF,
                   vcpu->type != MAIN_VCPU ? 0 : vcpu->main.Q.size);
    if ((i % 4) == 3) {
      logger_printf ("\n");
    }
  }
  logger_printf ("\n");
}

/* ************************************************** */

void
repl_queue_pop (repl_queue *Q)
{
  if (Q->head) {
    replenishment *r = Q->head->next;
    Q->head->t = Q->head->b = 0;
    Q->head->next = NULL;
    Q->head = r;
    Q->size--;
  }
}

void
repl_queue_add (repl_queue *Q, u64 b, u64 t)
{
  if (Q->size < MAX_REPL) {
    replenishment *r = NULL, **rq;
    int i;
    for (i=0; i<MAX_REPL; i++) {
      if (Q->array[i].t == 0) {
        r = &Q->array[i];
        break;
      }
    }
    if (!r) panic ("Q->size < MAX_REPL but no free entry");
    rq = &Q->head;
    /* find insertion point */
    while (*rq && (*rq)->t < t)
      rq = &(*rq)->next;
    /* insert */
    r->next = *rq;
    *rq = r;
    r->t = t;
    r->b = b;
    Q->size++;
  }
}

/* ************************************************** */

#ifdef CHECK_INVARIANTS
static void
check_run_invariants (void)
{
  vcpu
    *queue = percpu_read (vcpu_queue),
    *cur   = percpu_read (vcpu_current),
    *v, *q;
  int i;
  if (cur && !cur->running) panic ("current is not running");
  for (i=0; i<NUM_VCPUS; i++) {
    v = &vcpus[i];
    if (v->running && v != cur)
      panic ("vcpu running is not current");
    if (v->runnable) {
      for (q = queue; q; q = q->next) {
        if (q == v) goto ok;
      }
      panic ("vcpu runnable is not on queue");
    ok:;
    } else {
      for (q = queue; q; q = q->next) {
        if (q == v) panic ("vcpu not runnable is on queue");
      }
    }
    if (v->type == MAIN_VCPU) {
      if (v->main.Q.size >= MAX_REPL-1)
        logger_printf ("vcpu %d has %d repls\n", i, v->main.Q.size);
      u64 sum = 0;
      replenishment *r;
      for (r = v->main.Q.head; r != NULL; r = r->next)
        sum += r->b;
      if (sum != v->C) {
        com1_printf ("v->C=0x%llX sum=0x%llX\n", v->C, sum);
        panic ("vcpu replenishments out of whack");
      }
    }
  }
}
#endif

extern void
vcpu_schedule (void)
{
  task_id next = 0;
  vcpu
    *queue = percpu_read (vcpu_queue),
    *cur   = percpu_read (vcpu_current),
    *vcpu  = NULL,
    **ptr,
    **vnext = NULL;
  u64 tprev = percpu_read64 (pcpu_tprev);
  u64 tcur, tdelta, Tprev = 0, Tnext = 0;
  bool timer_set = FALSE;

#ifdef CHECK_INVARIANTS
  check_run_invariants ();
#endif

  RDTSC (tcur);

  tdelta = tcur - tprev;

  DLOG ("tcur=0x%llX tprev=0x%llX tdelta=0x%llX", tcur, tprev, tdelta);

  if (cur) {
    Tprev = cur->T;

    /* handle end-of-timeslice accounting */
    vcpu_acnt_end_timeslice (cur);

    /* invoke VCPU-specific end of timeslice budgeting */
    if (cur->hooks->end_timeslice)
      cur->hooks->end_timeslice (cur, tdelta);
  } else idle_time_acnt_end ();

  if (queue) {
    /* pick highest priority vcpu with available budget */
    for (ptr = &queue; *ptr != NULL; ptr = &(*ptr)->next) {
      if (Tnext == 0 || (*ptr)->T < Tnext) {
        /* update replenishments to refresh budget */
        if ((*ptr)->hooks->update_replenishments)
          (*ptr)->hooks->update_replenishments (*ptr, tcur);
        if ((*ptr)->b > 0) {
          Tnext = (*ptr)->T;
          vnext = ptr;
        }
      }
    }

    if (vnext) {
      vcpu = *vnext;
      /* internally schedule */
      vcpu_internal_schedule (vcpu);
      /* keep vcpu on queue if it has other runnable tasks */
      if (vcpu->runqueue == 0) {
        /* otherwise, remove it */
        *vnext = (*vnext)->next;
        vcpu->runnable = FALSE;
      }
      next = vcpu->tr;
      percpu_write (vcpu_current, vcpu);
      percpu_write (vcpu_queue, queue);
      DLOG ("scheduling vcpu=%p with budget=0x%llX", vcpu, vcpu->b);
    } else {
      percpu_write (vcpu_current, NULL);
    }

    /* find time of next important event */
    if (vcpu)
      tdelta = vcpu->b;
    else
      tdelta = 0;
    for (ptr = &queue; *ptr != NULL; ptr = &(*ptr)->next) {
      if (Tnext == 0 || (*ptr)->T <= Tnext) {
        u64 event = 0;
        if ((*ptr)->hooks->next_event)
          event = (*ptr)->hooks->next_event (*ptr);
        if (event != 0 && (tdelta == 0 || event - tcur < tdelta))
          tdelta = event - tcur;
      }
    }

    /* set timer */
    if (tdelta > 0) {
      u32 count = (u32) div64_64 (tdelta * ((u64) cpu_bus_freq), tsc_freq);
      if (count == 0)
        count = 1;
      if (count > cpu_bus_freq / QUANTUM_HZ)
        count = cpu_bus_freq / QUANTUM_HZ;
      if (vcpu) {
        vcpu->prev_delta = tdelta;
        vcpu->prev_count = count;
      }
      LAPIC_start_timer (count);
      timer_set = TRUE;
    }
  }

  if (!timer_set)
    LAPIC_start_timer (cpu_bus_freq / QUANTUM_HZ);

  if (vcpu)
    /* handle beginning-of-timeslice accounting */
    vcpu_acnt_begin_timeslice (vcpu);
  else
    idle_time_acnt_begin ();
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

  /* current time becomes previous time */
  percpu_write64 (pcpu_tprev, tcur);

  /* measure schedule running time */
  u64 now; RDTSC (now);
  u32 sched_time = percpu_read (pcpu_sched_time);
  percpu_write (pcpu_sched_time, sched_time + (u32) (now - tcur));

  if (cur) cur->running = FALSE;
  if (vcpu) vcpu->running = TRUE;

  /* switch to new task or continue running same task */
  if (str () == next)
    return;
  else
    switch_to (next);
}

extern void
vcpu_wakeup (task_id task)
{
  DLOG ("vcpu_wakeup (0x%x), cpu=%d", task, LAPIC_get_physical_ID ());
  quest_tss *tssp = lookup_TSS (task);
  static int next_vcpu_binding = 1;

  /* assign vcpu if not already set */
  if (tssp->cpu == 0xFF) {
    do {
      tssp->cpu = next_vcpu_binding;
      next_vcpu_binding++;
      if (next_vcpu_binding >= NUM_VCPUS)
        next_vcpu_binding = 0;
    } while  (vcpu_lookup (tssp->cpu)->type != MAIN_VCPU);
    com1_printf ("vcpu: task 0x%x now bound to vcpu=%d\n", task, tssp->cpu);
  }

  vcpu *v = vcpu_lookup (tssp->cpu);

  /* put task on vcpu runqueue (2nd level) */
  vcpu_runqueue_append (v, task);

  /* put vcpu on pcpu queue (1st level) */
  vcpu_queue_append (percpu_pointer (v->cpu, vcpu_queue), v);

  if (!v->runnable && !v->running && v->hooks->unblock)
    v->hooks->unblock (v);

  v->runnable = TRUE;

  /* check if preemption necessary */
  u64 now;
  vcpu *cur = percpu_read (vcpu_current);

  RDTSC (now);

  if (v->hooks->update_replenishments)
    v->hooks->update_replenishments (v, now);

  if (v->b > 0 && (cur == NULL || cur->T > v->T))
    /* preempt */
    LAPIC_start_timer (1);
}

/* ************************************************** */

/* MAIN_VCPU */

static inline s64
capacity (vcpu *v)
{
  u64 now; RDTSC (now);
  if (v->main.Q.head == NULL || v->main.Q.head->t > now)
    return 0;
  return (s64) v->main.Q.head->b - (s64) v->usage;
}

static void repl_merge (vcpu *);

static void
main_vcpu_update_replenishments (vcpu *v, u64 tcur)
{
  s64 cap = capacity (v);
  v->b = (cap > 0 ? cap : 0);
}

static u64
main_vcpu_next_event (vcpu *v)
{
  replenishment *r;
  u64 now; RDTSC (now);
  for (r = v->main.Q.head; r != NULL; r = r->next) {
    if (now < r->t)
      return r->t;
  }
  return 0;
}

static void
repl_merge (vcpu *v)
{
  /* possibly merge */
  while (v->main.Q.size > 1) {
    u64 t = v->main.Q.head->t;
    u64 b = v->main.Q.head->b;
    /* observation 3 */
    if (t + b >= v->main.Q.head->next->t) {
      repl_queue_pop (&v->main.Q);
      v->main.Q.head->b += b;
      v->main.Q.head->t = t;
    } else
      break;
  }
}

static void
budget_check (vcpu *v)
{
  if (capacity (v) <= 0) {
    while (v->main.Q.head->b <= v->usage) {
      /* exhaust and reschedule the replenishment */
      v->usage -= v->main.Q.head->b;
      u64 b = v->main.Q.head->b, t = v->main.Q.head->t;
      repl_queue_pop (&v->main.Q);
      t += v->T;
      repl_queue_add (&v->main.Q, b, t);
    }
    if (v->usage > 0) {
      /* v->usage is the overrun amount */
      v->main.Q.head->t += v->usage;
      /* possibly merge */
      if (v->main.Q.size > 1) {
        u64 t = v->main.Q.head->t;
        u64 b = v->main.Q.head->b;
        if (t + b >= v->main.Q.head->next->t) {
          repl_queue_pop (&v->main.Q);
          v->main.Q.head->b += b;
          v->main.Q.head->t = t;
        }
      }
    }
#if 0
    if (capacity (v) == 0) {
      /* S.Q.head.time > Now */
      /* if not blocked then S.replenishment.enqueue (S.Q.head.time) */
    }
#endif
  }
}

static void
split_check (vcpu *v)
{
  u64 now; RDTSC (now);
  if (v->usage > 0 && v->main.Q.head->t <= now) {
    u64 remnant = v->main.Q.head->b - v->usage;
    if (v->main.Q.size == MAX_REPL) {
      /* merge with next replenishment */
      repl_queue_pop (&v->main.Q);
      v->main.Q.head->b += remnant;
    } else {
      /* leave remnant as reduced replenishment */
      v->main.Q.head->b = remnant;
    }
    repl_queue_add (&v->main.Q, v->usage, v->main.Q.head->t + v->T);
    /* invariant: sum of replenishments remains the same */
    v->usage = 0;
  }
}

static void
main_vcpu_end_timeslice (vcpu *cur, u64 tdelta)
{
  /* timeslice ends for one of 3 reasons: budget depletion,
   * preemption, or blocking */

  if (cur->b < tdelta) {
    cur->sched_overhead = tdelta - cur->b;
    percpu_write64 (pcpu_overhead, cur->sched_overhead);
  }

  cur->usage += tdelta;
  budget_check (cur);
  if (!cur->runnable)
    /* blocked */
    split_check (cur);

  s64 cap = capacity (cur);
  if (cap > 0) {
    /* was preempted or blocked */
    cur->b = cap;
  } else {
    /* budget was depleted */
    cur->b = 0;
    cur->prev_usage = cur->usage;
  }
}

static void
unblock_check (vcpu *v)
{
  if (capacity (v) > 0) {
    u64 now;
    RDTSC (now);
    v->main.Q.head->t = now;
    /* merge replenishments using observation 3 */
    while (v->main.Q.size > 1) {
      u64 b = v->main.Q.head->b;
      if (v->main.Q.head->next->t <= now + b - v->usage) {
        repl_queue_pop (&v->main.Q);
        v->main.Q.head->b += b;
        v->main.Q.head->t = now;
      } else
        break;
    }
  } else {
    /* S.replenishment.enqueue (S.Q.head.time) */
  }
}

static void
main_vcpu_unblock (vcpu *v)
{
  unblock_check (v);
}

static vcpu_hooks main_vcpu_hooks = {
  .update_replenishments = main_vcpu_update_replenishments,
  .next_event = main_vcpu_next_event,
  .end_timeslice = main_vcpu_end_timeslice,
  .unblock = main_vcpu_unblock
};

/* IO_VCPU */

static void
io_vcpu_update_replenishments (vcpu *v, u64 tcur)
{
  if (v->io.r.t != 0 && v->io.r.t <= tcur) {
    v->b = v->io.r.b;
    v->io.r.t = 0;
  }
}

static u64
io_vcpu_next_event (vcpu *v)
{
  return v->io.r.t;
}

static void
io_vcpu_end_timeslice (vcpu *cur, u64 tdelta)
{
  u64 u = tdelta;
  s64 overhead = percpu_read64 (pcpu_overhead);

  /* subtract from budget of current */
  if (cur->b < tdelta) {
    u = cur->b;
    cur->sched_overhead = tdelta - cur->b;
    overhead = cur->sched_overhead;
    percpu_write64 (pcpu_overhead, overhead);
  }

  cur->b -= u;

  cur->usage += tdelta;
  if (!cur->runnable || cur->b <= 0) {
    cur->io.e += div64_64 (cur->usage * cur->io.Uden, cur->io.Unum);
    if (cur->io.r.t == 0) {
      cur->io.r.t = cur->io.e;
      cur->io.r.b = div64_64 (cur->T * cur->io.Unum, cur->io.Uden);
    } else {
      cur->io.r.t = cur->io.e;
    }
    cur->prev_usage = cur->usage;
    cur->usage = 0;
    if (cur->runnable)
      cur->b = 0;
    else
      cur->io.budgeted = FALSE;
  }
}

extern void
io_vcpu_unblock (vcpu *v)
{
  u64 now;
  RDTSC (now);

  if (!v->io.budgeted && v->io.e < now)
    v->io.e = now;

  u64 Cmax = div_u64_u32_u32 (v->T * v->io.Unum, v->io.Uden);

  if (v->io.r.t == 0) {
    if (!v->io.budgeted) {
      v->io.r.t = v->io.e;
      v->io.r.b = Cmax;
    }
  } else {
    v->io.r.b = Cmax;
  }
  v->io.budgeted = TRUE;
}

extern void
iovcpu_job_wakeup (task_id job, u64 T)
{
  quest_tss *tssp = lookup_TSS (job);
  vcpu *v = vcpu_lookup (tssp->cpu);
  if (v->type != IO_VCPU)
    return;
  v->T = T;
  wakeup (job);
}

extern void
iovcpu_job_wakeup_for_me (task_id job)
{
  vcpu *cur = percpu_read (vcpu_current);
  if (cur)
    iovcpu_job_wakeup (job, cur->T);
  else
    wakeup (job);
}

extern void
iovcpu_job_completion (void)
{
  schedule ();
}

extern uint
count_set_bits (u32 v)
{
  /* Wegner's method */
  u32 c;
  for (c = 0; v; c++) {
    v &= v - 1;              /* clear the least significant bit set */
  }
  return c;
}

extern uint
select_iovcpu (iovcpu_class class)
{
  uint i, idx = lowest_priority_vcpu (), matches = 0;
  for (i=0; i<NUM_VCPUS; i++) {
    struct vcpu_params *p = &init_params[i];
    if (p->type == IO_VCPU) {
      u32 m = count_set_bits (p->class & class);
      if (m >= matches) {
        idx = i;
        matches = m;
      }
    }
  }
  return idx;
}

extern void
set_iovcpu (task_id task, iovcpu_class class)
{
  uint i = select_iovcpu (class);
  logger_printf ("iovcpu: task 0x%x requested class 0x%x and got IO-VCPU %d\n",
                 task, class, i);
  lookup_TSS (task)->cpu = i;
}

static vcpu_hooks io_vcpu_hooks = {
  .update_replenishments = io_vcpu_update_replenishments,
  .next_event = io_vcpu_next_event,
  .end_timeslice = io_vcpu_end_timeslice,
  .unblock = io_vcpu_unblock
};

/* ************************************************** */

static vcpu_hooks *vcpu_hooks_table[] = {
  [MAIN_VCPU] = &main_vcpu_hooks,
  [IO_VCPU] = &io_vcpu_hooks
};

extern void
vcpu_init (void)
{
  uint eax, ecx;

  cpuid (1, 0, NULL, NULL, &ecx, NULL);
  cpuid (6, 0, &eax, NULL, NULL, NULL);

  logger_printf ("vcpu: init num_vcpus=%d num_cpus=%d TSC_deadline=%s ARAT=%s\n",
                 NUM_VCPUS, mp_num_cpus,
                 (ecx & (1 << 24)) ? "yes" : "no",
                 (eax & (1 << 2))  ? "yes" : "no");

  memset (vcpus, 0, sizeof(vcpus));

  int cpu_i=0, vcpu_i;
  vcpu *vcpu;

  RDTSC (vcpu_init_time);
  tsc_freq_msec = div_u64_u32_u32 (tsc_freq, 1000);
  logger_printf ("vcpu: tsc_freq_msec=0x%X\n", tsc_freq_msec);

  /* distribute VCPUs across PCPUs */
  for (vcpu_i=0; vcpu_i<NUM_VCPUS; vcpu_i++) {
    u32 C = init_params[vcpu_i].C;
    u32 T = init_params[vcpu_i].T;
    vcpu_type type = init_params[vcpu_i].type;
    vcpu = vcpu_lookup (vcpu_i);
    vcpu->cpu = cpu_i++;
    if (cpu_i >= mp_num_cpus)
      cpu_i = 0;
    vcpu->quantum = div_u64_u32_u32 (tsc_freq, QUANTUM_HZ);
    vcpu->C = C * tsc_freq_msec;
    vcpu->T = T * tsc_freq_msec;
    vcpu->type = type;
    if (vcpu->type == MAIN_VCPU) {
      repl_queue_add (&vcpu->main.Q, vcpu->C, vcpu_init_time);
    } else if (vcpu->type == IO_VCPU) {
      vcpu->io.Unum = C;
      vcpu->io.Uden = T;
      vcpu->b = vcpu->C;
    }
    vcpu->hooks = vcpu_hooks_table[type];
    logger_printf ("vcpu: %svcpu=%d pcpu=%d C=0x%llX T=0x%llX U=%d%%\n",
                   type == IO_VCPU ? "IO " : "",
                   vcpu_i, vcpu->cpu, vcpu->C, vcpu->T, (C * 100) / T);
    if (type == IO_VCPU) {
    }
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
