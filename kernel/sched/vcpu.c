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
#define DUMP_STATS_VERBOSE
#define CHECK_INVARIANTS

#ifdef DEBUG_VCPU
#define DLOG(fmt,...) DLOG_PREFIX("vcpu",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

u32 tsc_freq_msec;
u64 vcpu_init_time;

struct vcpu_params { vcpu_type type; u32 C, T; };
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
  uint i, n, T=0;
  for (i=0; i<NUM_VCPUS; i++) {
    if (init_params[i].type == MAIN_VCPU &&
        ( T == 0 || init_params[i].T < T )) {
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

/* task accounting */
static void
vcpu_acnt_end_timeslice (vcpu *vcpu)
{
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

static int
repl_list_size (replenishment *r)
{
  int n = 0;
  for (; r != NULL; r = r->next) n++;
  return n;
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

  logger_printf ("vcpu_dump_stats overhead=0x%llX sched=0x%X\n", overhead, stime);
#ifdef DUMP_STATS_VERBOSE
  logger_printf ("idle tsc=0x%llX%s\n", idle_time, (cur==NULL ? " (*)" : ""));
#endif
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
  u32 res = compute_percentage (now, idle_time);
  logger_printf ("summary: idle=%d.%d%%", res >> 16, res & 0xFF);
  for (i=0; i<NUM_VCPUS; i++) {
    vcpu *vcpu = &vcpus[i];
    res = compute_percentage (now, vcpu->timestamps_counted);
    logger_printf (" v%d=%d.%d%% %d", i, res >> 16, res & 0xFF,
                   vcpu->type != MAIN_VCPU ? 0 : vcpu->main.Q.size);
  }
  logger_printf ("\n");
}

static void
add_replenishment (vcpu *v, u64 b)
{
  u64 t = v->main.a + v->T;
  replenishment **r, *rnew;
  pow2_alloc (sizeof (replenishment), (u8 **) &rnew);
  if (!rnew)
    panic ("add_replenishment: out of memory");
  rnew->t = t;
  rnew->b = b;
  rnew->next = NULL;
  for (r = &v->main.R; *r != NULL; r = &(*r)->next);
  *r = rnew;
}

static void
update_replenishments (vcpu *q, u64 tcur)
{
  for (; q != NULL; q = q->next) {
    if (q->hooks->update_replenishments)
      q->hooks->update_replenishments (q, tcur);
  }
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

/* runnable budget threshold */
#define MIN_B 0

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
    /* update replenishments */
    update_replenishments (queue, tcur);

    /* pick highest priority vcpu with available budget */
    for (ptr = &queue; *ptr != NULL; ptr = &(*ptr)->next) {
      if ((*ptr)->b > MIN_B && (Tnext == 0 || (*ptr)->T < Tnext)) {
        Tnext = (*ptr)->T;
        vnext = ptr;
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
  percpu_write (pcpu_sched_time, (u32) (now - tcur));

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
  vcpu_rr_wakeup (task);
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
  repl_merge (v);
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
    u64 b = v->main.Q.head->b;
    /* observation 3 */
    if (v->main.Q.head->t + b >= v->main.Q.head->next->t) {
      repl_queue_pop (&v->main.Q);
      v->main.Q.head->b += b;
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
        u64 b = v->main.Q.head->b;
        if (v->main.Q.head->t + b >= v->main.Q.head->next->t) {
          repl_queue_pop (&v->main.Q);
          v->main.Q.head->b += b;
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
  }
}

/* invariant: sum of budget and pending replenishments must be equal
 * to C */
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
  s64 cap = capacity (cur);
  if (cap > 0) {
    /* was preempted or blocked */
    if (!cur->runnable)
      /* blocked */
      split_check (cur);
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
  if (!cur->runnable || cur->b <= MIN_B) {
    cur->io.e += (u64) div_u64_u32_u32 (cur->usage * cur->io.Uden, cur->io.Unum);
    if (cur->io.r.t == 0) {
      cur->io.r.t = cur->io.e;
      cur->io.r.b = div_u64_u32_u32 (cur->T * cur->io.Unum, cur->io.Uden);
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
  vcpu *cur = percpu_read (vcpu_current);
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
  if (v->hooks->update_replenishments)
    v->hooks->update_replenishments (v, now);

  if (v->b > MIN_B && (cur == NULL || cur->T > v->T))
    LAPIC_start_timer (1);
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
iovcpu_job_completion (void)
{
  schedule ();
}

extern uint
select_iovcpu (u32 flags)
{
  uint i;
  for (i=0; i<NUM_VCPUS; i++) {
    if (init_params[i].type == IO_VCPU)
      return i;
  }
  return lowest_priority_vcpu ();
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
