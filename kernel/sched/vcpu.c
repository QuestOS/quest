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

#ifdef DEBUG_VCPU
#define DLOG(fmt,...) DLOG_PREFIX("vcpu",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

u32 tsc_freq_msec, tsc_lapic_factor;
u64 vcpu_init_time;

#define NUM_VCPUS 5
static vcpu vcpus[NUM_VCPUS] ALIGNED (VCPU_ALIGNMENT);
static struct { vcpu_type type; u32 C, T; } init_params[NUM_VCPUS] = {
  { MAIN_VCPU, 1, 3 },
  { MAIN_VCPU, 1, 5 },
  { MAIN_VCPU, 2, 10 },
  { MAIN_VCPU, 1, 50 },
  { IO_VCPU, 1, 50 },
};

uint lowest_priority_vcpu = NUM_VCPUS - 2;

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
}

/* ************************************************** */

#define FUDGE_FACTOR 0x40000LL

DEF_PER_CPU (u64, pcpu_tprev);
INIT_PER_CPU (pcpu_tprev) {
  percpu_write64 (pcpu_tprev, 0LL);
}
DEF_PER_CPU (s64, pcpu_overhead);
INIT_PER_CPU (pcpu_overhead) {
  percpu_write64 (pcpu_overhead, 0LL);
}
DEF_PER_CPU (u64, pcpu_overhead_fudge);
INIT_PER_CPU (pcpu_overhead_fudge) {
  percpu_write64 (pcpu_overhead_fudge, FUDGE_FACTOR);
}
DEF_PER_CPU (u64, pcpu_idle_time);
INIT_PER_CPU (pcpu_idle_time) {
  percpu_write64 (pcpu_idle_time, 0LL);
}
DEF_PER_CPU (u64, pcpu_idle_prev_tsc);
INIT_PER_CPU (pcpu_idle_prev_tsc) {
  percpu_write64 (pcpu_idle_prev_tsc, 0LL);
}

static void
idle_time_begin ()
{
  u64 now;

  RDTSC (now);
  percpu_write64 (pcpu_idle_prev_tsc, now);
}

static void
idle_time_end ()
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
  u64 overhead_fudge = percpu_read64 (pcpu_overhead_fudge);
  u64 idle_time = percpu_read64 (pcpu_idle_time);
  u64 sum = idle_time;
  logger_printf ("vcpu_dump_stats overhead=0x%llX fudge=0x%llX\n", overhead, overhead_fudge);
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
      logger_printf ("  b=0x%llX overhead=0x%llX delta=0x%llX count=0x%X\n",
                     vcpu->b, vcpu->sched_overhead, vcpu->prev_delta,
                     vcpu->prev_count);
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
    logger_printf (" vcpu%d=%d.%d%%", i, res >> 16, res & 0xFF);
  }
  logger_printf ("\n");
}

static void
add_replenishment (vcpu *v, u64 b)
{
  u64 t = v->a + v->T;
  replenishment **r, *rnew;
  pow2_alloc (sizeof (replenishment), (u8 **) &rnew);
  if (!rnew)
    panic ("add_replenishment: out of memory");
  rnew->t = t;
  rnew->b = b;
  rnew->next = NULL;
  for (r = &v->R; *r != NULL; r = &(*r)->next);
  *r = rnew;
}

static void
update_replenishments_vcpu (vcpu *v, u64 tcur)
{
  replenishment **r, *temp;
  for (r = &v->R; *r != NULL; r = &(*r)->next) {
  do_r:
    if ((*r)->t <= tcur) {
      DLOG ("update_replenishments: vcpu=%p replenishing 0x%llX budget",
            v, (*r)->b);
      if (v->type != IO_VCPU)
        v->a = tcur;          /* set activation time */
      v->b += (*r)->b;        /* increase budget */
      temp = *r;
      *r = (*r)->next;
      pow2_free ((u8 *) temp);
      if (*r) goto do_r; else break;
    }
  }
}

static void
update_replenishments (vcpu *q, u64 tcur)
{
  for (; q != NULL; q = q->next) {
    update_replenishments_vcpu (q, tcur);
  }
}

static void
check_activations (u64 tcur, u64 Tprev, u64 Tnext)
{
  /* FIXME: make per-cpu */
  int i;
  for (i=0; i<NUM_VCPUS; i++) {
    if (vcpus[i].type == MAIN_VCPU &&
        vcpus[i].b > 0 && Tnext <= vcpus[i].T && vcpus[i].T < Tprev)
      vcpus[i].a = tcur;
  }
}

/* runnable budget threshold */
#define MIN_B 50

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
  s64 overhead = percpu_read64 (pcpu_overhead);
  u64 overhead_fudge = percpu_read64 (pcpu_overhead_fudge);
  bool timer_set = FALSE;

  RDTSC (tcur);

  tdelta = tcur - tprev;

  DLOG ("tcur=0x%llX tprev=0x%llX tdelta=0x%llX", tcur, tprev, tdelta);

  if (cur) {
    u64 u;

    Tprev = cur->T;

    /* handle end-of-timeslice accounting */
    vcpu_acnt_end_timeslice (cur);

    //logger_printf ("tdelta=0x%llX cur->b=0x%llX\n", tdelta, cur->b);
    /* subtract from budget of current */
    if (cur->b < tdelta) {
      u = cur->b;
      cur->sched_overhead = tdelta - cur->b;
      overhead = cur->sched_overhead;
      //if (overhead > MIN_B) overhead_fudge += (overhead>>3);
      percpu_write64 (pcpu_overhead, overhead);
      percpu_write64 (pcpu_overhead_fudge, overhead_fudge);
    } else
      u = tdelta;

    cur->b -= u;
    DLOG ("budget: vcpu=%p used=0x%llX replenish@=0x%llX", cur, u,
          tprev + cur->T);

    if (cur->type == IO_VCPU) {
      if (cur->state == IO_VCPU_JOB_COMPLETE || cur->b <= MIN_B) {
        cur->a += (u64) div_u64_u32_u32 (tsc_freq_msec * cur->Uden, cur->Unum) >> 1;
        cur->b = 0;
        if (cur->state != IO_VCPU_JOB_COMPLETE) {
          add_replenishment (cur, div_u64_u32_u32 (cur->T * cur->Unum, cur->Uden));
        } else {
          cur->state = IO_VCPU_JOB_INCOMPLETE;
          add_replenishment (cur, div_u64_u32_u32 (cur->T * cur->Unum, cur->Uden));
        }
      }
    } else {
      /* schedule replenishment of used budget */
      add_replenishment (cur, u);
    }
  } else idle_time_end ();

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
      if (vcpu->runqueue == 0)
        /* otherwise, remove it */
        *vnext = (*vnext)->next;
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
        replenishment *r;
        for (r = (*ptr)->R; r != NULL; r = r->next) {
          if (tdelta == 0 || r->t - tcur < tdelta) {
            tdelta = r->t - tcur;
          }
        }
      }
    }

    /* set timer */
    if (tdelta > 0) {
      u64 o = tdelta;
#if 1
      if (overhead_fudge > tsc_lapic_factor) {
        if (tdelta <= overhead_fudge + tsc_lapic_factor)
          tdelta = tsc_lapic_factor;
        else
          tdelta -= overhead_fudge;
      }
#endif
      /* consider upper-bounding tdelta by QUANTUM ms? */
      u32 count = div_u64_u32_u32 (tdelta, tsc_lapic_factor);
      if (vcpu) {
        vcpu->prev_delta = tdelta;
        vcpu->prev_count = count;
      }
      //if (cur) logger_printf (" (%llX, %llX) %llX / %X = %X\n", o, overhead_fudge, tdelta, tsc_lapic_factor, count);
      DLOG ("start_timer: count=0x%x", count);
      LAPIC_start_timer (count);
      timer_set = TRUE;
    }
  }

  check_activations (tcur, Tprev, Tnext);

  if (!timer_set)
    LAPIC_start_timer (cpu_bus_freq / QUANTUM_HZ);

  if (vcpu)
    /* handle beginning-of-timeslice accounting */
    vcpu_acnt_begin_timeslice (vcpu);
  else
    idle_time_begin ();
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

extern void
iovcpu_job_wakeup (task_id job, u64 T)
{
  quest_tss *tssp = lookup_TSS (job);
  vcpu *v, *cur = percpu_read (vcpu_current);
  u64 now;

  wakeup (job);
  v = vcpu_lookup (tssp->cpu);
  if (v->type != IO_VCPU)
    return;
  RDTSC (now);

  v->T = T;
  if ((v->state == IO_VCPU_JOB_INCOMPLETE) && ((v->a + v->T) < now))
    v->a = now - v->T;

  u64 Cmax = div_u64_u32_u32 (T * v->Unum, v->Uden);

  if (v->R == NULL) {
    if (v->state != IO_VCPU_BUDGETED)
      add_replenishment (v, Cmax);
  } else {
    v->R->b = Cmax;
  }
  v->state = IO_VCPU_BUDGETED;
  update_replenishments_vcpu (v, now);

  if (v->b > MIN_B && (cur == NULL || cur->T > T))
    LAPIC_start_timer (1);
}

extern void
iovcpu_job_completion (void)
{
  vcpu *cur = percpu_read (vcpu_current);

  if (cur->type != IO_VCPU) {
    schedule ();
    return;
  }

  cur->state = IO_VCPU_JOB_COMPLETE;

  schedule ();
}

/* ************************************************** */

extern void
vcpu_init (void)
{
  uint ecx;
  cpuid (1, 0, NULL, NULL, &ecx, NULL);

  logger_printf ("vcpu: init num_vcpus=%d num_cpus=%d tsc_deadline=%s\n",
                 NUM_VCPUS, mp_num_cpus,
                 (ecx & (1 << 24)) ? "yes" : "no");

  memset (vcpus, 0, sizeof(vcpus));

  int cpu_i=0, vcpu_i;
  vcpu *vcpu;

  RDTSC (vcpu_init_time);
  tsc_freq_msec = div_u64_u32_u32 (tsc_freq, 1000);
  if (tsc_freq <= cpu_bus_freq)
    /* workaround for bochs where this might hold true */
    tsc_lapic_factor = 1;
  else
    tsc_lapic_factor = div_u64_u32_u32 (tsc_freq, cpu_bus_freq);
  logger_printf ("vcpu: tsc_freq_msec=0x%x tsc_lapic_factor=0x%x\n",
        tsc_freq_msec,
        tsc_lapic_factor);

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
    vcpu->C = vcpu->b = C * tsc_freq_msec;
    vcpu->T = T * tsc_freq_msec;
    vcpu->type = type;
    logger_printf ("vcpu: %svcpu=%d pcpu=%d C=0x%llX T=0x%llX U=%d%%\n",
                   type == IO_VCPU ? "IO " : "",
                   vcpu_i, vcpu->cpu, vcpu->C, vcpu->T, (C * 100) / T);
    if (type == IO_VCPU) {
      vcpu->Unum = C;
      vcpu->Uden = T;
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
