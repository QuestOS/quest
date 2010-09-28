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

#ifndef _SCHED_VCPU_H_
#define _SCHED_VCPU_H_

#include "kernel.h"
#include "util/cassert.h"

#define VCPU_ALIGNMENT (LOCK_ALIGNMENT<<2)

typedef enum {
  MAIN_VCPU = 0, IO_VCPU
} vcpu_type;

typedef struct _replenishment {
  u64 t, b;
  struct _replenishment *next;
} replenishment;

#define MAX_REPL 8
typedef struct {
  replenishment array[MAX_REPL];
  replenishment *head;
  u32 size;
} repl_queue;
void repl_queue_pop (repl_queue *Q);
void repl_queue_add (repl_queue *Q, u64 b, u64 t);

struct _vcpu;
typedef struct {
  void (*update_replenishments) (struct _vcpu *, u64 tcur);
  u64  (*next_event) (struct _vcpu *);
  void (*end_timeslice) (struct _vcpu *, u64 delta);
  void (*unblock) (struct _vcpu *);
} vcpu_hooks;

/* Virtual CPU */
typedef struct _vcpu
{
  union {
    struct {
      spinlock lock;
      vcpu_type type;
      vcpu_hooks *hooks;
      struct _vcpu *next;       /* next vcpu in a queue */
      bool runnable, running;
      u16 cpu;                  /* cpu affinity for vcpu */
      u16 tr;                   /* task register */
      u16 runqueue;             /* per-VCPU runqueue */
      u32 quantum;              /* internal VCPU scheduling quantum */
      u64 next_schedule;        /* when to trigger internal schedule */

      u64 C, T, b, usage;       /* common scheduling parameters */

      /* type-specific parameters */
      union {
        /* MAIN_VCPU */
        struct {
          u64 a;                /* activation time */
          replenishment *R;     /* replenishment list */
          repl_queue Q;
        } main;
        /* IO_VCPU */
        struct {
          u64 e;                /* eligibility time */
          replenishment r;      /* replenishment */
          u32 Unum, Uden;       /* utilization fraction */
          bool budgeted;
        } io;
      };

      u64 prev_tsc;
      u64 timestamps_counted;
      u64 prev_pmc[2];
      u64 pmc_total[2];
      u64 local_miss_count;     /* incl. pre-fetches */
      u64 global_miss_count;    /* 0x09, 0x03 UNC_L3_MISS.ANY (Neh.) */
      u64 sched_overflow;
      u64 sched_overhead;
      u64 prev_usage;
      u64 prev_delta;
      u32 prev_count;
    };
    u8 raw[VCPU_ALIGNMENT];     /* pad to VCPU_ALIGNMENT */
  };
} vcpu;
CASSERT (sizeof (vcpu) == VCPU_ALIGNMENT, vcpu);

extern void iovcpu_job_wakeup (task_id job, u64 T);
extern void iovcpu_job_wakeup_for_me (task_id job);
extern void iovcpu_job_completion (void);

extern uint lowest_priority_vcpu (void);
extern uint select_iovcpu (u32);

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
