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

#ifndef _SCHED_VCPU_H_
#define _SCHED_VCPU_H_

#include "kernel.h"
#include "arch/i386-percpu.h"
#include "util/cassert.h"

#define VCPU_ALIGNMENT (LOCK_ALIGNMENT<<3)

/* If BEST_EFFORT_VCPU, sched_param, vcpu_type or iovcpu_class are
   changed they must also be changed in libc's vcpu.h */

/* Moving BEST_EFFORT_VCPU to kernel.h */
//#define BEST_EFFORT_VCPU 0


typedef enum {
  MAIN_VCPU = 0, IO_VCPU
} vcpu_type;

typedef enum {
  IOVCPU_CLASS_ALL = 0,
  IOVCPU_CLASS_ATA = (1<<0),
  IOVCPU_CLASS_NET = (1<<1),
  IOVCPU_CLASS_USB = (1<<2),
  IOVCPU_CLASS_DISK = (1<<3),
  IOVCPU_CLASS_CDROM = (1<<4),
} iovcpu_class;


struct sched_param
{
  int sched_priority;

  vcpu_type type;
  iovcpu_class io_class;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
  int affinity;                 /* CPU (or Quest-V sandbox affinity) */
  int machine_affinity;         /* -- EM -- Machine affinity hack
                                   right now is a just a bool to
                                   indicate stay (0) or move to other
                                   machine (1) */
};

#ifndef _SCHED_VCPU_REPL_
#define _SCHED_VCPU_REPL_
typedef struct _replenishment {
  u64 t, b;
  struct _replenishment *next;
} replenishment;
#define MAX_REPL 32
#endif

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
      task_id tr;               /* task register */
      task_id runqueue;         /* per-VCPU runqueue */
      u32 quantum;              /* internal VCPU scheduling quantum */
      u64 next_schedule;        /* when to trigger internal schedule */
      u64 prev_tsc;             /* when started running */
      u64 virtual_tsc;          /* virtual timestamp counter */
      int index;

      u64 C, T, b, usage;       /* common scheduling parameters */
      u32 _C, _T;               /* Original C and T before frequency computation */

      /* type-specific parameters */
      union {
        /* MAIN_VCPU */
        struct {
          u64 a;                /* activation time */
          repl_queue Q;
        } main;
        /* IO_VCPU */
        struct {
          u64 e;                /* eligibility time */
          replenishment r;      /* replenishment */
          u32 Unum, Uden;       /* utilization fraction */
          bool budgeted;
          iovcpu_class class;
        } io;
      };

      /* statistics tracking */
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

      /* Cache accounting */
      u64 acnt_tsc;
      u64 cache_occupancy;
      u64 mpki;
      u64 prev_local_miss;
      u64 prev_global_miss;
      u64 prev_inst_ret;
    };
    u8 raw[VCPU_ALIGNMENT];     /* pad to VCPU_ALIGNMENT */
  };
} vcpu;
CASSERT (sizeof (vcpu) == VCPU_ALIGNMENT, vcpu);

extern u64 vcpu_current_vtsc (void);

extern void iovcpu_job_wakeup (task_id job, u64 T);
extern void iovcpu_job_wakeup_for_me (task_id job);
extern void iovcpu_job_completion (void);

extern int create_vcpu(struct sched_param* params, vcpu** vcpu_p);
extern int create_main_vcpu(int C, int T, vcpu** vcpu_p);

extern uint lowest_priority_vcpu (void);
extern uint select_iovcpu (iovcpu_class);
extern void set_iovcpu (task_id, iovcpu_class);
extern vcpu * vcpu_lookup (int);
extern bool vcpu_in_runqueue (vcpu *, task_id);
extern void vcpu_remove_from_runqueue (vcpu *, task_id);
extern bool vcpu_fix_replenishment (quest_tss*, vcpu*, replenishment[], bool remote_tsc_diff,
                                    uint64 remote_tsc);
#ifdef USE_VMX
extern bool validate_migration_condition (quest_tss*);
extern void vcpu_reset (void);
#endif

extern DEF_PER_CPU (vcpu*, vcpu_current);
extern DEF_PER_CPU (vcpu*, vcpu_queue);

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
