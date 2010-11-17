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
#include "arch/i386-div64.h"
#include "kernel.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/printf.h"
#include "sched/sched.h"

/* Scheduler-integrated blocking sleep routines */

//#define DEBUG_SCHED_SLEEP

#ifdef DEBUG_SCHED_SLEEP
#define DLOG(fmt,...) DLOG_PREFIX("sched-sleep",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static task_id sleepqueue = 0;

extern uint64 tsc_freq;         /* timestamp counter frequency */

static inline uint64
compute_finish (uint32 usec)
{
  uint64 ticks;
  uint64 start;

  RDTSC (start);

  ticks = div64_64 (tsc_freq * (u64) usec, 1000000LL);

  return start + ticks;
}

/* Must hold lock */
extern void
sched_usleep (uint32 usec)
{
  if (mp_enabled) {
    task_id sel;
    quest_tss *tssp;
    uint64 finish = compute_finish (usec);
#ifdef DEBUG_SCHED_SLEEP
    u64 now; RDTSC (now);
#endif
    sel = str ();
    DLOG ("task 0x%x sleeping for %d usec (0x%llX -> 0x%llX)",
          sel, usec, now, finish);
    tssp = lookup_TSS (sel);
    tssp->time = finish;
    queue_append (&sleepqueue, sel);

    schedule ();
  } else
    /* interrupts not enabled */
    tsc_delay_usec (usec);
}

/* Spin for a given amount of microsec. */
extern void
tsc_delay_usec (uint32 usec)
{
  uint64 value, finish;

  finish = compute_finish (usec);
  for (;;) {
    RDTSC (value);
    if (value >= finish)
      break;
    asm volatile ("pause");
  }
}

/* Must hold lock */
extern void
process_sleepqueue (void)
{
  uint64 now;
  task_id *q, next;
  quest_tss *tssp;

  RDTSC (now);

  if (sleepqueue == 0)
    return;

  q = &sleepqueue;
  tssp = lookup_TSS (sleepqueue);
  DLOG ("process_sleepqueue 0x%llX", now);
  for (;;) {
    /* examine finish time of task */
    next = tssp->next;

    if (tssp->time <= now) {
      DLOG ("waking task 0x%x (0x%llX <= 0x%llX)", *q, tssp->time, now);
      /* time to wake-up */
      wakeup (*q);
      /* remove from sleepqueue */
      *q = next;
      tssp->time = 0;
    } else
      q = &tssp->next;

    /* move to next sleeper */
    if (next == 0)
      break;
    tssp = lookup_TSS (*q);
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
