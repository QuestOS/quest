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

#include "arch/i386.h"
#include "arch/i386-percpu.h"
#include "kernel.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/debug.h"
#include "sched/sched.h"

#define QUEST_SCHED vcpu        /* Use the VCPU scheduler */

//#define DEBUG_SCHED
#ifdef DEBUG_SCHED
#define DLOG(fmt,...) DLOG_PREFIX("sched",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* ************************************************** */
uint8 sched_enabled = 0;

/* These functions assume exclusive access to the queue. */
extern void
queue_append (quest_tss **queue, quest_tss *selector)
{

  quest_tss *tssp;

  if (*queue) {
    if (*queue == selector)
      return;                   /* already on queue */

    for (tssp = *queue; tssp->next; tssp = tssp->next) {
      if (tssp->next == selector) {
        /* already on queue */
        return;
      }
    }

    /* add to end of queue */
    tssp->next = selector;

  }
  else {
    *queue = selector;
  }
  

  selector->next = NULL;

}

extern quest_tss *
queue_remove_head (quest_tss **queue)
{

  quest_tss *head;

  if (!(head = *queue))
    return NULL;

  *queue = head->next;

  return head;
}

extern void
wakeup_queue (quest_tss **q)
{
  quest_tss *head;

  while ((head = queue_remove_head (q)))
    wakeup (head);
}

/* ************************************************** */

DEF_PER_CPU (quest_tss *, current_task);
INIT_PER_CPU (current_task) {
  percpu_write (current_task, NULL);
}

/* Hooks for scheduler */

#define ___glue(a,b) a##b
#define __glue(a,b) ___glue(a,b)
#define S __glue(QUEST_SCHED,_schedule)
#define W __glue(QUEST_SCHED,_wakeup)
extern void S (void);
void (*schedule) (void) = S;
extern void W (quest_tss *);
void (*wakeup) (quest_tss *) = W;

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
