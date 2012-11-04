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

#ifndef _SCHED_H_
#define _SCHED_H_

#include "kernel.h"
#include "arch/i386-percpu.h"
#include "sched/sched-defs.h"

extern void runqueue_append (uint32 prio, task_id selector);
extern void queue_append (task_id * queue, task_id selector);
extern task_id queue_remove_head (task_id * queue);
extern void (*schedule) (void);
extern void (*wakeup) (task_id);
extern void wakeup_queue (task_id *);

extern void sched_usleep (uint32);
extern void process_sleepqueue (void);

extern DEF_PER_CPU (task_id, current_task);
static inline task_id
str (void)
{
  return percpu_read (current_task);
}
static inline void
ltr (task_id id)
{
  percpu_write (current_task, id);
}

static inline void
software_context_switch (task_id next)
{
  task_id tr = percpu_read (current_task);
  quest_tss *cur_TSS = (tr == 0 ? NULL : lookup_TSS (tr));
  quest_tss *nxt_TSS = lookup_TSS (next);

  percpu_write (current_task, next);
  
  asm volatile ("call _sw_jmp_task":
                :"S" (cur_TSS), "D" (nxt_TSS)
                :"eax", "ebx", "ecx", "edx");
  
}


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
