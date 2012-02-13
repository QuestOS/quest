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

#ifndef _PROC_H_
#define _PROC_H_

#include "smp/semaphore.h"
#include "util/cassert.h"
#include "types.h"

typedef union
{
  uint32 raw;
  struct {
    uint16 org_sandbox;  /* The original sandbox where this task was created */
    uint16 tid;          /* Task ID that is only unique in each sandbox but not globally */
  };
} quest_id PACKED;
CASSERT (sizeof (task_id) == sizeof (uint32), task_id);

#define NUM_M 32

/* A Quest TSS is a software-only construct, a.k.a Thread Control
 * Block (TCB). */
typedef struct _quest_tss
{
  u32 ESP;
  u32 EBP;
  /* The initial instruction pointer is written both to here and the
   * kernel stack.  In the case of the initial user-space application,
   * this field is used to load the first IRET into user-space.  In
   * all other cases, the kernel stack contains the EIP to resume
   * operation. */
  u32 initial_EIP;
  u32 CR3;
  u32 EFLAGS;
  struct _semaphore Msem;
  u32 M[NUM_M];

  task_id next;                 /* selector for next TSS in corresponding queue
                                   (beit the runqueue for the CPU or a waitqueue for
                                   a resource; 0 if task is at end of queue. If a
                                   task is runnable 'next' refers to a TSS selector
                                   on the runqueue; if a task is waiting on a
                                   resource, 'next' refers to a TSS selector on the
                                   corresponding waitqueue; for all other cases,
                                   'next' is irrelevant */
  task_id waitqueue;            /* queue of other tasks waiting for this
                                   one -- either attempting to send IPC to it,
                                   or waiting for it to exit */
  bool busy;                    /* mutex for server: when busy, clients must add themselves to
                                   waitqueue above */
  uint32 priority;
  uint64 time;                  /* A field for time values associated
                                   with task, for example, to be used
                                   by waitqueue managers. */
  u16 cpu;                      /* [V]CPU binding */
  struct _quest_tss * next_tss;
  struct _quest_tss * prev_tss;
  task_id tid;
} quest_tss;

extern quest_tss init_tss;

extern task_id
duplicate_TSS (uint32 ebp,
               uint32 *esp,
               uint32 child_eip,
               uint32 child_ebp,
               uint32 child_esp,
               uint32 child_eflags,
               uint32 child_directory);

extern quest_tss * alloc_quest_tss (void);
extern void free_quest_tss (quest_tss *tss);
extern task_id alloc_idle_TSS (int cpu_num);
extern task_id alloc_TSS (void *pPageDirectory, void *pEntry, int mod_num);
extern quest_tss *lookup_TSS (task_id tid);
extern void tss_add_head (quest_tss * new_tss);
extern void tss_add_tail (quest_tss * new_tss);
extern task_id new_task_id (void);
extern void tss_remove (task_id tid);

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
