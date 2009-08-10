/* -*- Mode: C -*- */

#ifndef _SCHED_H_
#define _SCHED_H_

#include "sched/sched-defs.h"

extern void runqueue_append (uint32 prio, uint16 selector);
extern void queue_append (uint16 * queue, uint16 selector);
extern uint16 queue_remove_head (uint16 * queue);
extern void schedule (void);
extern void wakeup (uint16);
extern void wakeup_queue (uint16 *);


#endif
