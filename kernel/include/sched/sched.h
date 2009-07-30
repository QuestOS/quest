#ifndef _SCHED_H_
#define _SCHED_H_

#define MAX_PRIO_QUEUES 32      /* NOTE: linux uses 140 */
#define MIN_PRIO ( MAX_PRIO_QUEUES - 1 )

extern void runqueue_append (uint32 prio, uint16 selector);
extern void queue_append (uint16 * queue, uint16 selector);
extern uint16 queue_remove_head (uint16 * queue);
extern void schedule (void);
extern void wakeup (uint16);
extern void wakeup_queue (uint16 *);


#endif
