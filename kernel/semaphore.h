#ifndef _SEMAPHORE_H_
#define _SEMAPHORE_H_
#include "kernel.h"
#include "spinlock.h"

struct semaphore {
  int s, max;
  struct spinlock lock;
  unsigned short waitqueue;
};

static inline int semaphore_init(struct semaphore *sem, int max, int init) {
  sem->s = init;
  sem->max = max;
  sem->waitqueue = 0;
  spinlock_init(&sem->lock);
  return 0;
}

static inline int semaphore_signal(struct semaphore *sem, int s) {
  int status = 0;
  spinlock_lock(&sem->lock);
  sem->s += s;
  if (sem->s > sem->max) {
    sem->s = sem->max;
    status = -1;
  }
  /* wake up waiters */
  wakeup_queue(&sem->waitqueue);
  spinlock_unlock(&sem->lock);
  return status;
}

/* timeout: millisec, (-1) for indefinite */
static inline int semaphore_wait(struct semaphore *sem, int s, short timeout) {
  for(;;) {
    spinlock_lock(&sem->lock);
    if (sem->s >= s) {
      sem->s -= s;
      spinlock_unlock(&sem->lock);
      return 0;
    } else {
      spinlock_unlock(&sem->lock);
      schedule();
    }
  }
}

static inline int semaphore_destroy(struct semaphore *sem) {
  return 0;
}


#endif
