#ifndef _SPINLOCK_H_
#define _SPINLOCK_H_
#include"atomic.h"

struct spinlock {
  DWORD lock;
};

extern volatile int mp_enabled;

static inline void spinlock_lock(struct spinlock *lock) {
  if (mp_enabled) {
    while (atomic_xchg_dword(&lock->lock, 1) == 1) {
      asm volatile("pause");
    }
  }
}

static inline void spinlock_unlock(struct spinlock *lock) {
  atomic_store_dword(&lock->lock, 0);
}

static inline void spinlock_init(struct spinlock *lock) {
  atomic_store_dword(&lock->lock, 0);
}

#define SPINLOCK_INIT {0}

#endif
