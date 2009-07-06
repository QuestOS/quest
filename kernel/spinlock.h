#ifndef _SPINLOCK_H_
#define _SPINLOCK_H_
#include"atomic.h"

struct spinlock {
  DWORD lock;
};

extern volatile int mp_enabled;

static inline void spinlock_lock(struct spinlock *lock) {
  extern void com1_putc(char);
  extern void com1_puts(char *);
  extern void com1_putx(unsigned long);
  BYTE LAPIC_get_physical_ID(void);

  if (mp_enabled) {
    int x = 1;
    DWORD *addr = &lock->lock;
    for (;;) {
      asm volatile("lock xchgl %1,(%0)" : "=r"(addr), "=ir"(x) : "0"(addr), "1"(x));
      if (x == 0) break;
      asm volatile("pause");
    }
  }

#if 0
  if((unsigned)lock == 0xFFC10048)
    com1_putc('L');
#endif

#if 0
  com1_puts("LC"); com1_putx(LAPIC_get_physical_ID());
  com1_puts("L");
  com1_putx((unsigned)lock);
  com1_putc('\n');
#endif
}

static inline void spinlock_unlock(struct spinlock *lock) {
  int x = 0;
  DWORD *addr = &lock->lock;
  extern void com1_putc(char);
  extern void com1_puts(char *);
  extern void com1_putx(unsigned long);
  BYTE LAPIC_get_physical_ID(void);
  void stacktrace(void);
     
#if 0
  if((unsigned)lock == 0xFFC10048)
    com1_putc('U');
#endif

#if 0
  com1_puts("UC"); com1_putx(LAPIC_get_physical_ID());
  com1_puts("L");
  com1_putx((unsigned)lock);
  com1_putc('\n');
#endif

  asm volatile("lock xchgl %1,(%0)" : "=r"(addr), "=ir"(x) : "0"(addr), "1"(x));
  
#if 0
  if(x == 0 && ((unsigned)lock == 0xFFC10048) && LAPIC_get_physical_ID() == 1) {
    /* unlocked an unlocked lock */
    com1_putc('\n');
    stacktrace();
  }
#endif

#if 0
  if(x == 0 && ((unsigned)lock == 0xFFC10048)) {
    /* unlocked an unlocked lock */
    com1_putc('!');
  }
#endif
}

static inline void spinlock_init(struct spinlock *lock) {
  atomic_store_dword(&lock->lock, 0);
}

#define SPINLOCK_INIT {0}

#endif
