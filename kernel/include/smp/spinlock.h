/* -*- Mode: C -*- */

#ifndef _SPINLOCK_H_
#define _SPINLOCK_H_
#include"smp/atomic.h"

//#define DEBUG_SPINLOCK
#define DEBUG_MAX_SPIN 1000000

struct _spinlock
{
  uint32 lock;
};
typedef struct _spinlock spinlock;

extern volatile bool mp_enabled;

static inline void
spinlock_lock (spinlock * lock)
{
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  uint8 LAPIC_get_physical_ID (void);

  if (mp_enabled) {
#ifdef DEBUG_SPINLOCK
    int count = 0;
    extern void panic (char *);
    extern void com1_printf (const char *, ...);
#endif
    int x = 1;
    uint32 *addr = &lock->lock;
    for (;;) {
      asm volatile ("lock xchgl %1,(%0)":"=r" (addr),
                    "=ir" (x):"0" (addr), "1" (x));
      if (x == 0)
        break;
      asm volatile ("pause");
#ifdef DEBUG_SPINLOCK
      count++;
      if (count > DEBUG_MAX_SPIN) {
        com1_printf ("DEADLOCK (CPU %d)\n", LAPIC_get_physical_ID ());
        panic ("DEADLOCK\n");
      }
#endif
    }
  }
}

static inline void
spinlock_unlock (spinlock * lock)
{
  uint32 x = 0;
  uint32 *addr = &lock->lock;
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  uint8 LAPIC_get_physical_ID (void);
  void stacktrace (void);

  asm volatile ("lock xchgl %1,(%0)":"=r" (addr), "=ir" (x):"0" (addr),
                "1" (x));
}

static inline void
spinlock_init (spinlock * lock)
{
  atomic_store_dword (&lock->lock, 0);
}

#define SPINLOCK_INIT {0}

#endif

/* vi: set et sw=2 sts=2: */
