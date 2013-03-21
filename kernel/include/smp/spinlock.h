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

#ifndef _SPINLOCK_H_
#define _SPINLOCK_H_
#include "smp/atomic.h"
#include "include/arch/i386.h"

//#define DEBUG_SPINLOCK
#define DEBUG_MAX_SPIN 100000000

struct _spinlock
{
  volatile uint32 lock;
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

#ifdef DEBUG_SPINLOCK
  int count = 0;
  extern void panic (char *);
  extern void com1_printf (const char *, ...);
#endif
  int x = 1;
  volatile uint32 *addr = &lock->lock;
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
      com1_printf ("DEADLOCK on lock located at 0x%p", lock);
      panic ("DEADLOCK\n");
    }
#endif
  }
}

static u32 get_flags()
{
  u32 flags;
  asm volatile("pushf ; pop %0"
               : "=rm" (flags)
               : /* no input */
               : "memory");
  
  return flags;
}



static inline u32 WARN_UNUSED_RESULT
_spinlock_lock_irq_save(spinlock* lock)
{
  u32 flags = get_flags();
  cli();
  spinlock_lock(lock);
  return flags;
}

#define spinlock_lock_irq_save(lock, flags)     \
  do {                                          \
    flags = _spinlock_lock_irq_save(lock);      \
  } while(0)


static inline bool spinlock_attempt_lock(spinlock * lock)
{
  if (mp_enabled) {

    int x = 1;
    volatile uint32 *addr = &lock->lock;
    asm volatile ("lock xchgl %1,(%0)":"=r" (addr),
                  "=ir" (x):"0" (addr), "1" (x));
    
    return x == 0;
  }
  return TRUE;
}

static inline void
spinlock_unlock (spinlock * lock)
{
  uint32 x = 0;
  volatile uint32 *addr = &lock->lock;
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  uint8 LAPIC_get_physical_ID (void);
  void stacktrace (void);

  asm volatile ("lock xchgl %1,(%0)":"=r" (addr), "=ir" (x):"0" (addr),
                "1" (x));
}

static inline void restore_flags(u32 flags)
{
  asm volatile("push %0 ; popf"
               : /* no output */
               :"g" (flags)
               :"memory", "cc");
}

static inline void spinlock_unlock_irq_restore(spinlock* lock, u32 flags)
{
  spinlock_unlock(lock);
  restore_flags(flags);
}

static inline void
spinlock_init (spinlock * lock)
{
  atomic_store_dword (&lock->lock, 0);
}

#define SPINLOCK_INIT {0}

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
