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

#ifndef _ATOMIC_H_
#define _ATOMIC_H_

#include "types.h"

/* Intel Manual 3A 8.10.6.7 recommends no more than one lock or
 * semaphore be present within each 128-byte aligned block of memory,
 * to reduce bus traffic. */
#define LOCK_ALIGNMENT_LOG2 7
#define LOCK_ALIGNMENT (1<<LOCK_ALIGNMENT_LOG2)

typedef struct { volatile int counter; } atomic_t;
#define ATOMIC_T_INIT    {0}

static inline uint32
atomic_load_dword (volatile uint32 * addr)
{
  return *((volatile uint32 *) addr);
}

static inline void
atomic_store_dword (volatile uint32 * addr, uint32 x)
{
  *((volatile uint32 *) addr) = x;
}

static inline uint32
atomic_xchg_dword (volatile uint32 * addr, uint32 x)
{
  asm volatile ("xchgl %1,(%0)":"=r" (addr), "=ir" (x):"0" (addr), "1" (x));
  return x;
}

static inline void
atomic_inc (atomic_t *v)
{
  asm volatile (
      "lock incl %0"
      :"=m" (v->counter)
      :"m" (v->counter));
}

static inline void
atomic_dec (atomic_t *v)
{
  asm volatile (
      "lock decl %0"
      :"=m" (v->counter)
      :"m" (v->counter));
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
