/* -*- Mode: C -*- */

#ifndef _ATOMIC_H_
#define _ATOMIC_H_

#include "types.h"

static inline uint32
atomic_load_dword (uint32 * addr)
{
  return *((volatile uint32 *) addr);
}

static inline void
atomic_store_dword (uint32 * addr, uint32 x)
{
  *((volatile uint32 *) addr) = x;
}

static inline uint32
atomic_xchg_dword (uint32 * addr, uint32 x)
{
  asm volatile ("xchgl %1,(%0)":"=r" (addr), "=ir" (x):"0" (addr), "1" (x));
  return x;
}

#endif
