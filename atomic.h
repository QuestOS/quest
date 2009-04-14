#ifndef _ATOMIC_H_
#define _ATOMIC_H_

#include "types.h"

static inline DWORD atomic_load_dword(DWORD *addr) {
  return *((volatile DWORD *)addr);
}

static inline void atomic_store_dword(DWORD *addr, DWORD x) {
  *((volatile DWORD *)addr) = x;
}

static inline DWORD atomic_xchg_dword(DWORD *addr, DWORD x) {
  asm volatile("xchgl %1,(%0)" : "=r"(addr), "=ir"(x) : "0"(addr), "1"(x));
  return x;
}

#endif
