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

#ifndef __I386_DIV64_H__
#define __I386_DIV64_H__

#include "types.h"
#include <arch/i386.h>

/* Courtesy of Linux */

/*
 * do_div() is NOT a C function. It wants to return
 * two values (the quotient and the remainder), but
 * since that doesn't work very well in C, what it
 * does is:
 *
 * - modifies the 64-bit dividend _in_place_
 * - returns the 32-bit remainder
 *
 * This ends up being the most efficient "calling
 * convention" on x86.
 */
#define do_div(n, base)						\
({								\
  unsigned long __upper, __low, __high, __mod, __base;          \
  __base = (base);                                              \
  asm("":"=a" (__low), "=d" (__high) : "A" (n));		\
  __upper = __high;                                             \
  if (__high) {                                                 \
    __upper = __high % (__base);                                \
    __high = __high / (__base);                                 \
  }                                                             \
  asm("divl %2":"=a" (__low), "=d" (__mod)                      \
      : "rm" (__base), "0" (__low), "1" (__upper));             \
  asm("":"=A" (n) : "a" (__low), "d" (__high));                 \
  __mod;							\
 })

static inline u64
div_u64_rem(u64 dividend, u32 divisor, u32 *remainder)
{
  union {
    u64 v64;
    u32 v32[2];
  } d = { dividend };
  u32 upper;

  upper = d.v32[1];
  d.v32[1] = 0;
  if (upper >= divisor) {
    d.v32[1] = upper / divisor;
    upper %= divisor;
  }
  asm ("divl %2" : "=a" (d.v32[0]), "=d" (*remainder) :
       "rm" (divisor), "0" (d.v32[0]), "1" (upper));
  return d.v64;
}
#define div_u64_rem	div_u64_rem

/* 64bit divisor, dividend and result. dynamic precision */
static inline uint64_t
div64_64 (uint64_t dividend, uint64_t divisor)
{
  uint32_t high, d;

  high = divisor >> 32;
  if (high) {
    unsigned int shift = fls(high);

    d = divisor >> shift;
    dividend >>= shift;
  } else
    d = divisor;

  do_div(dividend, d);

  return dividend;
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
