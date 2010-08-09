/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#ifndef __I386_PERPCU_H__
#define __I386_PERCPU_H__

#include "util/stringify.h"

#define PER_CPU_SEG fs
#define PER_CPU_DBG dr3
#define PER_CPU_SEG_STR __stringify(PER_CPU_SEG)
#define PER_CPU_DBG_STR __stringify(PER_CPU_DBG)

#ifndef __ASSEMBLER__           /* for C only */
#include "types.h"

/* ************************************************** */
/* API */

/* Define a per-CPU variable */
#define DEF_PER_CPU(type,var) PER_CPU_ATTR type var

/* Define an initialization function for a per-CPU variable */
#define INIT_PER_CPU(var)                               \
  void __##var##_ctor_func (void);                      \
  PER_CPU_CTOR_ATTR void (*__##var##_ctor_ptr) (void) = \
    __##var##_ctor_func;                                \
  void __##var##_ctor_func (void)

/* Access macros for per-CPU variables */
#define percpu_read(var) percpu_op_src ("mov", var, "m" (var))
#define percpu_write(var, val) percpu_op_dest ("mov", var, val)

/* Initialization for each CPU */
extern void percpu_per_cpu_init (void);

/* Get a pointer to a per-CPU variable, with explicit CPU parameter */
extern u8 *percpu_virt[];
#define percpu_pointer(cpu, var) ((void *) (&percpu_virt[cpu][(uint) &var]))

/* ************************************************** */

#define PER_CPU_ATTR __attribute__((section(".percpu")))
#define PER_CPU_CTOR_ATTR __attribute__((section(".percpu.ctor")))

#define __percpu_arg(x) "%%"PER_CPU_SEG_STR":%"#x
#define percpu_op_dest(op, var, val)                    \
  do {                                                  \
    switch (sizeof (typeof (var))) {                    \
    case 1:                                             \
      asm (op "b %1, "__percpu_arg (0)                  \
           : "+m" (var) : "qi" (val));                  \
      break;                                            \
    case 2:                                             \
      asm (op "w %1, "__percpu_arg (0)                  \
           : "+m" (var) : "ri" (val));                  \
      break;                                            \
    case 4:                                             \
      asm (op "l %1, "__percpu_arg (0)                  \
           : "+m" (var) : "ri" (val));                  \
      break;                                            \
    case 8:                                             \
      asm (op "q %1, "__percpu_arg (0)                  \
           : "+m" (var) : "re" (val));                  \
      break;                                            \
    default: panic ("percpu_op_dest: bad size");        \
    }                                                   \
  } while (0)

#define percpu_op_src(op, var, constraint)      \
  ({                                            \
    typeof(var) _percpu_ret;                    \
    switch (sizeof (typeof (var))) {            \
    case 1:                                     \
      asm (op "b "__percpu_arg (1)", %0"        \
           : "=q" (_percpu_ret) : constraint);  \
      break;                                    \
    case 2:                                     \
      asm (op "w "__percpu_arg (1)", %0"        \
           : "=r" (_percpu_ret) : constraint);  \
      break;                                    \
    case 4:                                     \
      asm (op "l "__percpu_arg (1)", %0"        \
           : "=r" (_percpu_ret) : constraint);  \
      break;                                    \
    case 8:                                     \
      asm (op "q "__percpu_arg (1)", %0"        \
           : "=r" (_percpu_ret) : constraint);  \
      break;                                    \
    default: panic ("percpu_op_src: bad size"); \
    }                                           \
    _percpu_ret;                                \
  })

#endif

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
