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

#ifndef _SCHED_VCPU_H_
#define _SCHED_VCPU_H_

#include "kernel.h"

#define VCPU_ALIGNMENT LOCK_ALIGNMENT
/* Virtual CPU */
typedef struct _vcpu
{
  union {
    struct {
      spinlock lock;
      struct _vcpu *next;       /* next vcpu in a queue */
      u16 cpu;                  /* cpu affinity for vcpu */
      u16 tr;                   /* task register */
      u16 runqueue;             /* per-VCPU runqueue */
      u64 prev_tsc;
      u64 timestamps_counted;
      u64 prev_pmc[2];
      u64 pmc_total[2];
      u64 local_miss_count;     /* incl. pre-fetches */
      u64 global_miss_count;    /* 0x09, 0x03 UNC_L3_MISS.ANY (Neh.) */
    };
    u8 raw[VCPU_ALIGNMENT];     /* pad to VCPU_ALIGNMENT */
  };
} vcpu;

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
