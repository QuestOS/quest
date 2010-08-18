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

#ifndef _PERFMON_H_
#define _PERFMON_H_

#include "kernel.h"

/* Performance monitoring counter MSRs */
#define IA32_PMC(x) (0xC1 + (x))
/* Performance event select MSRs paired with PMCs */
#define IA32_PERFEVTSEL(x) (0x186 + (x))

typedef union {
  u32 raw;
  struct {
    u8 event_select;
    u8 unit_mask;
    u8 user:1;
    u8 os:1;
    u8 edge_detect:1;
    u8 pin_control:1;
    u8 int_enable:1;
    u8 _reserved:1;
    u8 enable_counters:1;
    u8 invert_counter_mask:1;
    u8 counter_mask;
  } PACKED;
} ia32_perfevtsel_t;

static inline void
perfmon_pmc_config (int x, u8 event_select, u8 unit_mask)
{
  ia32_perfevtsel_t conf;

  conf.raw = 0;
  conf.event_select = event_select;
  conf.unit_mask = unit_mask;
  conf.enable_counters = 1;
  conf.user = conf.os = 1;

  wrmsr (IA32_PERFEVTSEL (x), conf.raw);
}

static inline u64
perfmon_pmc_read (int x)
{
  return rdmsr (IA32_PMC (x));
}

extern void perfmon_init (void);
extern bool perfmon_enabled;

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
