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

#include "kernel.h"
#include "arch/i386.h"
#include "util/cpuid.h"
#include "util/debug.h"
#include "util/perfmon.h"

#define DEBUG_PERFMON

#ifdef DEBUG_PERFMON
#define DLOG(fmt,...) DLOG_PREFIX("perfmon",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* Version 1 */

static u8 bit_width, perfmon_version, num_pmcs=0;
#define IA32_FIXED_CTR_CTRL 0x38D
#define IA32_PERF_GLOBAL_STATUS 0x38E
#define IA32_PERF_GLOBAL_CTRL 0x38F
#define IA32_PERF_GLOBAL_OVF_CTRL 0x390

#define IA32_FIXED_CTR(x) (0x309 + (x))

static struct predefined_arch_perfevts {
  char *name;
  u8 event_select, unit_mask;
  bool supported;
} predefined_arch_perfevts[] = {
  { .name = "UnHalted Core Cycles",
    .unit_mask = 0x00, .event_select = 0x3C },
  { .name = "Instruction Retired",
    .unit_mask = 0x00, .event_select = 0xC0 },
  { .name = "UnHalted Reference Cycles",
    .unit_mask = 0x01, .event_select = 0x3C },
  { .name = "LLC Reference",
    .unit_mask = 0x4F, .event_select = 0x2E },
  { .name = "LLC Misses",
    .unit_mask = 0x41, .event_select = 0x2E },
  { .name = "Branch Instruction Retired",
    .unit_mask = 0x00, .event_select = 0xC4 },
  { .name = "Branch Misses Retired",
    .unit_mask = 0x00, .event_select = 0xC5 },
};
#define NUM_PREDEFINED_ARCH_PERFEVTS \
  (sizeof (predefined_arch_perfevts) / sizeof (struct predefined_arch_perfevts))

extern void
perfmon_init (void)
{
  u32 eax, ebx, edx, i, display;

  display = cpuid_display_family_model ();

  cpuid (0xA, 0, &eax, &ebx, NULL, &edx);

  if ((u8) eax == 0) {
    DLOG ("unsupported");
    return;
  }

  perfmon_version = (u8) eax;

  DLOG ("version=0x%X display family_model=0x%.08X", perfmon_version, display);

  num_pmcs = (u8) (eax >> 8);

  DLOG ("IA32 PMC range 0x%X - 0x%X",
        IA32_PMC(0),
        IA32_PMC(num_pmcs - 1));

  bit_width = (u8) (eax >> 16);

  DLOG ("bit_width=%d", bit_width);

  if (perfmon_version > 1) {
    if ((edx & 0x1F) == 0) {
      /* Quirk */
      DLOG ("assuming 3 fixed-function counters");
    } else {
      DLOG ("num fixed-function perf counters per-core=%d; bit-width=%d",
            edx & 0x1F, (u8) (edx >> 5));
    }
  }

  perfmon_pmc_config (0, 0x3C, 0);
  u64 tsc;
  RDTSC (tsc);
  DLOG ("pmc0=0x%llX tsc=0x%llX", perfmon_pmc_read (0), tsc);
  for (i=0; i<NUM_PREDEFINED_ARCH_PERFEVTS; i++) {
    if ((ebx & (1 << i)) == 0) {
      DLOG ("Predefined event \"%s\" supported", predefined_arch_perfevts[i].name);
      predefined_arch_perfevts[i].supported = TRUE;
    } else
      predefined_arch_perfevts[i].supported = FALSE;
  }
  RDTSC (tsc);
  DLOG ("pmc0=0x%llX tsc=0x%llX", perfmon_pmc_read (0), tsc);
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
