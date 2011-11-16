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
#include "arch/i386-percpu.h"
#include "mem/mem.h"
#include "arch/i386-div64.h"

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

#define IA32_LEVEL_CACHES  0x5

/* --??-- For effeciency, PMC0, PMC1 and UNCORE_PMC0 are reserved */
/* This should be replaced later with a PMC allocator system.     */

/* Statically allocate 2 general PMC's and 1 UNCORE PMC for accounting */
#define PERFMON_LM_PMC    0
#define PERFMON_GM_PMC    0   /* UNCORE PC Counter needs to be enabled */
#define PERFMON_IR_PMC    1

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

static struct cache_info {
  uint64 cache_size;  /* Total cache size in bytes */
  u8 num_apic;        /* # of APIC IDs reserved for this package */
  uint num_thread;    /* # of threads sharing this cache */
  bool fully_assoc;   /* Fully associative? */
  u8 self_init_level; /* Self initialising cache level */
  u8 cache_level;     /* Cache level */
  u8 cache_type;      /* 0 - Null, 1 - Data, 2 - Instruction, 3 - Unified */
  uint associativity; /* Ways of associativity */
  uint line_part;     /* Physical line partition */
  uint line_size;     /* System coherency line size */
  uint sets;          /* # of sets */
  bool inclusive;     /* Cache inclusive to lower cache level? */
  u8 invd;
} cache_info [IA32_LEVEL_CACHES];

bool perfmon_enabled = FALSE;
bool nehalem_perfmon_enabled = FALSE;
bool westmere_perfmon_enabled = FALSE;

uint llc_lines = 0; /* Total number of lines in last level cache */
uint llc_line_size = 0; /* Last level cache line size */

/* x specifies which IA32_PERFEVTSEL and IA32_PMC msr pair to use.
 * rsp specifies which MSR_OFFCORE_RSP msr to use.
 * Only MSR_OFFCORE_RSP0 is available in Nehalem.
 * Westmere has both MSR_OFFCORE_RSP0 and MSR_OFFCORE_RSP1.
 */
extern void
offcore_perfmon_pmc_config (int x, int rsp, uint64 offcore_evts)
{
  switch (rsp) {
    case 0 :
      if (nehalem_perfmon_enabled || westmere_perfmon_enabled) {
        wrmsr (MSR_OFFCORE_RSP (0), offcore_evts);
        perfmon_pmc_config (x, OFFCORE_RSP0_EVT, OFFCORE_RSP_MASK);
      } else {
        DLOG ("Off-Core Response Event is not supported");
      }
      break;

    case 1 :
      if (westmere_perfmon_enabled) {
        wrmsr (MSR_OFFCORE_RSP (1), offcore_evts);
        perfmon_pmc_config (x, OFFCORE_RSP1_EVT, OFFCORE_RSP_MASK);
      } else {
        DLOG ("MSR_OFFCORE_RSP1 is only available on Westmere");
      }
      break;

    default :
      DLOG ("At most 2 off-core response msr's are supported");
  }
}

/* Get detailed cache information from the current processor by using
 * CPUID.4H. Cache information is stored in cache_info list declared
 * above.
 */
static
void perfmon_get_cache_info (void)
{
  u32 eax = 0, ebx = 0, ecx = 0, edx = 0;
  int level = 0 ,i = 0;

  for (i = 0; i < IA32_LEVEL_CACHES; i++) {
    cpuid (0x4, i, &eax, &ebx, &ecx, &edx);
    //DLOG ("eax=0x%X, ebx=0x%x, ecx=0x%x, edx=0x%x", eax, ebx, ecx, edx);
    if (!(eax & 0x1F)) {
      llc_lines = cache_info [i-1].associativity * cache_info [i-1].sets;
      llc_line_size = cache_info [i-1].line_size;
      DLOG ("Last level cache line size: %d", llc_line_size);
      DLOG ("%d lines in total", llc_lines);
      break;
    }

    /* For details about the bit fields below, please refer to Intel
     * documentation on cpuid instructuion. Application Note 485,
     * page 29-30, Table 2-9
     */
    level = (eax >> 5) & 0x7;
    cache_info [i].num_apic = ((eax >> 26) & 0x3F) + 1;
    cache_info [i].num_thread = ((eax >> 14) & 0xFFF) + 1;
    cache_info [i].fully_assoc = (eax >> 9) & 0x1;
    cache_info [i].self_init_level = (eax >> 8) & 0x1;
    cache_info [i].cache_level = level;
    cache_info [i].cache_type = eax & 0x1F;
    cache_info [i].associativity = ((ebx >> 22) & 0x3FF) + 1;
    cache_info [i].line_part = ((ebx >> 12) & 0x3FF) + 1;
    cache_info [i].line_size = (ebx & 0xFFF) + 1;
    cache_info [i].sets = ecx + 1;
    cache_info [i].inclusive = (edx >> 1) & 0x1;
    cache_info [i].invd = edx & 0x1;
    /* Cache size = ways x partitions x line size x sets */
    cache_info [i].cache_size =
      cache_info [i].associativity * cache_info [i].line_part *
      cache_info [i].line_size * cache_info [i].sets;

    DLOG ("Level %d cache detected:", level);
    DLOG ("  Total cache size: %lld KB", cache_info [i].cache_size / 1024);
    switch (cache_info [i].cache_type) {
      case 1 :
        DLOG ("  Level %d  Data Cache", cache_info [i].cache_level);
        break;
      case 2 :
        DLOG ("  Level %d  Instruction Cache", cache_info [i].cache_level);
        break;
      case 3 :
        DLOG ("  Level %d  Unified Cache", cache_info [i].cache_level);
        break;
      default:
        DLOG ("  Level %d  Unknown Cache", cache_info [i].cache_level);
    }
    if (cache_info [i].fully_assoc) {
      DLOG ("  Fully associative");
    } else {
      DLOG ("  %d way associative", cache_info [i].associativity);
    }
    DLOG ("  Line size: %d", cache_info [i].line_size);
    DLOG ("  Physical line partitions: %d", cache_info [i].line_part);
    DLOG ("  Number of sets: %d", cache_info [i].sets);
    DLOG ("  Inclusive: %s", cache_info [i].inclusive ? "Yes" : "No");
  }
}

/* Get local and global last level cache misses at current time */
static void
perfmon_get_misses (uint64 *local_miss, uint64 *global_miss)
{
  *local_miss = perfmon_pmc_read (PERFMON_LM_PMC);
  *global_miss = perfmon_uncore_pmc_read (PERFMON_GM_PMC);
}

extern void
perfmon_pervcpu_reset (vcpu * vcpu)
{
  uint64 local_miss = 0, global_miss = 0, instruction_retired = 0;

  /* Initialise percpu cache occupancy estimation variables */
  perfmon_get_misses (&local_miss, &global_miss);
  instruction_retired = perfmon_pmc_read (PERFMON_IR_PMC);

  vcpu->prev_local_miss = local_miss;
  vcpu->prev_global_miss = global_miss;
  vcpu->prev_inst_ret = instruction_retired;
}

extern void
perfmon_vcpu_acnt_start (vcpu * vcpu)
{
  uint64 now;

  if (vcpu && nehalem_perfmon_enabled) {
    perfmon_pervcpu_reset (vcpu);
    RDTSC (now);
    vcpu->acnt_tsc = now;
    //DLOG ("Per-VCPU accounting begins");
  }
}

extern void
perfmon_vcpu_acnt_end (vcpu * vcpu)
{
  uint64 cur_occupancy = 0, inst_ret = 0, local_miss = 0, global_miss = 0;
  uint64 prev_local_miss = 0, prev_global_miss = 0, prev_occupancy = 0, prev_inst_ret = 0;
  int i = 0;

  if (vcpu && nehalem_perfmon_enabled) {
    prev_occupancy = vcpu->cache_occupancy;

    /* Get current local and global last level cache miss info */
    perfmon_get_misses (&local_miss, &global_miss);
    inst_ret = perfmon_pmc_read (PERFMON_IR_PMC);

    prev_local_miss = vcpu->prev_local_miss;
    prev_global_miss = vcpu->prev_global_miss;
    prev_inst_ret = vcpu->prev_inst_ret;

    local_miss -= prev_local_miss;
    global_miss -= prev_global_miss;
    inst_ret -= prev_inst_ret;

#if 0
    DLOG ("Local L3 miss difference: %d", local_miss);
    DLOG ("Global L3 miss difference: %d", global_miss);
    DLOG ("Local Instructions retired difference: %d", instruction_retired);
    DLOG ("Previous miss based occupancy: %d", prev_occupancy);
#endif

    /* i will be used to do the division, which will be implemented as right shift */
    for (i = 0; (llc_lines >> i) > 0 && (llc_lines >> i) != 1; i++);
    //asm volatile ("bsrl %1,%0":"=r" (i):"r" (llc_lines));
    //DLOG ("Should shift right %d bits", i);

    cur_occupancy = prev_occupancy + local_miss - (global_miss >> i) * prev_occupancy;
    vcpu->cache_occupancy = cur_occupancy;
    vcpu->mpki = div64_64 (local_miss * 1000, inst_ret);
    //DLOG ("vcpu:%X local miss:%llX instruction retired:%llX",
    //      (u32) vcpu, local_miss, inst_ret);
    //DLOG ("Per-VCPU accounting ends");
  }
}

extern bool
perfmon_init (void)
{
  u32 eax, ebx, edx, i, display;

  display = cpuid_display_family_model ();

  cpuid (0xA, 0, &eax, &ebx, NULL, &edx);

  if ((u8) eax == 0) {
    DLOG ("unsupported");
    return FALSE;
  }

  /* Enable Off-core Rsp Perfmon when current microarchitecture is Nehalem */
  if ((((display >> 8) & 0xF) == 0x6) &&
      ((display & 0xFF) == 0x1A)) {
    DLOG ("Nehalem Enhancements of Performance Monitoring enabled.");
    nehalem_perfmon_enabled = TRUE;
  }

  /* Get cache information */
  perfmon_get_cache_info ();

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
  perfmon_pmc_config (1, 0x3C, 1);
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

  /* Monitoring number of Instructions Retired */
  perfmon_pmc_config (PERFMON_IR_PMC, 0xC0, 0);

  /* If platform is Nehalem, enable uncore counter and set events */
  if (nehalem_perfmon_enabled) {
    perfmon_uncore_cntr_enable (0x0LL | UNCORE_EN_PC0 | UNCORE_EN_FC0);
    perfmon_uncore_fixed_enable (0);

    DLOG ("Selecting local and global cache miss events");

    /* Monitering local last level cache miss from off-core */
    offcore_perfmon_pmc_config (PERFMON_LM_PMC, 0, (uint64) 0x0 |
        OFFCORE_DMND_DATA_RD |
        OFFCORE_DMND_IFETCH |
        OFFCORE_WB |
        OFFCORE_PF_DATA_RD |
        OFFCORE_PF_RFO |
        OFFCORE_PF_IFETCH |
        OFFCORE_OTHER |
        OFFCORE_REMOTE_CACHE_FWD |
        OFFCORE_REMOTE_DRAM |
        OFFCORE_LOCAL_DRAM);

    /* Monitering global last level cache miss from uncore */
    /* 0x0A and 0x0F for UNC_L3_LINES_IN.ANY */
    perfmon_uncore_pmc_config (PERFMON_GM_PMC, 0x0A, 0x0F);
    /* 0x09 and 0x03 for UNC_L3_MISS.ANY */
    //perfmon_uncore_pmc_config (0, 0x09, 0x03);
  }

  perfmon_enabled = TRUE;

#if 0
  asm volatile ("wbinvd");

  DLOG ("Now, after flush");
  DLOG ("Fixed reading: 0x%llX", rdmsr (MSR_UNCORE_FIXED_CTR0));

  perfmon_percpu_reset ();

  uint64 occupancy = 0;
  uint64 local_miss = 0, global_miss = 0;
  occupancy = perfmon_miss_occupancy ();
  occupancy = percpu_read64 (perfmon_prev_miss_occupancy);
  local_miss = percpu_read64 (perfmon_prev_local_miss);
  global_miss = percpu_read64 (perfmon_prev_global_miss);
  DLOG ("Occupancy prediction: %d lines", occupancy);
  DLOG ("Previous local L3 miss: %d", local_miss);
  DLOG ("Previous global L3 miss: %d", global_miss);

  uint32 phy_addr = alloc_phys_frames (64);
  void * virt_addr = map_contiguous_virtual_pages (phy_addr | 0x3, 64);
  int k = 0;
  for (k = 0; k < 4096 * 64; k++) {
    *(((char*) virt_addr) + k) = 1;
  }

  occupancy = perfmon_miss_occupancy ();
  occupancy = percpu_read64 (perfmon_prev_miss_occupancy);
  local_miss = percpu_read64 (perfmon_prev_local_miss);
  global_miss = percpu_read64 (perfmon_prev_global_miss);
  DLOG ("Occupancy prediction: %d lines", occupancy);
  DLOG ("Previous local L3 miss: %d", local_miss);
  DLOG ("Previous global L3 miss: %d", global_miss);

  DLOG ("Fixed reading: 0x%llX", rdmsr (MSR_UNCORE_FIXED_CTR0));

  for (;;);
#endif
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = perfmon_init
};

//DEF_MODULE (perfmon, "Performance monitoring driver", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
