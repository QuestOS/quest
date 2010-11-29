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
/* Off-core Response MSR. 0 for Nehalem. 0 and 1 for Westmere. */
#define MSR_OFFCORE_RSP(x) (0x1A6 + (x))
#define OFFCORE_RSP0_EVT    0xB7
#define OFFCORE_RSP1_EVT    0xBB /* Available in Westmere only */
#define OFFCORE_RSP_MASK    0x01
/* Uncore performance monitoring facility */
#define MSR_UNCORE_PERF_GLOBAL_CTRL       0x391
#define MSR_UNCORE_PERF_GLOBAL_STATUS     0x392
#define MSR_UNCORE_PERF_GLOBAL_OVF_CTRL   0x393

#define MSR_UNCORE_PERFEVTSEL(x)  (0x3B0 + (x))
#define MSR_UNCORE_PMC(x)         (0x3C0 + (x))

#define MSR_UNCORE_FIXED_CTR0        0x394
#define MSR_UNCORE_FIXED_CTR_CTRL    0x395

/* MSR_UNCORE_PERF_GLOBAL_CTRL bit fields to enable/disable
 * general-purpose and fixed-function counters in the uncore
 */
#define UNCORE_EN_PC0       (0x1LL)
#define UNCORE_EN_PC1       (0x1LL << 1)
#define UNCORE_EN_PC2       (0x1LL << 2)
#define UNCORE_EN_PC3       (0x1LL << 3)
#define UNCORE_EN_PC4       (0x1LL << 4)
#define UNCORE_EN_PC5       (0x1LL << 5)
#define UNCORE_EN_PC6       (0x1LL << 6)
#define UNCORE_EN_PC7       (0x1LL << 7)
#define UNCORE_EN_FC0       (0x1LL << 32)
#define UNCORE_PMI_CORE0    (0x1LL << 48)
#define UNCORE_PMI_CORE1    (0x1LL << 49)
#define UNCORE_PMI_CORE2    (0x1LL << 50)
#define UNCORE_PMI_CORE3    (0x1LL << 51)
#define UNCORE_PMI_FRZ      (0x1LL << 63)

/* MSR_OFFCORE_RSP_Z Bit Field Definition */
#define OFFCORE_DMND_DATA_RD            (0x1)
#define OFFCORE_DMND_RFO                (0x1 << 1)
#define OFFCORE_DMND_IFETCH             (0x1 << 2)
#define OFFCORE_WB                      (0x1 << 3)
#define OFFCORE_PF_DATA_RD              (0x1 << 4)
#define OFFCORE_PF_RFO                  (0x1 << 5)
#define OFFCORE_PF_IFETCH               (0x1 << 6)
#define OFFCORE_OTHER                   (0x1 << 7)
#define OFFCORE_UNCORE_HIT              (0x1 << 8)
#define OFFCORE_OTHER_CORE_HIT_SNP      (0x1 << 9)
#define OFFCORE_OTHER_CORE_HITM         (0x1 << 10)
#define OFFCORE_REMOTE_CACHE_FWD        (0x1 << 12)
#define OFFCORE_REMOTE_DRAM             (0x1 << 13)
#define OFFCORE_LOCAL_DRAM              (0x1 << 14)
#define OFFCORE_NON_DRAM                (0x1 << 15)

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
    u8 _reserved0:1;
    u8 enable_counters:1;
    u8 invert_counter_mask:1;
    u8 counter_mask;
  } PACKED;
  struct {
    u8 event_select;
    u8 unit_mask;
    u8 _reserved1:1;
    u8 reset_occ:1;
    u8 edge_detect:1;
    u8 _reserved2:1;
    u8 pmi_enable:1;
    u8 _reserved3:1;
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

  wrmsr (IA32_PERFEVTSEL(x), conf.raw);
}

static inline void
perfmon_uncore_pmc_config (int x, u8 event_select, u8 unit_mask)
{
  ia32_perfevtsel_t conf;

  conf.raw = 0;
  conf.event_select = event_select;
  conf.unit_mask = unit_mask;
  conf.enable_counters = 1;
  //conf.edge_detect = 1;

  wrmsr (MSR_UNCORE_PERFEVTSEL(x), conf.raw);
}

static inline u64
perfmon_pmc_read (int x)
{
  return rdmsr (IA32_PMC(x));
}

static inline void
perfmon_uncore_cntr_enable (uint64 counters)
{
  wrmsr (MSR_UNCORE_PERF_GLOBAL_CTRL, counters);
}

static inline void
perfmon_uncore_fixed_enable (bool pmi)
{
  pmi ? wrmsr (MSR_UNCORE_FIXED_CTR_CTRL, 0x5LL) :
    wrmsr (MSR_UNCORE_FIXED_CTR_CTRL, 0x1LL);
}

static inline u64
perfmon_uncore_pmc_read (int x)
{
  return rdmsr (MSR_UNCORE_PMC(x));
}

extern bool perfmon_init (void);
extern void offcore_perfmon_pmc_config (int, int, uint64);
extern bool perfmon_enabled;
extern bool nehalem_perfmon_enabled;
extern bool westmere_perfmon_enabled;
extern uint64 perfmon_miss_occupancy (void);
extern void perfmon_percpu_reset (void);

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
