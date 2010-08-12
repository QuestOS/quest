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

/* Timing and measurement code */

#include "kernel.h"
#include "util/debug.h"

extern u32 tick;

#define TIMING_ASM(var, insrs, edi)                                     \
  asm volatile ("movl $1, %%eax\n"                                      \
                "cpuid\n"                                               \
                "rdtsc\n"                                               \
                "movl %%eax, %%esi\n"                                   \
                                                                        \
                insrs                                                   \
                                                                        \
                "movl $1, %%eax\n"                                      \
                "cpuid\n"                                               \
                "rdtsc\n"                                               \
                "subl %%esi, %%eax\n"                                   \
                :"=a" (diff):"D" (edi): "ebx", "ecx", "edx", "esi")

void
measure_LAPIC_read (void)
{
  u32 diff;
  if ((tick & 0xFF) == 0) {
    TIMING_ASM (diff,                             \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                "movl 0xFEE00020, %%ecx\n"        \
                , 0);
    logger_printf ("measure_LAPIC_read: %u\n", diff);
  }
}

void
measure_DR3_write (void)
{
  u32 diff;
  if ((tick & 0xFF) == 0) {
    TIMING_ASM (diff,                           \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"         \
                  "movl %%ecx, %%dr3\n"
                , 0);
    logger_printf ("measure_DR3_write: %u\n", diff);
  }
}

void
measure_DR3_read (void)
{
  u32 diff;
  if ((tick & 0xFF) == 0) {
    TIMING_ASM (diff,                           \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"         \
                  "movl %%dr3, %%ecx\n"
                , 0);
    logger_printf ("measure_DR3_read: %u\n", diff);
  }
}

void
measure_FS_read (void)
{
  u32 diff;
  static u32 idx = 0;
  descriptor seg = {
    .pBase0 = 0x1000,
    .pBase1 = 0xFF,
    .pBase2 = 0xFF,
    .uLimit0 = 0x1000,
    .uType  = 0x12,             /* writeable */
    .uDPL   = 0,
    .fPresent = 1,
    .uLimit1 = 0,
    .f = 0,
    .f0 = 0,
    .fX = 1,
    .fGranularity = 0
  };
  descriptor *ad = (descriptor *) KERN_GDT;
  if ((tick & 0xFF) == 0) {
    if (idx == 0) {
      for (idx=1;idx<256;idx++) {
        if (!(ad[idx].fPresent)) {
          memcpy (&ad[idx], &seg, sizeof (seg));
          idx <<= 3;
          break;
        }
      }
    }

    TIMING_ASM (diff,
                "movw %%di, %%fs\n"           \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"        \
                "movl %%fs:0, %%ecx\n"
                , idx);

    logger_printf ("measure_FS_read: %u\n", diff);
  }
}

void
measure_FS_load (void)
{
  u32 diff;
  if ((tick & 0xFF) == 0) {
    TIMING_ASM (diff,
                "movw $0x10, %%dx\n"       \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                "movw %%dx, %%fs\n" \
                , 0);
    logger_printf ("measure_FS_load: %u\n", diff);
  }
}

void
measure_timing_overhead (void)
{
  u32 diff;
  if ((tick & 0xFF) == 0) {
    TIMING_ASM (diff, , 0);
    logger_printf ("measure_timing_overhead: %u\n", diff);
  }
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
