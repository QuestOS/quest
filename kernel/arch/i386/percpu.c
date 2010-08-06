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

/* Support for per-CPU variables, partially inspired by the Linux
 * implementation.  All per-CPU variables are declared in a special
 * non-loaded section so they all gain distinct addresses.  Then space
 * is allocated after the number of CPUs is determined.  A segment
 * descriptor for each space is setup, and each CPU loads a spare
 * segment register with it (like %fs).  Per-CPU variable access is
 * done, via macros, relative to that segment override. */

/* Per-CPU variable initialization functions are arranged so that a
 * null-terminated array of pointers to these functions are placed in
 * a special section beginning at _percpu_ctor_list.  This is inspired
 * by the way C++ does global constructors. */

#include <kernel.h>
#include <util/debug.h>
#include <mem/physical.h>
#include <mem/virtual.h>
#include <arch/i386-percpu.h>
#include <smp/apic.h>

//#define DEBUG_PERCPU

#ifdef DEBUG_PERCPU
#define DLOG(fmt,...) DLOG_PREFIX("percpu",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

extern u8 _percpu_pages;
extern void (*_percpu_ctor_list)();

u8 *percpu_virt[MAX_CPUS];

extern void
percpu_per_cpu_init (void)
{
  descriptor *ad = (descriptor *)KERN_GDT;
  int i, pages = (int) &_percpu_pages;
  int limit = pages * 0x1000 - 1;

  if (pages == 0) {
    DLOG ("no per-CPU variables in system");
    /* fall back to the usual data segment */
    asm volatile ("movw %%ds, %%ax\n"
                  "movw %%ax, %%"PER_CPU_SEG_STR"\n"
                  "movzwl %%ax, %%eax\n"
                  "movl %%eax, %%"PER_CPU_DBG_STR"":::"eax");
    return;
  }

  for (i=1; i<256; i++)
    if (ad[i].fPresent == 0)
      break;
  if (i == 256) panic ("out of GDT");

  uint start_frame = alloc_phys_frames (pages);
  if (start_frame == -1) panic ("out of physical RAM");
  uint start_virt = (uint) map_contiguous_virtual_pages (start_frame | 3, pages);
  if (start_virt == 0) panic ("out of virtual RAM");

  percpu_virt[LAPIC_get_physical_ID ()] = (u8 *) start_virt;

  descriptor seg = {
    .pBase0 = start_virt & 0xFFFF,
    .pBase1 = (start_virt >> 16) & 0xFF,
    .pBase2 = (start_virt >> 24) & 0xFF,
    .uLimit0 = limit & 0xFFFF,
    .uType  = 0x12,             /* writeable */
    .uDPL   = 0,
    .fPresent = 1,
    .uLimit1 = (limit >> 16) & 0xF,
    .f = 0,
    .f0 = 0,
    .fX = 1,
    .fGranularity = 0
  };

  memcpy (&ad[i], &seg, sizeof (seg));

  i <<= 3;
  DLOG ("init percpu_pages=%d segsel=0x%X start_frame=%p start_virt=%p",
        pages, i, start_frame, start_virt);

  asm volatile ("mov %0, %%"PER_CPU_SEG_STR"\n"
                "mov %0, %%"PER_CPU_DBG_STR :: "r" (i));

  /* invoke initialization functions */
  void (**ctor) ();
  for (ctor = &_percpu_ctor_list; *ctor; ctor++)
    (*ctor) ();
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
