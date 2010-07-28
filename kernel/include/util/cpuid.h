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

#ifndef CPUID_H_
#define CPUID_H_
#include "types.h"

#define I386_CPUID_BRAND_STRING_LENGTH 49
#define I386_CPUID_BRAND_STRING_INPUT1 0x80000000
#define I386_CPUID_BRAND_STRING_INPUT2 0x80000002
#define I386_CPUID_BRAND_STRING_INPUT3 0x80000003
#define I386_CPUID_BRAND_STRING_INPUT4 0x80000004


static inline void
cpuid (in_eax, in_ecx, out_eax, out_ebx, out_ecx, out_edx)
     uint32 in_eax, in_ecx;
     uint32 *out_eax, *out_ebx, *out_ecx, *out_edx;
{
  uint32 eax, ebx, ecx, edx;
  asm volatile ("cpuid":"=a" (eax), "=b" (ebx), "=c" (ecx), "=d" (edx)
                :"a" (in_eax), "c" (in_ecx));
  if (out_eax)
    *out_eax = eax;
  if (out_ebx)
    *out_ebx = ebx;
  if (out_ecx)
    *out_ecx = ecx;
  if (out_edx)
    *out_edx = edx;
}

void cpuid_get_brand_string (char *, uint32);
bool cpuid_vmx_support (void);
bool cpuid_tsc_support (void);
bool cpuid_msr_support (void);

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
