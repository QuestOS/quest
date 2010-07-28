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

#include"arch/i386.h"
#include"util/cpuid.h"
#include"kernel.h"

void
cpuid_get_brand_string (char *str, uint32 n)
{
  uint32 eax, ebx, ecx, edx;
  char buf[I386_CPUID_BRAND_STRING_LENGTH], *ptr = buf;

  if (n < 1)
    return;
  cpuid (I386_CPUID_BRAND_STRING_INPUT1, 0, &eax, NULL, NULL, NULL);
  if (eax & I386_CPUID_BRAND_STRING_INPUT1) {
#define WRBUF(r) *((int *)ptr) = r; ptr+=4
    cpuid (I386_CPUID_BRAND_STRING_INPUT2, 0, &eax, &ebx, &ecx, &edx);
    WRBUF (eax);
    WRBUF (ebx);
    WRBUF (ecx);
    WRBUF (edx);
    cpuid (I386_CPUID_BRAND_STRING_INPUT3, 0, &eax, &ebx, &ecx, &edx);
    WRBUF (eax);
    WRBUF (ebx);
    WRBUF (ecx);
    WRBUF (edx);
    cpuid (I386_CPUID_BRAND_STRING_INPUT4, 0, &eax, &ebx, &ecx, &edx);
    WRBUF (eax);
    WRBUF (ebx);
    WRBUF (ecx);
    WRBUF (edx);
    buf[48] = 0;                /* nul-terminate */
    memcpy (str, buf,
            n <
            I386_CPUID_BRAND_STRING_LENGTH ? n :
            I386_CPUID_BRAND_STRING_LENGTH);
  } else {
    memcpy (str, "Brand String not supported.", n);
  }
  str[n - 1] = 0;
}

bool
cpuid_tsc_support (void)
{
  int edx;
  cpuid (1, 0, NULL, NULL, NULL, &edx);
  return (edx & (1 << 4));
}

bool
cpuid_msr_support (void)
{
  int edx;
  cpuid (1, 0, NULL, NULL, NULL, &edx);
  return (edx & (1 << 5));
}

bool
cpuid_vmx_support (void)
{
  int ecx;
  cpuid (1, 0, NULL, NULL, &ecx, NULL);
  return (ecx & (1 << 5));
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
