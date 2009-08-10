/* -*- Mode: C -*- */

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
cpuid_vmx_support (void)
{
  int ecx;
  cpuid (1, 0, NULL, NULL, &ecx, NULL);
  return (ecx & (1 << 5));
}
