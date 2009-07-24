#ifndef CPUID_H_
#define CPUID_H_
#include "types.h"

#define I386_CPUID_BRAND_STRING_LENGTH 49
#define I386_CPUID_BRAND_STRING_INPUT1 0x80000000
#define I386_CPUID_BRAND_STRING_INPUT2 0x80000002
#define I386_CPUID_BRAND_STRING_INPUT3 0x80000003
#define I386_CPUID_BRAND_STRING_INPUT4 0x80000004


static inline void cpuid ( in_eax, in_ecx, out_eax, out_ebx, out_ecx, out_edx )
  int in_eax, in_ecx;
  int *out_eax, *out_ebx, *out_ecx, *out_edx;
{
  int eax, ebx, ecx, edx;
  asm volatile ( "cpuid" 
                 : "=a" (eax), "=b" (ebx), "=c" (ecx), "=d" (edx)
                 : "a" (in_eax), "c" (in_ecx) );
  if (out_eax) *out_eax = eax;
  if (out_ebx) *out_ebx = ebx;
  if (out_ecx) *out_ecx = ecx;
  if (out_edx) *out_edx = edx;
}

void cpuid_get_brand_string(char *, int);
bool cpuid_vmx_support(void);

#endif
