#include<stdio.h>
#include"cpuid.h"

static void test_brand_string(void) 
{
  int eax, ebx, ecx, edx;
  char buf[49], *ptr = buf;

  cpuid(I386_CPUID_BRAND_STRING_INPUT1, 0, &eax, NULL, NULL, NULL);
  if (eax & I386_CPUID_BRAND_STRING_INPUT1) {
#define WRBUF(r) *((int *)ptr) = r; ptr+=4
    cpuid(I386_CPUID_BRAND_STRING_INPUT2, 0, &eax, &ebx, &ecx, &edx); 
    WRBUF(eax); WRBUF(ebx); WRBUF(ecx); WRBUF(edx);
    cpuid(I386_CPUID_BRAND_STRING_INPUT3, 0, &eax, &ebx, &ecx, &edx); 
    WRBUF(eax); WRBUF(ebx); WRBUF(ecx); WRBUF(edx);
    cpuid(I386_CPUID_BRAND_STRING_INPUT4, 0, &eax, &ebx, &ecx, &edx); 
    WRBUF(eax); WRBUF(ebx); WRBUF(ecx); WRBUF(edx);
    buf[48] = 0;                /* nul-terminate */
    printf("%s\n",buf);
  } else {
    printf("Processor brand string not supported.\n");
  }
}

static void test_vmx(void) 
{
  int ecx;
  cpuid(1, 0, NULL, NULL, &ecx, NULL);
  if (ecx & (1<<5)) printf ("VMX supported\n");
  else printf ("VMX not supported\n");
}

int main(void) 
{
  test_brand_string();
  test_vmx();
  return 0;
}

/* 
 * Local Variables:
 * compile-command: "gcc -o test_cpuid test_cpuid.c"
 * End:
 */
