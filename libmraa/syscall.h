#ifndef _SYSCALL_
#define _SYSCALL_

#define CLOBBERS1 "memory","cc","%ebx","%ecx","%edx","%esi","%edi"
#define CLOBBERS2 "memory","cc","%ecx","%edx","%esi","%edi"
#define CLOBBERS3 "memory","cc","%ebx","%edx","%esi","%edi"
#define CLOBBERS4 "memory","cc","%ebx","%ecx","%esi","%edi"
#define CLOBBERS5 "memory","cc","%edx","%esi","%edi"
#define CLOBBERS6 "memory","cc","%esi","%edi"
#define CLOBBERS7 "memory","cc","%edi"

/* int $0x30 is syscall0, defined in boot.S */
static int
make_i2c_syscall(int operation, int arg1, int arg2, int arg3)
{
  int ret;
	asm volatile ("int $0x30\n":"=a" (ret) : "a" (13L),
			"b"(operation), "c" (arg1), "d" (arg2), "S" (arg3) : CLOBBERS7);
	return ret;
}

static int
make_gpio_syscall(int operation, int arg1, int arg2, int arg3)
{
	int ret;
	asm volatile ("int $0x30\n":"=a" (ret) : "a" (12L),
			"b"(operation), "c" (arg1), "d" (arg2), "S" (arg3) : CLOBBERS7);
	return ret;
}

#endif

