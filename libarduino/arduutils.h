#include <stdio.h>

#ifndef __ARDU_UTILS__
#define __ARDU_UTILS__

void print_long_long_hex(unsigned long long int val)
{
	unsigned long val_lo = (unsigned long)val;
	unsigned long val_hi = (unsigned long)(val >> 32);

	if (val_hi != 0) {
		printf("0x%lx", val_hi);
		printf("%.8lx\n", val_lo);
	} else {
		printf("0x%lx\n", val_lo);
	}
}

#endif

