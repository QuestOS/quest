#include <unistd.h>
#include "ardurdtsc.h"

#ifndef _ARDUTIME_IO_H_
#define _ARDUTIME_IO_H_

void
delay(unsigned long ms)
{
    usleep(ms * 1000);
}

void
rdtsc(unsigned long long *rdtsc_ptr)
{
	rdtsc_syscall(rdtsc_ptr);
}

#endif
