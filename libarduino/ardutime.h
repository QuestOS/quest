#include <unistd.h>
#include <sys/time.h>
#include "ardurdtsc.h"

#ifndef _ARDUTIME_IO_H_
#define _ARDUTIME_IO_H_

void
delay(unsigned long ms)
{
  usleep(ms * 1000);
}

void
delayMicroseconds(unsigned long us)
{
	usleep(us);
}

void
rdtsc(unsigned long long *rdtsc_ptr)
{
	rdtsc_syscall(rdtsc_ptr);
}

unsigned long
micros()
{
	struct timeval t;
	gettimeofday(&t, NULL);
	return t.tv_sec * 1000000 + t.tv_usec;
}

#define uRdtsc(var)                                            \
 {                                                             \
   unsigned long var##_lo, var##_hi;                           \
   asm volatile("rdtsc" : "=a"(var##_lo), "=d"(var##_hi));     \
   var = var##_hi;                                             \
   var <<= 32;                                                 \
   var |= var##_lo;                                            \
 }

#endif
