#include <unistd.h>
#include <sys/time.h>
#include <string.h>

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

#define uRdtsc(var)                                            \
 {                                                             \
   unsigned long var##_lo, var##_hi;                           \
   asm volatile("rdtsc" : "=a"(var##_lo), "=d"(var##_hi));     \
   var = var##_hi;                                             \
   var <<= 32;                                                 \
   var |= var##_lo;                                            \
 }

void
rdtsc(unsigned long long int *rdtsc_ptr)
{
	unsigned long long int rdtsc_val;
	uRdtsc(rdtsc_val);
	memcpy(rdtsc_ptr, &rdtsc_val, sizeof(rdtsc_val));
}

unsigned long
micros()
{
	struct timeval t;
	gettimeofday(&t, NULL);
	return t.tv_sec * 1000000 + t.tv_usec;
}

#endif
