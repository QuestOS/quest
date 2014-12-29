#include <stdio.h>    
#include <stdlib.h>    
#include <sys/time.h>    
#include <ardutime.h>

struct timeval tval_start, tval_end;

loop()
{
}

setup()
{
	time_t ms_start, ms_end;
	gettimeofday (&tval_start, NULL);
	ms_start = tval_start.tv_sec * 1000 + tval_start.tv_usec / 1000;
	delay(2000);
	gettimeofday (&tval_end, NULL);
  ms_end = tval_end.tv_sec * 1000 + tval_end.tv_usec / 1000;
  printf ("Time: %lld milliseconds\n", ms_end - ms_start);
}
