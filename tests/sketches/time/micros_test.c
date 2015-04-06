#include <ardutime.h>
#include <stdio.h>

unsigned long start, end, duration;

void
setup()
{
	start = micros();
	delay(10);
	end = micros();
	duration = end - start;
	printf("%ld\n", duration);
}

void 
loop()
{

}
