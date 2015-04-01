#include <stdio.h>
#include <arduthread.h>
#include <ardutime.h>

#define UNIT 0x563A9
unsigned long counter;
unsigned int counter_keeper[512];
int index = 0;

/*
void
loop(1,10,100) {
	counter++;
}
*/

void
loop(2,30,300) {
	unsigned long long start, end;
	rdtsc(&start);
	while(1) {
		counter++;
		rdtsc(&end);
		if (end - start >= 300 * UNIT) {
			//printf("%d\n", counter);
			counter_keeper[index++] = counter;
			if (index == 512) break;
			start = end;
			counter = 0;
		}
	}

	int i;
	for (i = 0; i < index; i++) 
		printf("%d\n", counter_keeper[i]);
	while(1);
}

volatile int s;
void dummy_func()
{
	int local;
	while (1) {
		local++;
		s += local;
	}
}

void
loop(3,50,500) {
	dummy_func();
}

void
loop(4,50,500) {
	dummy_func();
}

void
loop(5,50,500) {
	dummy_func();
}

void
setup() {
}
