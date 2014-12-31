#include <pin.h>
#include <ardutime.h>
#include <arduutils.h>

volatile int counter;
int status = LOW;
int FLIP_N = 4000;
unsigned long start_t, end_t;

void
IntHandler()
{
  counter++;
}

void setup() {
  // put your setup code here, to run once:
	unsigned long long start_t, end_t;
  attachInterrupt(2, IntHandler, CHANGE);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly: 
	int i;

	printf("starting...\n");
	rdtsc(&start_t);
  for (i = 1; i <= FLIP_N; i++) {
    digitalWrite(13, status = !status);
    delay(100);
  }
	rdtsc(&end_t);
	printf("counter is %d\n", counter);
	print_long_long_hex(end_t - start_t);
	while (1);
}
