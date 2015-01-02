#include <pin.h>
#include <ardutime.h>
#include <arduutils.h>

volatile int counter;
int status = LOW;
int FLIP_N = 4000;

void
IntHandler()
{
  counter++;
}

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(2, IntHandler, CHANGE);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly: 
	int i;
	unsigned long long start_t, end_t;

	printf("starting...\n");
	rdtsc(&start_t);
  for (i = 1; i <= FLIP_N; i++) {
    digitalWrite(13, status = !status);
    delay(10);
  }
	rdtsc(&end_t);
	printf("counter is %d\n", counter);
	print_long_long_hex(end_t - start_t);
	while (1);
}
