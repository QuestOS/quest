#include <pin.h>
#include <ardutime.h>
#include <arduutils.h>
#include <arduthread.h>

/* XXX: This is a sketch to test what is the
 * max interrupt freq that Qduino can handle
 * without harming the execution of digitalWrite.
 * NB: a better way would be to use oscilloscope 
 * to generate square wave to trigger the interrupt
 * handler on Qduino. See how what is the max freq
 * of the square wave that Qduino is able to handle
 * every interrupt */
volatile int counter;
int status = LOW;
int FLIP_N = 1000;

void
IntHandler()
{
  counter++;
}

void setup() {
  // put your setup code here, to run once:
  attachInterrupt_vcpu(2, IntHandler, CHANGE, 8, 20);
  pinMode(13, OUTPUT);
}

void loop(1,1,20) {
  // put your main code here, to run repeatedly: 
	int i;
	unsigned long long start_t, end_t;

	printf("starting...\n");
	rdtsc(&start_t);
  for (i = 1; i <= FLIP_N; i++) {
    digitalWrite(13, status = !status);
    delay(9);
  }
	rdtsc(&end_t);
	printf("counter is %d\n", counter);
	print_long_long_hex(end_t - start_t);
	while (1);
}
