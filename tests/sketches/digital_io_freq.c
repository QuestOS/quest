#include <pin.h>
#include <ardutime.h>
#include <arduutils.h>

int pin_status = LOW;
int BLINK_NB = 4000;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(13, OUTPUT);     
}

void loop() {
	//experiment
	unsigned long long start_t, end_t;
	int i;
	fprintf(stderr, "starting...");
	rdtsc(&start_t);
	for (i = 0; i < BLINK_NB; i++) 
		digitalWrite(13, pin_status = !pin_status);
	rdtsc(&end_t);
	print_long_long_hex(end_t - start_t);
	while (1);
}
