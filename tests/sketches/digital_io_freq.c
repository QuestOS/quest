#include <pin.h>
#include <ardutime.h>
#include <arduutils.h>
#include <vcpu.h>

int pin_status = LOW;
int BLINK_NB = 2000;

// the setup routine runs once when you press reset:
setup() {                
  // initialize the digital pin as an output.
  pinMode(13, OUTPUT);     
	/*
  struct sched_param s_params = {.type = MAIN_VCPU, .C = 20, .T = 100};
  int new_vcpu = vcpu_create(&s_params);
	if(new_vcpu < 0) {
    printf("Failed to create vcpu\n");
  }
  vcpu_bind_task(new_vcpu);
  usleep(1000000);
  usleep(1000000);
  usleep(1000000);
  usleep(1000000);
	*/

	//experiment
	unsigned long long start_t, end_t;
	int i;
	fprintf(stderr, "starting...");
	rdtsc(&start_t);
	for (i = 0; i < BLINK_NB; i++) 
		digitalWrite(13, pin_status = !pin_status);
	rdtsc(&end_t);
	print_long_long_hex(end_t - start_t);
}

loop() {

}
