#include <pin.h>
#include <ardutime.h>
#include <vcpu.h>
#include <stdlib.h>
#include <stdio.h>

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
}

void main()
{
	struct sched_param s_params = {.type = MAIN_VCPU, .C = 90, .T = 100};
  int new_vcpu = vcpu_create(&s_params);
  if(new_vcpu < 0) {
    printf("Failed to create vcpu\n");
    exit(1);
  }
  vcpu_bind_task(new_vcpu);

	setup();
	while(1) loop();
}
