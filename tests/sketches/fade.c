#include <pin.h>
#include <ardutime.h>
#include <vcpu.h>
#include <stdlib.h>
#include <stdio.h>

/* fade example */
int led = 9;           // the pin that the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup()  { 
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
} 

// the loop routine runs over and over again forever:
void loop()  { 
  // set the brightness of pin 9:
  analogWrite(led, brightness);    

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade: 
  if (brightness == 0 || brightness == 255) {
    fadeAmount = -fadeAmount ; 
  }     
  // wait for 30 milliseconds to see the dimming effect    
  delay(30);                            
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
