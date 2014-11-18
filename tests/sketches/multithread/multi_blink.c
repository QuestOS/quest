#include <pin.h>
#include <ardutime.h>
#include <arduthread.h>

#include <vcpu.h>

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led1 = 13;
int led2 = 9;
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

// the loop routine runs over and over again forever:
loop(1,40,100) {
	digitalWrite(led1, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);               // wait for a second
	digitalWrite(led1, LOW);    // turn the LED off by making the voltage LOW
	delay(1000);               // wait for a second
}

// the loop routine runs over and over again forever:
loop(2,40,100) { 
	// set the brightness of pin 9:
	analogWrite(led2, brightness);    
	// change the brightness for next time through the loop:
	brightness = brightness + fadeAmount;
	// reverse the direction of the fading at the ends of the fade: 
	if (brightness == 0 || brightness == 255) {
		fadeAmount = -fadeAmount ; 
	}     
	// wait for 30 milliseconds to see the dimming effect    
	delay(30);                            
}

// the setup routine runs once when you press reset:
setup() {                
  // initialize the digital pin as an output.
  pinMode(led1, OUTPUT);     
  pinMode(led2, OUTPUT);
}
