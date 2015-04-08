#include <pin.h>
#include <ardutime.h>
#include <stdio.h>

int pin_status = LOW;
int BLINK_NB = 2000;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, FAST_OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly: 
	delay(100);
	fastDigitalWrite(2, HIGH);
	delay(100);
	fastDigitalWrite(2, LOW);
}
