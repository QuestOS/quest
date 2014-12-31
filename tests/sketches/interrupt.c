#include <pin.h>
#include <stdio.h>
#include <ardutime.h>
#include <vcpu.h>

int pin = 13;
int int_pin = 2;
volatile int state = LOW;

void blink()
{
	state = !state;
}

void setup()
{
	pinMode(pin, OUTPUT);
	attachInterrupt(int_pin, blink, CHANGE);
}

void loop()
{
	digitalWrite(pin, state);
}
