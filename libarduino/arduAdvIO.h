#include "pin.h"
#include "ardutime.h"
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>

#ifndef _ARDUADV_IO_H_
#define _ARDUADV_IO_H_

#if 0
unsigned long _pulseIn(int pin, int value, unsigned long timeout)
{

  unsigned long long time_a, time_b, time_c;
	/* copied from Galileo code */

	time_a = micros();

	//wait for the previous pulse to end or timeout
	while (digitalRead(pin) == value) {
		time_b = micros();
		//timeBFlag = true;
		if (time_b - time_a > timeout) {
			printf("timeout\n");
			return 0;
		}
		else if (time_b < time_a) { //micros() overflow
			time_c = (0xFFFFFFFF - time_a) + time_b;
			if (time_c > timeout) {
				printf("timeout\n");
				return 0;
			}
		}
	}

	//wait for pin to go to target value or timeout
	while (digitalRead(pin) != value) {
		time_b = micros();
		//timeBFlag = true;
		if (time_b -  time_a > timeout) {
			printf("timeout\n");
			return 0;
		}
		else if (time_b < time_a) { //micros() overflow
			time_c = (0xFFFFFFFF - time_a) + time_b;
			if (time_c > timeout) {
				printf("timeout\n");
				return 0;
			}
		}
	}

	//determine pulse length
	/*
	if (!timeBFlag)
		time_b = micros();
	time_a = time_b;
	*/
	time_a = micros();
	printf("time_a is %ld\n", time_a);
	while (digitalRead(pin) == value);
	time_b = micros();
	printf("time_b is %ld\n", time_b);
	if (time_b > time_a)
		return time_b - time_a - 1;
	else { //micros() overflow
		time_c = (0xFFFFFFFF - time_a) + time_b - 1;
		return time_c;
	}
}
#endif

#define TSC_FREQ_USEC 0x157LL
#define MAX_PULSE_WIDTH 0x796C60LL

unsigned long pulseIn(int pin, int value)
{
  unsigned long long start_t, end_t, middle_t;

	//wait for the previous pulse to end
	while (fastDigitalRead(pin) == value);
	//wait for pin to go to target value
	while (fastDigitalRead(pin) != value);
	uRdtsc(start_t);
	while (fastDigitalRead(pin) == value) {
		/* XXX: Hack: sometimes we are not able to 
			 get out of this loop. The high level
			 pulse width seems endless. Don't know why
		 */
		uRdtsc(middle_t);
		if (middle_t - start_t > MAX_PULSE_WIDTH)
			return 0;
	}
	uRdtsc(end_t);

	return (end_t - start_t) / TSC_FREQ_USEC; //microseconds
}

#endif
