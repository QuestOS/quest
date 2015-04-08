#include "pin.h"
#include "ardutime.h"
#include <limits.h>
#include <stdbool.h>
//#include <stdio.h>

#ifndef _ARDUADV_IO_H_
#define _ARDUADV_IO_H_

unsigned long _pulseIn(int pin, int value, unsigned long timeout)
{

  unsigned long time_a, time_b, time_c;
	/* copied from Galileo code
	 * don't understand why we need this variable */
	//bool timeBFlag = false;

	time_a = micros();

	//wait for the previous pulse to end or timeout
	while (digitalRead(pin) == value) {
		time_b = micros();
		printf("1-\n");
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

	printf("stage 1 done\n");
	//wait for pin to go to target value or timeout
	while (digitalRead(pin) != value) {
		time_b = micros();
		printf("2-\n");
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
	printf("stage 2 done\n");
	time_a = micros();
	printf("time_a is %ld\n", time_a);
	while (digitalRead(pin) == value) {
		printf("3-\n");
	}
	time_b = micros();
	printf("time_b is %ld\n", time_b);
	if (time_b > time_a)
		return time_b - time_a - 1;
	else { //micros() overflow
		time_c = (0xFFFFFFFF - time_a) + time_b - 1;
		return time_c;
	}
}

unsigned long pulseIn(int pin, int value)
{
	return _pulseIn(pin, value, ULONG_MAX);
}

#endif
