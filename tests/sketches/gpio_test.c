#include <stdio.h>
#include "digital_io.h"

int
main ()
{
	int ledPin = 11;
	int ret;
	ret = pinMode(ledPin, OUTPUT);
	if (ret < 0)
		printf("failed to set pin mode");
	ret = digitalWrite(ledPin, HIGH);
	if (ret < 0)
		printf("failed to write");
	return 0;
}
