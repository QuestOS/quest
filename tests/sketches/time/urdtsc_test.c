#include <arduutils.h>
#include <ardutime.h>

void setup() {
	unsigned long long start_t, end_t;
	uRdtsc(start_t);
	delay(13000);
	uRdtsc(end_t);
	print_long_long_hex(end_t - start_t);
}

void loop() {
}
