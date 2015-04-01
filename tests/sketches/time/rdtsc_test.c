#include <ardutime.h>
#include <arduutils.h>

void setup() {
	unsigned long long start_t, end_t;
	rdtsc(&start_t);
	delay(13000);
	rdtsc(&end_t);
	print_long_long_hex(end_t - start_t);
}

void loop() {
}
