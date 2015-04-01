#include <ardutime.h>
#include <arduutils.h>
#include <arduthread.h>
#include <pin.h>

int pin_status = LOW;
int BLINK_NB = 2000;

unsigned long long
find_prime (int NUM_NB)    
{    
  int i, j, count = 0;
  unsigned long long start = 0, end = 0;
  unsigned int number = 0;

  printf ("Find prime in 1 to %d\n", NUM_NB);

  rdtsc(&start);
  for (i = 0; i < NUM_NB; i++) {

		/* mix digital io with find_prime */
		if (i < BLINK_NB)
			digitalWrite(13, pin_status = !pin_status);

    number++;
    if (number <= 3) {
      if ((number == 2) || (number == 3)) {
        count++;
        continue;
      } else {
        continue;
      }
    }
    if ((number % 2) == 0) continue;
    for (j = 3; j < number; j += 2) {
      if ((number % j) == 0) break;
    }
    if (j >= number) count++;
  }
  rdtsc(&end);

  printf ("Counting finished! %d prime numbers found.\n", count);
  return (end - start);
}

void loop(1,495,500) {
	unsigned long long res;
	int N = 80000;

	res = find_prime(N);
  printf("C=80, Cycles: ");
	print_long_long_hex(res);
	while (1);
}

void setup()
{
	pinMode(13, OUTPUT);
}
