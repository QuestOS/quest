#include <stdio.h>    
#include <stdlib.h>    
#include <string.h>
#include <stdint.h>
#include <vcpu.h>

#define RDTSC(var)                                              \
  {                                                             \
    uint32_t var##_lo, var##_hi;                                \
    asm volatile("rdtsc" : "=a"(var##_lo), "=d"(var##_hi));     \
    var = var##_hi;                                             \
    var <<= 32;                                                 \
    var |= var##_lo;                                            \
  }

int
main (int argc, char *argv[])    
{    
  int i, j, count = 0;
  unsigned long long start = 0, end = 0, ms_start = 0, ms_end = 0;
  int NUM_NB = 0;
  unsigned int number = 0;

  struct sched_param s_params = {.type = MAIN_VCPU, .C = 40, .T = 100};
  int new_vcpu = vcpu_create(&s_params);
  if(new_vcpu < 0) {
    printf("Failed to create vcpu\n");
    exit(1);
  }
  vcpu_bind_task(new_vcpu);

  if (argc != 2) {
    printf ("Usage: find_prime length_of_array\n");
    exit (0);
  }

  NUM_NB = atoi (argv[1]);
  printf ("Find prime in 1 to %d\n", NUM_NB);

  //gettimeofday (&tval_start, NULL);
  RDTSC (start);

  for (i = 0; i < NUM_NB; i++) {
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

  RDTSC (end);
  //gettimeofday (&tval_end, NULL);

  printf ("Counting finished! %d prime numbers found.\n", count);
  printf ("Cycles: %lld\n", end - start);
  //ms_start = tval_start.tv_sec * 1000 + tval_start.tv_usec / 1000;
  //ms_end = tval_end.tv_sec * 1000 + tval_end.tv_usec / 1000;
  //printf ("Time: %lld milliseconds\n", ms_end - ms_start);

  return 0;
}    

/* vi: set et ai sw=2 sts=2: */
