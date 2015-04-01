#include <pin.h>
#include <ardutime.h>
#include <arduutils.h>
#include <arduthread.h>
#include <stdio.h>

int pin_status = LOW;
int BLINK_NB = 2000;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
}

void loop(1,490,500) {
  // put your main code here, to run repeatedly: 
  int i;
  unsigned long long start_t, end_t, duration;
  
  //delay(3120);
  //start experiment
  printf("starting...");
  rdtsc(&start_t);
  for (i = 1; i <= BLINK_NB; i++)
    digitalWrite(13, pin_status = !pin_status);
  rdtsc(&end_t);
  duration = end_t - start_t;
  print_long_long_hex(duration);
  while(1);
}
