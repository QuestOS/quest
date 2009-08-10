/* -*- Mode: C -*- */

#include <stdlib.h>
#include <stdio.h>

int
main ()
{

  char *p;

  for (p = "In exec'd program!\n"; *p; p++)
    putchar (*p);

  printf ("memory = %d\n", meminfo ());

  return 0;
}
