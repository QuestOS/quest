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

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
