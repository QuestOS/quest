#include <stdio.h>

int
main ()
{

  int i, child, grandchild;

  if ((child = fork ())) {
    /* parent */
    for (i = 0; i < 10; i++)
      puts ("Parent!");

    waitpid (child);
  } else if ((grandchild = fork ())) {
    /* child */
    for (i = 0; i < 10; i++)
      puts ("Child!");

    waitpid (grandchild);
  } else
    /* grandchild */
    for (i = 0; i < 10; i++)
      puts ("Grandchild!");

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
