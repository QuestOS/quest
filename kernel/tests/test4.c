#include "syscall.h"

void
putx (unsigned long l)
{

  int i, li;

  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      putchar ('A' + li - 0x0A);
    else
      putchar ('0' + li);
}

void
print (char *s)
{
  while (*s) {
    putchar (*s++);
  }
}

void
_start ()
{
  int pid;
  int info = meminfo ();
  unsigned shared_id;
  int *shared_mem;
  int i;

  print ("MEMINFO: ");
  putx (info);
  print ("\n");

  shared_id = shared_mem_alloc ();
  if (shared_id < 0) {
    _exit (1);
  }
  print ("shared_id = ");
  putx (shared_id);
  print ("\n");

  /* try to test a race condition */

#define ITERATIONS 1000000
  if ((pid = fork ())) {
    /* PARENT */

    shared_mem = shared_mem_attach (shared_id);
    if ((unsigned) shared_mem == -1) {
      shared_mem_free (shared_id);
      _exit (1);
    }
    print ("parent shared_mem = ");
    putx ((unsigned) shared_mem);
    print ("\n");

    for (i = 0; i < ITERATIONS; i++)
      (*shared_mem)--;

    if (waitpid (pid) < 0) {
      print ("waitpid returned -1\n");
    }

    print ("value = ");
    putx (*shared_mem);
    print ("\n");

    shared_mem_detach (shared_mem);

    shared_mem_free (shared_id);
    _exit (0);

  } else {
    /* CHILD */
    shared_mem = shared_mem_attach (shared_id);
    if ((unsigned) shared_mem == -1) {
      shared_mem_free (shared_id);
      _exit (1);
    }
    print ("child shared_mem = ");
    putx ((unsigned) shared_mem);
    print ("\n");

    for (i = 0; i < ITERATIONS; i++)
      (*shared_mem)++;

    shared_mem_detach (shared_mem);

    _exit (0);
  }
}
