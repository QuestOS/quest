/* -*- Mode: C; c-file-style: "gnu"; c-basic-offset: 2 -*- */

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
  int var = 0;

  if ((pid = fork ())) {
    waitpid (pid);
    print ("parent (");
    putx (var);
    print (")\n");
    _exit (0);
  } else {
    print ("child          \n");
    var = 1;
    _exit (0);
  }
}

/* vi: set et sw=2 sts=2: */
