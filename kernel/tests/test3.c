/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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



  if ((pid = fork ())) {
    /* PARENT */
    if (waitpid (pid) < 0) {
      print ("waitpid returned -1\n");
    }

    shared_mem = shared_mem_attach (shared_id);
    if ((unsigned) shared_mem == -1) {
      shared_mem_free (shared_id);
      _exit (1);
    }
    print ("parent shared_mem = ");
    putx ((unsigned) shared_mem);
    print ("\n");

    print ("PARENT (");
    putx (*shared_mem);
    print (")\n");

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

    *shared_mem = 1;

    shared_mem_detach (shared_mem);

    _exit (0);
  }
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
