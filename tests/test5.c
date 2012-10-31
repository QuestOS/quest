/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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

#include "stdio.h"
#include "unistd.h"

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
main ()
{
  int pid;
  if ((pid = fork ())) {
    print ("00000000");
    waitpid (pid);
    print ("\n");
    _exit (0);
  } else if ((pid = fork ())) {
    print ("11111111");
    waitpid (pid);
    _exit (0);
  } else if ((pid = fork ())) {
    print ("22222222");
    waitpid (pid);
    _exit (0);
  } else if ((pid = fork ())) {
    print ("33333333");
    waitpid (pid);
    _exit (0);
  } else if ((pid = fork ())) {
    print ("44444444");
    waitpid (pid);
    _exit (0);
  } else {
    print ("55555555");
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
