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

#include "syscall.h"

#define NUM_THREADS 4

void
child (int idx)
{
  unsigned long i = 0;
  char ch;
  if (idx <= 9)
    ch = idx + '0';
  else
    ch = (idx - 10) + 'A';
  for (;;) {
    if (!(i & 0xFFFFF)) {
      putchar (ch);
    }
    i++;
  }
}

void
_start ()
{
  int pid[NUM_THREADS];
  int i;

  for (i=1; i<NUM_THREADS; i++) {
    if ((pid[i]=fork ()) == 0) {
      child (i);
    }
  }
  child (0);
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
