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

#include <stdio.h>
#include <stdlib.h>

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

static int
scanline (char *line)
{

  char c;
  int count = 0;

  while ((c = getchar ()) != '\n' && count < 80) {
#ifdef USE_VMX
    if (c == '.') {
      switch_screen (1);
      continue;
    } else if (c == ',') {
      switch_screen (0);
      continue;
    }
#endif
    *line++ = c;
    count++;
    putchar (c);
  }

  *line = '\0';

  putchar ('\n');

  return 1;
}


int
main ()
{

  char line[80];
  char *command_line_args = line;
  char *p;
  unsigned child_pid;

  while (1) {

    /* The shell prompt */

    putchar ('>');

    /* Wait for command line input */
    /* --??-- Assume user has entered command via keyboard */

    if (scanline (line)) {      /* Got input */
      /* --??-- Parse input and verify it is meaningful */

      if (*line == '\0')
        continue;

      /* Fork new process */
      if ((child_pid = fork ())) {      /* Parent */

#if DEBUG
        for (p = "parent!\n"; *p; p++)
          putchar (*p);
#endif
        /* switch_to( child_pid ); */
        waitpid (child_pid);
      } else {                  /* Child */

#if DEBUG
        for (p = "child!\n"; *p; p++)
          putchar (*p);
#endif
        exec (line, &command_line_args);

        /* Got here due to exec failing on an ext2fs_dir call  */
        for (p = "Error: file not found\n"; *p; p++)
          putchar (*p);

        _exit (1);
      }
    }
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
