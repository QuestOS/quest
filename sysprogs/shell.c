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
#include <vcpu.h>

 
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
  int child_pid;
  int vcpu_index;
  
  if(setenv("PATH","/boot/", 0)) {
    fprintf(stderr, "Failed add path to environment\n");
  }

  while (1) {

    /* The shell prompt */

    putchar ('>');

    /* Wait for command line input */
    /* --??-- Assume user has entered command via keyboard */

    if (scanline (line)) {      /* Got input */
      /* --??-- Parse input and verify it is meaningful */

      if (*line == '\0')
        continue;

      vcpu_index = vcpu_create_main(20, 100);

      if(vcpu_index < 0) {
        fprintf(stderr, "Failed to create vcpu with (C=20,T=100)\n"
                "Going to create child on best effort vcpu\n");
        child_pid = fork();
      }
      else {
        child_pid = vcpu_fork(vcpu_index);
      }

      if(child_pid < 0) {
        fprintf(stderr, "fork failed: pid = %d\n", child_pid);
      }

      /* Fork new process */
      if (child_pid) {      /* Parent */

#if DEBUG
        for (p = "parent!\n"; *p; p++)
          putchar (*p);
#endif
        /* switch_to( child_pid ); */
        waitpid (child_pid);
        
        if(vcpu_index >= 0) {
          if(vcpu_destroy(vcpu_index, 0) < 0) {
            fprintf(stderr, "Failed to destroy VCPU %d\n", vcpu_index);
          }
        }
      } else {                  /* Child */

#if DEBUG
        for (p = "child!\n"; *p; p++)
          putchar (*p);
#endif
        execvp (line, &command_line_args);

        /* Got here due to exec failing on an ext2fs_dir call  */
        for (p = "Error: file not found\n"; *p; p++)
          putchar (*p);

        exit (1);
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
