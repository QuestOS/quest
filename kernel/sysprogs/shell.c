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

static int
scanline (char *line)
{

  char c;
  int count = 0;

  while ((c = getchar ()) != '\n' && count < 80) {
    *line++ = c;
    count++;
    putchar (c);
  }

  *line = '\0';

  putchar ('\n');

  return 1;
}


void
_start ()
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
