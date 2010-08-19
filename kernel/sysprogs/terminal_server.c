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

#include <stdio.h>
#include <string.h>

static char *pchVideo = (char *) 0x000200000;   /* in middle of page table */

static int
_putchar (int ch, int attribute)
{

  static int x, y;

  if (ch == '\n') {
    x = 0;
    y++;

    if (y > 24) {
      memcpy (pchVideo, pchVideo + 160, 24 * 160);
      memset (pchVideo + (24 * 160), 0, 160);
      y = 24;
    }
    return (int) (unsigned char) ch;
  }

  if (y * 160 + x * 2 >= 0x1000) return ch;

  pchVideo[y * 160 + x * 2] = ch;
  pchVideo[y * 160 + x * 2 + 1] = attribute;
  x++;

  if (y * 160 + x * 2 >= 0x1000) return ch;

  pchVideo[y * 160 + x * 2] = ' ';
  pchVideo[y * 160 + x * 2 + 1] = attribute;

  /* Move cursor */
  outb (0x0E, 0x3D4);           /* CRTC Cursor location high index */
  outb ((y * 80 + x) >> 8, 0x3D5);      /* CRTC Cursor location high data */
  outb (0x0F, 0x3D4);           /* CRTC Cursor location low index */
  outb ((y * 80 + x) & 0xFF, 0x3D5);    /* CRTC Cursor location low data */

  return (int) (unsigned char) ch;
}


void
splash_screen (void)
{

  char line1[80];
  char line2[80];
  char *p;
  int i;

  uname (line2);

  sprintf (line1,
           "**** Quest kernel version: %s *****   //---\\ \\\\  \\ //-- //--\\ \\\\---\\ \n",
           line2);
  for (p = line1; *p; p++)
    _putchar (*p, 4);

  sprintf (line1,
           "* Copyright Boston University, 2005 *   ||   | ||  | ||-- \\\\--\\   || \n");
  for (p = line1; *p; p++)
    _putchar (*p, 4);

  sprintf (line1, "%u", meminfo ());
  i = strlen (line1);

  for (i = strlen (line1); i < 16; i++)
    line2[i - strlen (line1)] = '*';

  sprintf (line1,
           "******** %u bytes free %s   \\\\__\\_  \\\\_/ \\\\__ \\\\__/   || \n",
           meminfo (), line2);
  for (p = line1; *p; p++)
    _putchar (*p, 4);
}


int
main ()
{

  int arg;

  asm volatile ("movl %%ebx, %0":"=m" (arg));

  splash_screen ();

  /* Setup cursor to be a full block in foreground colour */
  outb (0x0A, 0x3D4);           /* CRTC Cursor start index */
  outb (0x00, 0x3D5);           /* CRTC Cursor start data */
  outb (0x0B, 0x3D4);           /* CRTC Cursor end index */
  outb (0x0E, 0x3D5);           /* CRTC Cursor end data */

  while (1) {
    _putchar (arg, 7);

    asm volatile ("iret");

    asm volatile ("movl %%ebx, %0":"=m" (arg));
  }

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
