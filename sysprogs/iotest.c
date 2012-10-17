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

#define NUM_THREADS 4
#define USB
#define USB_FILE "(usb)/test"
#define CDROM
#define CDROM_FILE "(cd)/gpxe.krn"
//#define CDROM_FILE "(cd)/boot/quest"

char buf[65536];
void *memset( void *p, int ch, size_t cb );

void
child (int idx)
{
  unsigned long i = 0, j;
  char ch;
  if (idx <= 9)
    ch = idx + '0';
  else
    ch = (idx - 10) + 'A';
  for (;;) {
    if (!(i & 0xFFFFF)) {
      memset (buf, ch, sizeof (buf));
      if (idx == 0) {
#ifdef USB
        open (USB_FILE, 0);
        read (USB_FILE, buf, sizeof (buf));
        for (j=0; j<sizeof (buf)-1; j++)
          if (buf[j] != 'a')
            printf ("buf[%d] == 'a'\n", j, buf[j]);
#endif
        putchar (buf[0]);
      } else if (idx == 1) {
#ifdef CDROM
        unsigned long sum = 0;
        open (CDROM_FILE, 0);
        read (CDROM_FILE, buf, sizeof (buf));
        printf ("%x", buf[0x21] & 0xFF);
        for (j=0; j<sizeof (buf); j++)
          sum += (unsigned char) buf[j];
#else
        putchar (buf[0]);
#endif
      } else {
        putchar (buf[0]);
      }
    }
    i++;
  }
}

int
main (void)
{
  int pid[NUM_THREADS];
  int i;

  for (i=1; i<NUM_THREADS; i++) {
    if ((pid[i]=fork ()) == 0) {
      child (i);
    }
  }

  child (0);
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
