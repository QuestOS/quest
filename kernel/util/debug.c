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

#include "kernel.h"
#include "util/printf.h"

#ifndef GDBSTUB_TCP
void putDebugChar (int c)
{
  while (!(inb (serial_port1 + 5) & 0x20));    /* check line status register, empty transmitter bit */
  outb (c, serial_port1);
}

int getDebugChar (void)
{
  while (!(inb (serial_port1 + 5) & 1));
  return inb (serial_port1);
}
#endif

void
com1_putc (char c)
{
#ifdef COM1_TO_SCREEN
  _putchar (c);
#elif !defined(ENABLE_GDBSTUB) || defined(GDBSTUB_TCP)
  if (c == '\n') {
    /* output CR before NL */
    while (!(inb (serial_port1 + 5) & 0x20));  /* check line status register, empty transmitter bit */
    outb ('\r', serial_port1);
  }

  while (!(inb (serial_port1 + 5) & 0x20));    /* check line status register, empty transmitter bit */
  outb (c, serial_port1);
#endif
}

void
com1_puts (char *p)
{
  while (*p)
    com1_putc (*p++);
}

void
com1_putx (uint32 l)
{
  int i, li;

  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      com1_putc ('A' + li - 0x0A);
    else
      com1_putc ('0' + li);
}

void
stacktrace (void)
{
  uint32 esp, ebp;
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  asm volatile ("movl %%esp, %0":"=r" (esp));
  asm volatile ("movl %%ebp, %0":"=r" (ebp));
  com1_printf ("Stacktrace:\n");
  while (ebp >= KERN_STK && ebp <= KERN_STK + 0x1000) {
    com1_printf ("%0.8X\n", *((uint32 *) (ebp + 4)));
    ebp = *((uint32 *) ebp);
  }
}

void
stacktrace_frame (uint esp, uint ebp)
{
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  com1_printf ("Stacktrace:\n");
  while (ebp >= KERN_STK && ebp <= KERN_STK + 0x1000) {
    com1_printf ("%0.8X\n", *((uint32 *) (ebp + 4)));
    ebp = *((uint32 *) ebp);
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
