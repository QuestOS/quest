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
  while (!(inb (PORT1 + 5) & 0x20));    /* check line status register, empty transmitter bit */
  outb (c, PORT1);
}

int getDebugChar (void)
{
  while (!(inb (PORT1 + 5) & 1));
  return inb (PORT1);
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
    while (!(inb (PORT1 + 5) & 0x20));  /* check line status register, empty transmitter bit */
    outb ('\r', PORT1);
  }

  while (!(inb (PORT1 + 5) & 0x20));    /* check line status register, empty transmitter bit */
  outb (c, PORT1);
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
  uint min = esp & (~0xFFF), max = min + 0x1000;
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  com1_printf ("Stacktrace:\n");
  while (ebp >= min && ebp <= max) {
    com1_printf ("%0.8X\n", *((uint32 *) (ebp + 4)));
    ebp = *((uint32 *) ebp);
  }
}

static char b64lookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
/* 11000000 22221111 33333322 */
/* aabbccdd aabbccdd aabbccdd */
/* 01234567 89012345 67890123 */
union v8 {
  struct {
    u8 a:2;
    u8 b:2;
    u8 c:2;
    u8 d:2;
  };
  struct {
    u8 ab:4;
    u8 cd:4;
  };
  struct {
    u8 a:2;
    u8 bcd:6;
  };
  struct {
    u8 abc:6;
    u8 d:2;
  };
} __attribute__((packed));

#define OUT(c) logger_putc(c)
void
base64encode_dump (u8 *str, int n)
{
  int i;
  for (i=0; i<n; i+=3) {
    union v8 *p = (union v8 *)&str[i];
    OUT (b64lookup[p[0].bcd]);
    if (i+1 < n)
      OUT (b64lookup[(p[0].a << 4) | (p[1].cd)]);
    else {
      OUT (b64lookup[p[0].a << 4]);
      OUT ('=');
      OUT ('=');
      break;
    }
    if (i+2 < n)
      OUT (b64lookup[(p[1].ab << 2) | p[2].d]);
    else {
      OUT (b64lookup[p[1].ab << 2]);
      OUT ('=');
      break;
    }
    OUT (b64lookup[p[2].abc]);
  }
}
#undef OUT

void
dump_page (u8 *addr)
{
  logger_printf ("***dump 0x%p\n", addr);
  base64encode_dump (addr, 0x1000);
  logger_printf ("\n");
}

static int getc (void)
{
  while (!(inb (PORT1 + 5) & 1));
  return inb (PORT1);
}

void
crash_debug (char *reason)
{
  extern void acpi_reboot (void);
  extern bool mp_ACPI_enabled;
  char c;
  u8 state;

  com1_printf ("Entering crash debug: %s\n", reason);
  for (;;) {
    com1_printf ("crash> ");
    c = getc ();
    com1_printf ("\nyou typed '%c' (=%d)\n", c, c);
    if (c == '6') {
      com1_printf ("REBOOTING...\n");
      tsc_delay_usec (100000);
      if (mp_ACPI_enabled) acpi_reboot ();
#define KEYBOARD_STATUS_PORT 0x64
      while (((state = inb (KEYBOARD_STATUS_PORT)) & 2) != 0);
      outb (0xFE, KEYBOARD_STATUS_PORT);
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
