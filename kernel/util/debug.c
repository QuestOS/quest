#include "kernel.h"
#include "util/printf.h"

void
com1_putc (char c)
{
#ifdef COM1_TO_SCREEN
  _putchar (c);
#else
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


/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
