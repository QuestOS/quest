#include "arch/i386.h"
#include "kernel.h"
#include "drivers/pci/pci.h"

static uint16 serial_port1 = 0x03F8;

void
serial_putc (char c)
{
	if (c == '\n') {
    /* output CR before NL */
    while (!(inb (serial_port1 + 5) & 0x20));  /* check line status register, empty transmitter bit */
    outb ('\r', serial_port1);
  }


  while (!(inb (serial_port1 + 5) & 0x20));    /* check line status register, empty transmitter bit */
  outb (c, serial_port1);
}

int serial_getc (void)
{
  unlock_kernel();
  sti();
  while (!(inb (serial_port1 + 5) & 1));
  cli();
  lock_kernel();
  return inb (serial_port1);
}

void
initialize_serial_port (void)
{
  outb (0, serial_port1 + 1);          /* Turn off interrupts - Port1 */

  /*         PORT 1 - Communication Settings         */

  outb (0x80, serial_port1 + 3);       /* SET DLAB ON */
  //outb (0x03, serial_port1 + 0);       /* Set Baud rate - Divisor Latch Low Byte */
  outb (0x01, serial_port1 + 0);       /* Set Baud rate - Divisor Latch Low Byte */
  /* Default 0x03 =  38,400 BPS */
  /*         0x01 = 115,200 BPS */
  /*         0x02 =  57,600 BPS */
  /*         0x06 =  19,200 BPS */
  /*         0x0C =   9,600 BPS */
  /*         0x18 =   4,800 BPS */
  /*         0x30 =   2,400 BPS */
  outb (0x00, serial_port1 + 1);       /* Set Baud rate - Divisor Latch High Byte */
  outb (0x03, serial_port1 + 3);       /* 8 Bits, No Parity, 1 Stop Bit */
  outb (0xC7, serial_port1 + 2);       /* FIFO Control Register */
  outb (0x0B, serial_port1 + 4);       /* Turn on DTR, RTS, and OUT2 */
  //com1_puts ("COM1 initialized.\n");
  //
#ifdef MINNOWMAX
  /* enable ttyS0 on MinnowMax */
  WRITE(0, 31, 0, 0x80, dword, 1);
#endif
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

