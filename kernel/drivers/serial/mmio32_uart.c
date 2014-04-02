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

#include "drivers/serial/serial.h"
#include "arch/i386.h"
#include "kernel.h"
#include "mem/physical.h"
#include "mem/virtual.h"

static uint32_t mmio_base = MMIO32_MEMBASE;

static void
mmio32_out (uint32_t port, int offset, int value)
{
  *((volatile uint32_t *) (port + (offset << 2))) = value;
}

static int
mmio32_in (uint32_t port, int offset)
{
  return *((volatile int *) (port + (offset << 2)));
}

static void
wait_for_xmitr (uint32_t port)
{
  unsigned int status;

  for (;;) {
    status = mmio32_in (port, UART_LSR);
    if ((status & BOTH_EMPTY) == BOTH_EMPTY)
      return;
    /* asm volatile ("rep;nop": : :"memory"); */
  }
}

static void
serial_putc (uint32_t port, int c)
{
  wait_for_xmitr (port);
  mmio32_out (port, UART_TX, c);
}

void
mmio32_putc (char c)
{
  if (c == '\n') serial_putc (mmio_base, '\r');
  serial_putc (mmio_base, c);
}

int
mmio32_getc ()
{
  /* TODO: Implement getc */
  for (;;);
  return 0;
}

unsigned int
probe_baud(uint32_t port)
{
  unsigned char lcr, dll, dlm;
  unsigned int quot;

  lcr = mmio32_in (port, UART_LCR);
  mmio32_out (port, UART_LCR, lcr | UART_LCR_DLAB);
  dll = mmio32_in (port, UART_DLL);
  dlm = mmio32_in (port, UART_DLM);
  mmio32_out (port, UART_LCR, lcr);

  quot = (dlm << 8) | dll;
  return (BASE_BAUD) / quot;
}

void
initialize_serial_mmio32 (void)
{
  uint32_t port = mmio_base;
  unsigned int divisor;
  unsigned char c;
  
  mmio32_out (port, UART_LCR, 0x3);      /* 8n1 */
  mmio32_out (port, UART_IER, 0);        /* no interrupt */
  mmio32_out (port, UART_FCR, 0);        /* no fifo */
  mmio32_out (port, UART_MCR, 0x3);      /* DTR + RTS */
  
  divisor = DIV_ROUND_CLOSEST (BASE_BAUD * 16, 16 * probe_baud (mmio_base));
  //divisor = DIV_ROUND_CLOSEST (BASE_BAUD * 16, 16 * BAUD);
  c = mmio32_in (port, UART_LCR);
  mmio32_out (port, UART_LCR, c | UART_LCR_DLAB);
  mmio32_out (port, UART_DLL, divisor & 0xff);
  mmio32_out (port, UART_DLM, (divisor >> 8) & 0xff);
  mmio32_out (port, UART_LCR, c & ~UART_LCR_DLAB);
}

void *
remap_serial_mmio32 (void)
{
  /* Map MMIO32 serial base into one kernel */
  mmio_base = (uint32_t) map_virtual_page (MMIO32_MEMBASE | 0x3);
  /* TODO: Remove page directory mapping */
  return (void *) mmio_base;
}

/* vi: set et sw=2 sts=2: */
