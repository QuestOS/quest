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

/* Pololu serial servo controller driver */

#include "arch/i386.h"
#include "arch/i386-percpu.h"
#include "util/printf.h"
#include "mem/physical.h"
#include "mem/virtual.h"
#include "kernel.h"
#include "drivers/serial/pololu.h"

uint16_t pololu_ports[NUM_POLOLU_PORTS] = {0xEE00};

static pololu_cmd_t pololu_cmd = {0x80, 0x01, 0x0, 0x0, 0x0, 0x0};

bool pololu_init (void);

int
pololu_send_cmd (uint32_t ssc, uint8_t servo, uint8_t cmd, uint8_t data1, uint8_t data2)
{
  int i = 0, clen = 0;
  uint8_t * c = (uint8_t *) &pololu_cmd;

  if (ssc >= NUM_POLOLU_PORTS) {
    com1_printf ("Pololu controller does not exist!\n");
    return -1;
  }

  switch (cmd) {
    case POLOLU_CMD_INIT:
      pololu_init ();
      return 0;
    case POLOLU_CMD_SETPARAM:
    case POLOLU_CMD_SETSPD:
    case POLOLU_CMD_SETPOS1:
      clen = 5;
      break;
    case POLOLU_CMD_SETPOS2:
    case POLOLU_CMD_SETPOSABS:
    case POLOLU_CMD_SETNEUTRAL:
      clen = 6;
      break;
    default:
      com1_printf ("Invalid command!\n");
      return -1;
  }

  pololu_cmd.cmd = cmd;
  pololu_cmd.servo_num = servo;
  pololu_cmd.data1 = data1;
  pololu_cmd.data2 = data2;

  for (i = 0; i < clen; i++) {
    while (!(inb (pololu_ports[ssc] + 5) & 0x20));
    outb (c[i], pololu_ports[ssc]);
  }

  return 0;
}

int
pololu_enable (uint32_t ssc, uint8_t servo)
{
  /* Set range to 15, this is the default. */
  return pololu_send_cmd (ssc, servo, POLOLU_CMD_SETPARAM, 0x4F, 0x0);
}

int
pololu_disable (uint32_t ssc, uint8_t servo)
{
  return pololu_send_cmd (ssc, servo, POLOLU_CMD_SETPARAM, 0x0, 0x0);
}

int
pololu_set_abspos (uint32_t ssc, uint8_t servo, uint32_t pos)
{
  if ((pos < 500) || (pos > 5500)) {
    com1_printf ("Position out of range!\n");
    return -1;
  }

  return pololu_send_cmd (ssc, servo, POLOLU_CMD_SETPOSABS,
                          (uint8_t) (pos / 128), (uint8_t) (pos % 128));
}

bool
pololu_init (void)
{
  int i = 0;

  com1_printf ("Initializing Pololu Serial Controller...\n");

  /* Initialize Serial Port(s) */
  for (i = 0; i < NUM_POLOLU_PORTS; i++) {
    outb (0, pololu_ports[i] + 1);     /* Turn off interrupts */

    /* Communication Settings */
    outb (0x80, pololu_ports[i] + 3);  /* SET DLAB ON */
    /* Set Baud to 38400. Pololu accepts 2000 - 40000. */
    outb (0x03, pololu_ports[i] + 0);  /* Set Baud rate - Divisor Latch Low Byte */
    /* 0x01 = 115,200 BPS */
    /* 0x02 =  57,600 BPS */
    /* 0x03 =  38,400 BPS */
    /* 0x06 =  19,200 BPS */
    /* 0x0C =   9,600 BPS */
    /* 0x18 =   4,800 BPS */
    /* 0x30 =   2,400 BPS */
    outb (0x00, pololu_ports[i] + 1);  /* Set Baud rate - Divisor Latch High Byte */
    outb (0x03, pololu_ports[i] + 3);  /* 8 Bits, No Parity, 1 Stop Bit (8N1) */
    outb (0xC7, pololu_ports[i] + 2);  /* FIFO Control Register */
    outb (0x0B, pololu_ports[i] + 4);  /* Turn on DTR, RTS, and OUT2 */
  }

  pololu_enable (0, 0);
  pololu_set_abspos (0, 0, 1500);
  tsc_delay_usec (3000000);
  pololu_set_abspos (0, 0, 4500);

  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = pololu_init
};

DEF_MODULE (pololu, "Pololu Serial Servo Controller Driver", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
