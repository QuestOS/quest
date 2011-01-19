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
#include "util/debug.h"
#include "sched/sched.h"

#ifndef NO_LOGGER
static u32 logger_stack[1024] ALIGNED(0x1000);

#define LOGGER_BUF_SIZE 65536   /* power of 2 please */
#define LOGGER_BUF_MOD_MASK (LOGGER_BUF_SIZE - 1)

static volatile bool logger_running = FALSE, dropped = FALSE;
static char logger_buf[LOGGER_BUF_SIZE] ALIGNED(0x1000);
static volatile u32 head = 0, tail = LOGGER_BUF_SIZE - 1;

static char
logger_getc (void)
{
  u32 newtail = (tail + 1) & LOGGER_BUF_MOD_MASK;
  if (newtail == head) {
    /* nothing to get */
    return 0;
  } else {
    char c = logger_buf[newtail];
    tail = newtail;
    return c;
  }
}

static void
logger_thread (void)
{
  logger_running = TRUE;

  /* main point of logger thread is that it runs in kernel space but
   * with interrupts enabled */
  unlock_kernel ();
  sti ();

  com1_printf ("logger: hello from 0x%x\n", str ());
  for (;;) {
    char c = logger_getc ();
    if (c == 0)
      asm volatile ("pause");
    else
      com1_putc (c);
    if (dropped) {
      printf ("***logger dropped char (h=%d,t=%d)***\n", head, tail);
      dropped = FALSE;
    }
  }
}
#endif

extern bool
logger_init (void)
{
#ifndef NO_LOGGER
  task_id id =
    start_kernel_thread ((u32) logger_thread, (u32) &logger_stack[1023]);

  uint lowest_priority_vcpu (void);
  lookup_TSS (id)->cpu = lowest_priority_vcpu ();

#endif
  return TRUE;
}

extern void
logger_putc (char c)
{
#ifdef NO_LOGGER
  com1_putc (c);
#else
  if (logger_running) {
    u32 newhead = (head + 1) & LOGGER_BUF_MOD_MASK;
    if (newhead == tail) {
      /* drop char */
      dropped = TRUE;
    } else {
      logger_buf[head] = c;
      head = newhead;
    }
  } else
    com1_putc (c);
#endif
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = logger_init
};

DEF_MODULE (logger, "Log manager", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
