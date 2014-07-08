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

#ifndef _GPIO_H_
#define _GPIO_H_

#define PIN_MODE  0
#define DIG_WRITE 1
#define DIG_READ  2

#define OUTPUT  0
#define INPUT   1
#define HIGH    1
#define LOW     0

#define CLOBBERS1 "memory","cc","%ebx","%ecx","%edx","%esi","%edi"
#define CLOBBERS2 "memory","cc","%ecx","%edx","%esi","%edi"
#define CLOBBERS3 "memory","cc","%ebx","%edx","%esi","%edi"
#define CLOBBERS4 "memory","cc","%ebx","%ecx","%esi","%edi"
#define CLOBBERS5 "memory","cc","%edx","%esi","%edi"
#define CLOBBERS6 "memory","cc","%esi","%edi"
#define CLOBBERS7 "memory","cc","%edi"

static inline int gpio_syscall(int operation, int arg1, int arg2)
{
  int ret;
  asm volatile ("int $0x30\n":"=a" (ret) : "a" (12L),
      "b"(operation), "c" (arg1), "d" (arg2), "S" (0) : CLOBBERS7);
  return ret;
}

static inline int pinMode(int pin, int mode)
{
  /* XXX: Need pin number error-checking */
  if (mode != OUTPUT && mode != INPUT)
    return -1;
  return gpio_syscall(PIN_MODE, pin, mode);
}

static inline int digitalRead(int pin)
{
  /* XXX: Need pin number error-checking */
  return gpio_syscall(DIG_READ, pin, 0);
}

static inline int digitalWrite(int pin, int value)
{
  /* XXX: Need pin number error-checking */
  if (value != HIGH && value != LOW)
    return -1;
  gpio_syscall(DIG_WRITE, pin, value);
  return 0;
}
#endif
/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
