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

#ifndef _PRINTF_H_
#define _PRINTF_H_

#include "acpi.h"
#include "util/screen.h"
#include "util/debug.h"


void closure_vprintf (void putc_clo (void *, char), void *data,
                      const char *fmt, va_list args);
void fun_vprintf (void putc (char), const char *fmt, va_list args);
void fun_printf (void putc (char), const char *fmt, ...);
void com1_printf (const char *fmt, ...);
void logger_printf (const char *fmt, ...);
void printf (const char *fmt, ...);
void _printf (const char *fmt, ...);

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
