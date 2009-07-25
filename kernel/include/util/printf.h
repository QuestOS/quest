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
void printf (const char *fmt, ...);
void _printf (const char *fmt, ...);

#endif
