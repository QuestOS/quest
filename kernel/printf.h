#ifndef _PRINTF_H_
#define _PRINTF_H_

#include"acpi.h"

void closure_vprintf(void putc_clo(void *,char), void *data, const char *fmt, va_list args);
void fun_vprintf(void putc(char), const char *fmt, va_list args);
void fun_printf(void putc(char), const char *fmt, ...);

#endif
