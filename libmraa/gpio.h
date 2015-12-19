#ifndef __GPIO__
#define __GPIO__

#include "syscall.h"

enum {PIN_MODE, GPIO_WRITE};
enum {OUTPUT, INPUT};

static int
gpio_write(int pin, int value)
{
	return make_gpio_syscall(GPIO_WRITE, pin, value, 0);
}

static int
gpio_mode(int pin, int mode)
{
	return make_gpio_syscall(PIN_MODE, pin, mode, 0);
}

#endif
