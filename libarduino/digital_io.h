#include <gpio.h>

#ifndef _DIGITAL_IO_H_
#define _DIGITAL_IO_H_

/* command */
#define PIN_MODE  0
#define DIG_WRITE 1
#define DIG_READ  2

#define OUTPUT  0
#define INPUT   1
#define HIGH    1
#define LOW     0

int
pinMode(int pin, int mode)
{
  /* XXX: Need pin number error-checking */
  if (mode != OUTPUT && mode != INPUT)
    return -1;
  return gpio_syscall(PIN_MODE, pin, mode);
}

int
digitalRead(int pin)
{
  /* XXX: Need pin number error-checking */
  return gpio_syscall(DIG_READ, pin, 0);
}

int
digitalWrite(int pin, int value)
{
  /* XXX: Need pin number error-checking */
  if (value != HIGH && value != LOW)
    return -1;
  gpio_syscall(DIG_WRITE, pin, value);
  return 0;
}

#endif
