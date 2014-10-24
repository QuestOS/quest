#include <gpio.h>

#ifndef _DIGITAL_IO_H_
#define _DIGITAL_IO_H_

/* command */
#define PIN_MODE  0
#define DIG_WRITE 1
#define DIG_READ  2
#define PWM 			3

#define OUTPUT  0
#define INPUT   1
#define HIGH    1
#define LOW     0

#define PERIOD 255

/* driver is using galileo pin number
 * users are using arduino pin number
 */
static const int arduino2galileo_gpio_mapping[] = {
  34, 35, 16, 2, 12, 1, 8,
  11, 10, 3, 0, 9, 22, 23
};

/* driver is using pwm number
 * users are using arduino gpio number
 */
#define GPIO_UNUSED			-1
static const int gpio2pwm_mapping[] = {
	7,
	5,
	3,
	1,
	GPIO_UNUSED,
	GPIO_UNUSED,
	GPIO_UNUSED,
	GPIO_UNUSED,
	6,
	4,
};

int
pinMode(int pin, int mode)
{
  /* XXX: Need pin number error-checking */
  if (mode != OUTPUT && mode != INPUT)
    return -1;
  return gpio_syscall(PIN_MODE, arduino2galileo_gpio_mapping[pin], mode);
}

int
digitalRead(int pin)
{
  /* XXX: Need pin number error-checking */
  return gpio_syscall(DIG_READ, arduino2galileo_gpio_mapping[pin], 0);
}

int
digitalWrite(int pin, int value)
{
  /* XXX: Need pin number error-checking */
  if (value != HIGH && value != LOW)
    return -1;
  gpio_syscall(DIG_WRITE, arduino2galileo_gpio_mapping[pin], value);
	return 0;
}

int 
analogWrite(int pin, int value)
{
  gpio_syscall(PWM, gpio2pwm_mapping[arduino2galileo_gpio_mapping[pin]], value);
	return 0;
}

#endif
