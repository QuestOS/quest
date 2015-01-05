#include <gpio.h>
#include <pthread.h>
#include <stdlib.h>
#include <vcpu.h>

#ifndef _DIGITAL_IO_H_
#define _DIGITAL_IO_H_

/* command */
#define PIN_MODE  			0
#define DIG_WRITE 			1
#define DIG_READ  			2
#define PWM 						3
#define INTERRUPT_REG 	4
#define INTERRUPT_WAIT 	5

#define OUTPUT  0
#define INPUT   1
#define HIGH    1
#define LOW     0

#define CHANGE 0

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
  return gpio_syscall(PIN_MODE, arduino2galileo_gpio_mapping[pin], mode, 0);
}

int
digitalRead(int pin)
{
  /* XXX: Need pin number error-checking */
  return gpio_syscall(DIG_READ, arduino2galileo_gpio_mapping[pin], 0, 0);
}

int
digitalWrite(int pin, int value)
{
  /* XXX: Need pin number error-checking */
  if (value != HIGH && value != LOW)
    return -1;
  gpio_syscall(DIG_WRITE, arduino2galileo_gpio_mapping[pin], value, 0);
	return 0;
}

int 
analogWrite(int pin, int value)
{
  gpio_syscall(PWM, gpio2pwm_mapping[arduino2galileo_gpio_mapping[pin]], value, 0);
	return 0;
}

struct isr_args {
	void (*handler)(void);
	int mode;
	int pin;
	int C;
	int T;
};

static void *isr(void *args)
{
	struct isr_args *local_args;
	int pin, mode, new_vcpu;
	void (*handler)(void);
	struct sched_param s_params;

	local_args = (struct isr_args *)args;
	pin = local_args->pin;
	mode = local_args->mode;
	handler = local_args->handler;
	s_params.C = MAIN_VCPU;
	s_params.C = local_args->C;
	s_params.T = local_args->T;

	/* create new main vcpu for handler */
  new_vcpu = vcpu_create(&s_params);
	if(new_vcpu < 0) {
    printf("Failed to create vcpu. Use best effort vcpu\n");
  }
  vcpu_bind_task(new_vcpu);

	/* Register pin to driver.
	 * Notice that pin is not converted to kernel-aware
	 * pin number as other syscalls do
	 */
	gpio_syscall(INTERRUPT_REG, pin, mode, s_params.T);

	//wait for interrupt
	while (1) {
		gpio_syscall(INTERRUPT_WAIT, pin, 0, 0);
		handler();
	}
	free(args);
}

int
attachInterrupt_vcpu(int pin, void (*handler)(void), int mode,
		int C, int T)
{
	pthread_t isr_thread;
	struct isr_args *args;
		
	args = malloc(sizeof (struct isr_args));
	args->pin = pin;
	args->handler = handler;
	args->mode = mode;
	args->C = C;
	args->T = T;
	pthread_create(&isr_thread, NULL, isr, (void *)args);
}

int
attachInterrupt(int pin, void (*handler)(void), int mode)
{
	attachInterrupt_vcpu(pin, handler, mode, 10, 100);
}

#endif
