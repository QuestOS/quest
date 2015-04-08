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

#include "cy8c9540a.h"
#include "drivers/gpio/quark_gpio.h"
#include "sched/sched.h"
#include "sched/vcpu.h"
#include "util/printf.h"

#define PIN_MODE        0
#define DIG_WRITE       1
#define DIG_READ        2
#define PWM             3
#define INTERRUPT_REG   4
#define INTERRUPT_WAIT  5
#define FAST_DIG_WRITE 			6
#define FAST_DIG_READ  			7

#define OUTPUT  0
#define INPUT   1
#define FAST_OUTPUT 2
#define FAST_INPUT 3
#define HIGH    1
#define LOW     0

#ifdef DEBUG_GPIO_SYSCALL
#define DLOG(fmt,...) DLOG_PREFIX("SYSCALL",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

static int pwm_enabled[14] = {0};

int
gpio_handler(int operation, int gpio, int val, int extra_arg)
{
	int ret;
  u8 quark_gpio_pin;
  DLOG("op: %d, gpio: %d, val: %d, extra_arg: %d\n",
      operation, gpio, val, extra_arg);

	switch(operation) {
		case PIN_MODE:
			if (val == OUTPUT) {
        ret = cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
        if (ret < 0)
          return ret;
				return cy8c9540a_gpio_direction_output(gpio, 0);
      } else if (val == INPUT) {
				return cy8c9540a_gpio_direction_input(gpio);
      } else {
        /* fast mdoe, select the right multiplex line */
        cy8c9540a_fast_gpio_mux(gpio);
        /* set the direction */
        quark_gpio_pin = (gpio == 16) ? 6 : 7;
        int out = (val == FAST_OUTPUT) ? 1 : 0;
        printf("1\n");
        quark_gpio_direction(quark_gpio_pin, out);
        break;
      }
		case DIG_WRITE:
			cy8c9540a_gpio_set_value(gpio, val);
      break;
		case DIG_READ:
			return cy8c9540a_gpio_get_value(gpio);
    case FAST_DIG_WRITE:
      printf("syscall: gpio is %d, val is %d\n",
          gpio, val);
      quark_gpio_pin = (gpio == 2) ? 6 : 7;
      quark_gpio_write(quark_gpio_pin, val);
      break;
    case FAST_DIG_READ:
      quark_gpio_pin = (gpio == 2) ? 6 : 7;
      return quark_gpio_read(quark_gpio_pin);
    case PWM:
      if (pwm_enabled[gpio] == 0) {
        cy8c9540a_pwm_enable(gpio);
        pwm_enabled[gpio] = 1;
      }
      val = (PERIOD * val) / 255;
      cy8c9540a_pwm_config(gpio, val, PERIOD);
      break;
    case INTERRUPT_REG:
      cy8c9540a_register_interrupt(gpio, val, extra_arg);
      break;
    case INTERRUPT_WAIT:
      cy8c9540a_wait_interrupt(gpio);
      break;
		default:
			printf("Unsupported operation!");
			return -1;
	}

	return 0;
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
