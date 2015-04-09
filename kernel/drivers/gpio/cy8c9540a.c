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

/* cy8c9540 driver */

#include "drivers/i2c/galileo_i2c.h"
#include "util/printf.h"
#include "cy8c9540a.h"
#include "sched/sched.h"
#include "sched/vcpu.h"
#include "mem/malloc.h"

//#define DEBUG_CYPRESS

#ifdef DEBUG_CYPRESS
#define DLOG(fmt,...) DLOG_PREFIX("CYPRESS",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* CY8C9540A settings */
#define TAR_ADDR   		0x20
#define NPORTS				6
#define NPWM          8
#define PWM_MAX_PERIOD	0xff

/* Register offset  */
#define REG_INPUT_PORT0			0x00
#define REG_OUTPUT_PORT0		0x08
#define REG_INTR_STAT_PORT0		0x10
#define REG_PORT_SELECT			0x18
#define REG_INTR_MASK			0x19
#define REG_SELECT_PWM			0x1a
#define REG_PIN_DIR			0x1c
#define REG_DRIVE_PULLUP		0x1d
#define REG_PWM_SELECT			0x28
#define REG_PWM_CLK			0x29
#define REG_PWM_PERIOD			0x2a
#define REG_PWM_PULSE_W			0x2b
#define REG_ENABLE			0x2d
#define REG_DEVID_STAT			0x2e

#define BIT(nr)			(1UL << (nr))

/* Per-port GPIO offset */
static const u8 cy8c9540a_port_offs[] = {
	0,
	8,
	16,
	20,
	28,
	36,
};

/* choose clock frequency 93.75khz
 * set period to 191 so that the resulting
 * output period will be 490.8hz, very close to
 * what Arduino uses. But we have less duty cycles
 * to use */
#define PWM_CLK				0x03	

/* PWM-to-GPIO mapping (0 == first Cypress GPIO).  */
#define PWM_UNUSED			-1
static const int pwm2gpio_mapping[] = {
	PWM_UNUSED,
	3,
	PWM_UNUSED,
	2,
	9,
	1,
	8,
	0,
};

struct cy8c9540a {
	/* cached output registers */
	u8 outreg_cache[NPORTS];
	/* cached IRQ mask */
	u8 irq_mask_cache[NPORTS];
	/* IRQ mask to be applied */
	u8 irq_mask[NPORTS];
  u32 addr;
};

static struct cy8c9540a dev = {
  .addr = TAR_ADDR,
};

static inline u8
cypress_get_port(unsigned gpio)
{
  u8 i = 0;
  for (i = 0; i < sizeof(cy8c9540a_port_offs) - 1; i++) {
    if (!(gpio / cy8c9540a_port_offs[i + 1]))
      break;
  }
	//printf("port is %d\n", i);
  return i;
}

static inline u8 cypress_get_offs(unsigned gpio, u8 port)
{
	//printf("offset is %d\n", gpio - cy8c9540a_port_offs[port]);
	return gpio - cy8c9540a_port_offs[port];
}

int
cy8c9540a_gpio_get_value(unsigned gpio)
{
  s32 ret = 0;
  u8 port = 0, pin = 0, in_reg = 0;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);
  in_reg = REG_INPUT_PORT0 + port;

  ret = i2c_read_byte_data(in_reg);
  if (ret < 0) {
    logger_printf("can't read input port%u", port);
  }

  return !!(ret & BIT(pin));
}

void
cy8c9540a_gpio_set_value(unsigned gpio, int val)
{
  s32 ret = 0;
  u8 port = 0, pin = 0, out_reg = 0;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);
  out_reg = REG_OUTPUT_PORT0 + port;

  if (val) {
    dev.outreg_cache[port] |= BIT(pin);
  } else {
    dev.outreg_cache[port] &= ~BIT(pin);
  }

  ret = i2c_write_byte_data(out_reg, dev.outreg_cache[port]);

  if (ret < 0) {
    logger_printf("can't read output port%u", port);
  }
}

int
cy8c9540a_gpio_set_drive(unsigned gpio, unsigned mode)
{
  s32 ret = 0;
  u8 port = 0, pin = 0, offs = 0, val = 0;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);

  switch(mode) {
    case GPIOF_DRIVE_PULLUP:
      offs = 0x0;
      break;
    case GPIOF_DRIVE_STRONG:
      offs = 0x4;
      break;
    case GPIOF_DRIVE_HIZ:
      offs = 0x6;
      break;
    default:
      return -1;
  }

  ret = i2c_write_byte_data(REG_PORT_SELECT, port);
  if (ret < 0) {
    logger_printf("can't select port%u", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_DRIVE_PULLUP + offs);
  if (ret < 0) {
    logger_printf("can't read drive mode port%u", port);
    return ret;
  }

  val = (u8)(ret | BIT(pin));

  ret = i2c_write_byte_data(REG_DRIVE_PULLUP + offs, val);
  if (ret < 0) {
    logger_printf("can't write drive mode port%u", port);
    return ret;
  }

  return 0;
}

int
cy8c9540a_gpio_direction(unsigned gpio, int out, int val)
{
  s32 ret = 0;
  u8 pins = 0, port = 0, pin = 0;
  port = cypress_get_port(gpio);

  if (out) {
    cy8c9540a_gpio_set_value(gpio, val);
  }

  ret = i2c_write_byte_data(REG_PORT_SELECT, port);
  if (ret < 0) {
    logger_printf("can't select port%u", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_PIN_DIR);
  if (ret < 0) {
    logger_printf("can't read pin direction", port);
    return ret;
  }

  pin = cypress_get_offs(gpio, port);

  pins = (u8)ret & 0xff;
  if (out) {
    pins &= ~BIT(pin);
  } else {
    pins |= BIT(pin);
  }
  
  ret = i2c_write_byte_data(REG_PIN_DIR, pins);
  if (ret < 0) {
    logger_printf("can't write pin direction", port);
    return ret;
  }

  return 0;
}

int cy8c9540a_gpio_direction_output(unsigned gpio, int val)
{
  return cy8c9540a_gpio_direction(gpio, 1, val);
}

int cy8c9540a_gpio_direction_input(unsigned gpio)
{
  return cy8c9540a_gpio_direction(gpio, 0, 0);
}

int cy8c9540a_fast_gpio_mux(unsigned gpio)
{
	unsigned mux = (gpio == 16) ? 15 : 14;
	printf("muxing gpio %d\n", mux);
	cy8c9540a_gpio_set_drive(mux, GPIOF_DRIVE_PULLUP);
	printf("done with set dirve\n");
	cy8c9540a_gpio_direction_output(mux, 0);
	printf("done with set direction\n");
	return 0;
}

/* ------ PWM ------------ */
int cy8c9540a_pwm_config(unsigned pwm, int duty, int period)
{
	int ret;

	if (pwm > NPWM) {
		logger_printf("invalid pwm number");
		return -1;
	}

	if (duty < 0 || period <= 0 || duty > period) {
		logger_printf("invalid duty and period configuration");
		return -1;
	}
	
	if (period > PWM_MAX_PERIOD) {
		logger_printf("period must be within [0-%d]ns", PWM_MAX_PERIOD);
		return -1;
	}

	ret = i2c_write_byte_data(REG_PWM_SELECT, (u8)pwm);
	if (ret < 0) {
		logger_printf("can't write to REG_PWM_SELECT");
		return ret;
	}

	ret = i2c_write_byte_data(REG_PWM_PERIOD, (u8)period);
	if (ret < 0) {
		logger_printf("can't write to REG_PWM_PERIOD");
		return ret;
	}

	ret = i2c_write_byte_data(REG_PWM_PULSE_W, (u8)duty);
	if (ret < 0) {
		logger_printf("can't write to REG_PWM_PULSE_W");
		return ret;
	}

	return 0;
}

int cy8c9540a_pwm_switch(unsigned pwm, int enable)
{
  int ret = 0, gpio = 0;
  u8 port = 0, pin = 0;
	u8 val = 0;

	if (pwm > NPWM) {
		logger_printf("invalid pwm number");
		return -1;
	}

	gpio = pwm2gpio_mapping[pwm];
	if (PWM_UNUSED == gpio) {
		logger_printf("pwm%u is unused", pwm);
		return -1;
	}
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);

	val = i2c_read_byte_data(REG_SELECT_PWM);
	if (val < 0) {
		logger_printf("can't read REG_SELECT_PWM");
		return val;
	}

	if (enable) {
		/* Set pin as output driving high */
		ret = cy8c9540a_gpio_direction_output(gpio, 1);
		if (ret < 0) {
			logger_printf("can't set pwm%u as output", pwm);
			return ret;
		}
		val |= BIT((u8)pin);
	} else {
		val &= ~BIT((u8)pin);
	}

	ret = i2c_write_byte_data(REG_SELECT_PWM, val);
	if (ret < 0) {
		logger_printf("can't write to REG_SELECT_PWM", pwm);
		return ret;
	}

	return 0;
}

int cy8c9540a_pwm_enable(unsigned pwm)
{
	/* XXX: in CY8C9540A manual, it says
		 pwm pin must be congfigured to an 
		 appropriate mode but doesn't say which
		 one is appropriate. I choose GPIOF_DRIVE_STRONG */
  cy8c9540a_gpio_set_drive(pwm2gpio_mapping[pwm],
		 	GPIOF_DRIVE_STRONG);
	return cy8c9540a_pwm_switch(pwm, 1);
}

int cy8c9540a_pwm_disable(unsigned pwm)
{
	return cy8c9540a_pwm_switch(pwm, 0);
}

/* ****************************************************** *
 * interrupt *
 */
static uint32 cy8c9540a_interrupt_stack[1024] ALIGNED (0x1000);
#ifndef NO_GPIO_IOVCPU
task_id cy8c9540a_interrupt_pid;

static void
cy8c9540a_interrupt_thread()
{
	while (1) {
		/* sleep until interrupt comes */
		iovcpu_job_completion();
		/* unlock kernel and enable interrupt
		 * so that interrupt thread can be preempted */
		unlock_kernel();
		sti();
		void cy8c9540a_irq_handler();
		cy8c9540a_irq_handler();
		/* unmask interrupt so that interrupt from 
		 * cy8c9540a can be delivered again */
		extern void quark_gpio_unmask_cypress_interrupt();
		quark_gpio_unmask_cypress_interrupt();
		cli();
		lock_kernel();
	}
}
#endif

static int
cy8c9540a_unmask_interrupt(unsigned gpio)
{
	s32 ret = 0;
  u8 port = 0, pin = 0, val;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);

	ret = i2c_write_byte_data(REG_PORT_SELECT, port);
  if (ret < 0) {
    logger_printf("can't select port%u", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_INTR_MASK);
  if (ret < 0) {
    logger_printf("can't read interrupt mask port%u", port);
    return ret;
  }

  val = (u8)(ret & ~BIT(pin));
  ret = i2c_write_byte_data(REG_INTR_MASK, val);
  if (ret < 0) {
    logger_printf("can't write interrupt mask port%u", port);
    return ret;
  }

  return 0;
}

#define INT_PIN_NUM 14

struct int_pin_data {
	task_id waitqueue;
	u8 port;
	u8 pin;
} *int_pin_array[INT_PIN_NUM];

static const unsigned int arduino2galileo_gpio_mapping[] = {
  34, 35, 16, 2, 12, 1, 8,
  11, 10, 3, 0, 9, 22, 23
};

#ifndef NO_GPIO_IOVCPU
int gpio_handler_T_min = 1000000;
int gpio_handler_T_min_tid;
#endif

void
cy8c9540a_register_interrupt(unsigned gpio, int mode, int T)
{
	u8 port;
	unsigned kgpio;

	if (int_pin_array[gpio]) {
		logger_printf("warning: this pin has already got a handler attached.\n");
	} else {
		int_pin_array[gpio] = kmalloc(sizeof(*int_pin_array[0]));
		kgpio = arduino2galileo_gpio_mapping[gpio];
		port = cypress_get_port(kgpio);
		int_pin_array[gpio]->port = port;
		int_pin_array[gpio]->pin = cypress_get_offs(kgpio, port);
		/* unmask interrupt */
		cy8c9540a_unmask_interrupt(kgpio);
#ifndef NO_GPIO_IOVCPU
		if (T < gpio_handler_T_min) {
			gpio_handler_T_min = T;
			gpio_handler_T_min_tid = str();
		}
#endif
	}
}

void
cy8c9540a_wait_interrupt(unsigned gpio)
{
	queue_append(&(int_pin_array[gpio]->waitqueue), str());
	schedule();
}

static u8 
cy8c9540a_read_int_status(u8 port_off)
{
	u8 int_status;

	/* i2c use either polling or blocking, depending on 
	 * mp_enabled. Since this function is called
	 * in interrupt context, so set mp_enabled
	 * to 0 to make i2c do polling.
	 * XXX: this is a hack!! Some other components also
	 * depend on mp_enabled. This could cause other
	 * CPUs to detect wrong system status if SMP enabled */
#ifndef NO_GPIO_IOVCPU
	cli();
	lock_kernel();
#endif
	mp_enabled = 0;
	int_status = i2c_read_byte_data(REG_INTR_STAT_PORT0 + port_off);
	mp_enabled = 1;
#ifndef NO_GPIO_IOVCPU
	unlock_kernel();
	sti();
#endif
	return int_status;
}

void
cy8c9540a_irq_handler()
{
	u8 int_status[6], status;
	int gpio;

	/* The only way to find out which pin triggered
	 * the interrupt is to read the port's interrupt
	 * status register via i2c. It has unnegligible time
	 * cost. So try to read as few ports as possible, only
	 * those to which pins with handler attached belongs to */
	if (int_pin_array[10] || int_pin_array[5]
			|| int_pin_array[3] || int_pin_array[9]) {
		/* pin 10, 5, 3, 9 belongs to port 0 */
		int_status[0] = cy8c9540a_read_int_status(0);
	}
	if (int_pin_array[6] || int_pin_array[11]
			|| int_pin_array[8] || int_pin_array[7]
			|| int_pin_array[4]) {
		/* pin 6, 11, 8, 7, 4 belongs to port 1 */
		int_status[1] = cy8c9540a_read_int_status(1);
	}
	if (int_pin_array[2]) {
		/* pin 2 belongs to port 2 */
		int_status[2] = cy8c9540a_read_int_status(2);
	}
	if (int_pin_array[12] || int_pin_array[13]) {
		/* pin 12, 13 belongs to port 3 */
		int_status[3] = cy8c9540a_read_int_status(3);
	}
	if (int_pin_array[0] || int_pin_array[1]) {
		/* pin 0, 1 belongs to port 5 */
		int_status[5] = cy8c9540a_read_int_status(5);
	}

	for (gpio = 0; gpio < INT_PIN_NUM; gpio++) {
		if (int_pin_array[gpio]) {
			/* this pin has handler attached 
			 * So it is possible to generate a interrupt */
			status = int_status[int_pin_array[gpio]->port];
			if (status & (1 << int_pin_array[gpio]->pin)) {
				/* Aha, we found the pin triggered the interrupt!
				 * Wake up the task waiting on this pin
				 * and stop checking */
#ifndef NO_GPIO_IOVCPU
				/* lock kernel before calling kernel function */
				cli();
				lock_kernel();
#endif
				wakeup_queue(&(int_pin_array[gpio]->waitqueue));
#ifndef NO_GPIO_IOVCPU
				unlock_kernel();
				sti();
#endif
				break;
			}
		}
	}
}

/* ************************************************************* */

int cypress_get_id()
{
  u8 dev_id = i2c_read_byte_data(REG_DEVID_STAT);
  return dev_id & 0xf0;
}

void cy8c9540a_test()
{
	//unsigned pwm = 1;
	//int fade_val;

#if 0
  unsigned gpio = 3;
  cy8c9540a_gpio_direction_output(gpio, 0);
  cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
  cy8c9540a_gpio_set_value(gpio, 1);
	tsc_delay_usec (3000000);
  for (i = 1; i <= 1000; i++) {
		cy8c9540a_gpio_set_value(gpio, 1);
		cy8c9540a_gpio_set_value(gpio, 0);
		cy8c9540a_gpio_set_value(gpio, 0);
	}
	cy8c9540a_pwm_enable(pwm);
	while(1) {
		for (fade_val = 0; fade_val <= 255; fade_val += 5) {
			cy8c9540a_pwm_config(pwm, fade_val, 255);
			tsc_delay_usec(30 * 1000);
		}
		for (fade_val = 255; fade_val >= 0; fade_val -= 5) {
			cy8c9540a_pwm_config(pwm, fade_val, 255);
			tsc_delay_usec(30 * 1000);
		}
	}
#endif

#if 0
	unsigned gpio = 16;
	unsigned mux = 15;
	/* route IO2 to GPIO6 */
	cy8c9540a_gpio_set_drive(mux, GPIOF_DRIVE_PULLUP);
	cy8c9540a_gpio_direction_output(mux, 0);
	/* route IO2 to cy8c9540a */
	cy8c9540a_gpio_set_drive(mux, GPIOF_DRIVE_PULLUP);
	cy8c9540a_gpio_direction_output(mux, 1);
	logger_printf("mux is 0x%x", cy8c9540a_gpio_get_value(mux));
	/* unmask IO2 */
	cy8c9540a_unmask_interrupt(gpio);
	/* enable GPIO6 as interrupt source */
	int i = 5;
	//for (i = 0; i < 8; i++) {
		quark_gpio_set_interrupt_type(i, EDGE);
		quark_gpio_set_interrupt_polarity(i, FALLING_EDGE);
		quark_gpio_interrupt_enable(i);
		quark_gpio_clear_interrupt(i);
	//}
	quark_gpio_registers();
#endif

	unsigned mux = 15, fast_gpio = 6;
	/* route IO2 to GPIO6 */
	cy8c9540a_gpio_set_drive(mux, GPIOF_DRIVE_PULLUP);
	cy8c9540a_gpio_direction_output(mux, 0);
	/* set GPIO6 direction and value */
	quark_gpio_direction(fast_gpio, 1);
	quark_gpio_high(fast_gpio);
	while(1);
}

bool cy8c9540a_setup()
{
	int ret = 0;
	int i = 0;
  int dev_id;

	/* enable i2c device */
	i2c_xfer_init(dev.addr);

  dev_id = cypress_get_id();
  logger_printf("dev_id is 0x%x\n", dev_id);

	/* Disable PWM, set all GPIOs as input.  */
	for (i = 0; i < NPORTS; i++) {
		ret = i2c_write_byte_data(REG_PORT_SELECT, i);
		if (ret < 0) {
			logger_printf("can't select port %u", i);
      return FALSE;
		}

		ret = i2c_write_byte_data(REG_SELECT_PWM, 0x00);
		if (ret < 0) {
			logger_printf("can't write to SELECT_PWM");
      return FALSE;
		}

		ret = i2c_write_byte_data(REG_PIN_DIR, 0xff);
		if (ret < 0) {
			logger_printf("can't write to PIN_DIR");
      return FALSE;
		}
	}

#if 0
	/* Cache the output registers */
	ret = i2c_read_block_data(REG_OUTPUT_PORT0,
            sizeof(dev.outreg_cache),
            dev.outreg_cache);
	if (ret < 0) {
    logger_printf("can't cache output registers");
    return ret;
	}
#endif

	/* Set default PWM clock source.  */
	for (i = 0; i < NPWM; i ++) {
		ret = i2c_write_byte_data(REG_PWM_SELECT, i);
		if (ret < 0) {
			logger_printf("can't select pwm %u", i);
      return ret;
		}

		ret = i2c_write_byte_data(REG_PWM_CLK, PWM_CLK);
		if (ret < 0) {
			logger_printf("can't write to REG_PWM_CLK");
      return ret;
		}
	}

#ifndef NO_GPIO_IOVCPU
	cy8c9540a_interrupt_pid = start_kernel_thread(
			(uint) cy8c9540a_interrupt_thread,
			(uint)&cy8c9540a_interrupt_stack[1023],
			"Cy8c9540a Interrupt Thread");
	lookup_TSS(cy8c9540a_interrupt_pid)->cpu = select_iovcpu(1<<5);
#endif

	//cy8c9540a_test();
	return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = cy8c9540a_setup
};

DEF_MODULE (galileo_cy8c9540a, "Galileo CY8C9540A driver", &mod_ops, {"galileo_i2c", "galileo_quark_gpio"});


