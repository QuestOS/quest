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
#include "drivers/gpio/quark_gpio.h"
#include "util/printf.h"
#include "cy8c9540a.h"

#define DEBUG_CYPRESS

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

/* Galileo-specific data */
#define PWM_CLK				0x00	/* see resulting PWM_TCLK_NS */
/* XXX: i donno why 32kHz is 31250 */
#define PWM_TCLK_NS 			31250 /* 32kHz */

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
    DLOG("can't read input port%u", port);
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
    DLOG("can't read output port%u", port);
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
    DLOG("can't select port%u", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_DRIVE_PULLUP + offs);
  if (ret < 0) {
    DLOG("can't read drive mode port%u", port);
    return ret;
  }

  val = (u8)(ret | BIT(pin));

  ret = i2c_write_byte_data(REG_DRIVE_PULLUP + offs, val);
  if (ret < 0) {
    DLOG("can't write drive mode port%u", port);
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
    DLOG("can't select port%u", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_PIN_DIR);
  if (ret < 0) {
    DLOG("can't read pin direction", port);
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
    DLOG("can't write pin direction", port);
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

int cy8c9540a_pwm_config(unsigned pwm, int duty, int period)
{
	int ret;

	if (pwm > NPWM) {
		DLOG("invalid pwm number");
		return -1;
	}

	if (duty < 0 || period <= 0 || duty > period) {
		DLOG("invalid duty and period configuration");
		return -1;
	}
	
	if (period > PWM_MAX_PERIOD) {
		DLOG("period must be within [0-%d]ns",
				PWM_MAX_PERIOD * PWM_TCLK_NS);
		return -1;
	}

	ret = i2c_write_byte_data(REG_PWM_SELECT, (u8)pwm);
	if (ret < 0) {
		DLOG("can't write to REG_PWM_SELECT");
		return ret;
	}

	ret = i2c_write_byte_data(REG_PWM_PERIOD, (u8)period);
	if (ret < 0) {
		DLOG("can't write to REG_PWM_PERIOD");
		return ret;
	}

	ret = i2c_write_byte_data(REG_PWM_PULSE_W, (u8)duty);
	if (ret < 0) {
		DLOG("can't write to REG_PWM_PULSE_W");
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
		DLOG("invalid pwm number");
		return -1;
	}

	gpio = pwm2gpio_mapping[pwm];
	if (PWM_UNUSED == gpio) {
		DLOG("pwm%u is unused", pwm);
		return -1;
	}
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);

	val = i2c_read_byte_data(REG_SELECT_PWM);
	if (val < 0) {
		DLOG("can't read REG_SELECT_PWM");
		return val;
	}

	if (enable) {
		/* Set pin as output driving high */
		ret = cy8c9540a_gpio_direction_output(gpio, 1);
		if (ret < 0) {
			DLOG("can't set pwm%u as output", pwm);
			return ret;
		}
		val |= BIT((u8)pin);
	} else {
		val &= ~BIT((u8)pin);
	}

	ret = i2c_write_byte_data(REG_SELECT_PWM, val);
	if (ret < 0) {
		DLOG("can't write to REG_SELECT_PWM", pwm);
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

static int
cy8c9540a_unmask_interrupt(unsigned gpio)
{
	s32 ret = 0;
  u8 port = 0, pin = 0, val;
  port = cypress_get_port(gpio);
  pin = cypress_get_offs(gpio, port);

	ret = i2c_write_byte_data(REG_PORT_SELECT, port);
  if (ret < 0) {
    DLOG("can't select port%u", port);
    return ret;
  }

  ret = i2c_read_byte_data(REG_INTR_MASK);
  if (ret < 0) {
    DLOG("can't read interrupt mask port%u", port);
    return ret;
  }

  val = (u8)(ret & ~BIT(pin));
	DLOG("about to write 0x%x to REG_INTR_MASK", val);

  ret = i2c_write_byte_data(REG_INTR_MASK, val);
  if (ret < 0) {
    DLOG("can't write interrupt mask port%u", port);
    return ret;
  }

  return 0;
}

int cypress_get_id()
{
  u8 dev_id = i2c_read_byte_data(REG_DEVID_STAT);
  return dev_id & 0xf0;
}

int
enabled_int_line()
{
	int i;
	for (i = 0; i <= 7; i++) {
		quark_gpio_interrupt_enable(i);
		quark_gpio_set_interrupt_type(i, EDGE);
		quark_gpio_set_interrupt_polarity(i, FALLING_EDGE);
	}
	return 0;
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
	unsigned gpio = 16;
	unsigned mux = 15;
	/* route IO2 to cy8c9540a */ 
	cy8c9540a_gpio_direction_output(mux, 1);
	/* turn on led */
  cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
	cy8c9540a_gpio_direction_output(gpio, 1);
	/* delay 3s */
	tsc_delay_usec(3000000);
	/* reset cy8c9540a */
	//DLOG("resetting cy8c9540a...");
	//quark_gpio_direction(4, 1);
	//quark_gpio_high(4);
	/* delay 3s */
	//tsc_delay_usec(3000000);
	/* route IO2 to quark GPIO 6*/
	DLOG("routing to GPIO 6...");
	quark_gpio_direction(6, 1);
	quark_gpio_high(6);
	cy8c9540a_gpio_direction_output(mux, 0);
	/* turn on led */
  //cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
	//cy8c9540a_gpio_direction_output(gpio, 1);
	/* delay 3s */
	//tsc_delay_usec(3000000);
	tsc_delay_usec(3000000);

	DLOG("routing back to cy8c9540a...");
	/* route IO2 to cy8c9540a */ 
	cy8c9540a_gpio_direction_output(mux, 1);
	/* turn on led */
  cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
	cy8c9540a_gpio_direction_output(gpio, 1);
	tsc_delay_usec(3000000);

	//while(1);
	//u8 port = cypress_get_port(gpio);
	//cy8c9540a_unmask_interrupt(gpio);
	/* clear interrupt status */
	//i2c_read_byte_data(REG_INTR_STAT_PORT0 + port);
	//enabled_int_line();
	//int i;
	//DLOG("cypress setting");
  //cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
	//cy8c9540a_gpio_direction_output(gpio, 1);
	//tsc_delay_usec(2000000);
#if 0
	for (i = 7; i >= 0; i--) { 
		DLOG("setting %d", i);
		quark_gpio_direction(i, 1);
		quark_gpio_high(i);
		DLOG("port status is 0x%x", quark_gpio_read_port_status());
		tsc_delay_usec(2000000);
	}
#endif

}

bool cy8c9540a_setup()
{
	int ret = 0;
	int i = 0;
  int dev_id;
	//u8 eeprom_enable_seq[] = {0x43, 0x4D, 0x53, 0x2};

	/* enable i2c device */
	i2c_xfer_init(dev.addr);

  dev_id = cypress_get_id();
  DLOG("dev_id is 0x%x", dev_id);

	/* Disable PWM, set all GPIOs as input.  */
	for (i = 0; i < NPORTS; i++) {
		ret = i2c_write_byte_data(REG_PORT_SELECT, i);
		if (ret < 0) {
			DLOG("can't select port %u", i);
      return FALSE;
		}

		ret = i2c_write_byte_data(REG_SELECT_PWM, 0x00);
		if (ret < 0) {
			DLOG("can't write to SELECT_PWM");
      return FALSE;
		}

		ret = i2c_write_byte_data(REG_PIN_DIR, 0xff);
		if (ret < 0) {
			DLOG("can't write to PIN_DIR");
      return FALSE;
		}
	}

#if 0
	/* Cache the output registers */
	ret = i2c_read_block_data(REG_OUTPUT_PORT0,
            sizeof(dev.outreg_cache),
            dev.outreg_cache);
	if (ret < 0) {
    DLOG("can't cache output registers");
    return ret;
	}
#endif

	/* Set default PWM clock source.  */
	for (i = 0; i < NPWM; i ++) {
		ret = i2c_write_byte_data(REG_PWM_SELECT, i);
		if (ret < 0) {
			DLOG("can't select pwm %u", i);
      return ret;
		}

		/* XXX: actually there is no API implemented to set clock */
		ret = i2c_write_byte_data(REG_PWM_CLK, PWM_CLK);
		if (ret < 0) {
			DLOG("can't write to REG_PWM_CLK");
      return ret;
		}
	}

#if 0
	/* Enable the EEPROM */
	ret = i2c_write_block_data(REG_ENABLE,
					     sizeof(eeprom_enable_seq),
					     eeprom_enable_seq);
	if (ret < 0) {
		DLOG("can't enable EEPROM");
    return ret;
	}
#endif

	cy8c9540a_test();

	return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = cy8c9540a_setup
};

DEF_MODULE (galileo_cy8c9540a, "Galileo CY8C9540A driver", &mod_ops, {"galileo_i2c", "galileo_quark_gpio"});


