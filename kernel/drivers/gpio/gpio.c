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
#include "util/printf.h"

#define PIN_MODE  0
#define DIG_WRITE 1
#define DIG_READ  2

#define OUTPUT  0
#define INPUT   1
#define HIGH    1
#define LOW     0

#define DLOG(fmt,...) DLOG_PREFIX("SYSCALL",fmt,##__VA_ARGS__)

int gpio_handler(int operation, int gpio, int val, int arg)
{
	int ret;

	switch(operation) {
		case PIN_MODE:
			ret = cy8c9540a_gpio_set_drive(gpio, GPIOF_DRIVE_STRONG);
			if (ret < 0)
				return ret;
			if (val ==  OUTPUT)
				return cy8c9540a_gpio_direction_output(gpio, 0);
			else 
				return cy8c9540a_gpio_direction_input(gpio);
		case DIG_WRITE:
			cy8c9540a_gpio_set_value(gpio, val);
		case DIG_READ:
			return cy8c9540a_gpio_get_value(gpio);
		default:
			DLOG("Unsupported operation!");
			return -1;
	}

	return 0;
}

