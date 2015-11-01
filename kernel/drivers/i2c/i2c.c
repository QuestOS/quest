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

#include "util/printf.h"
#include "drivers/i2c/minnowmax_i2c.h"


#define DEBUG_I2C_SYSCALL

#ifdef DEBUG_I2C_SYSCALL
#define DLOG(fmt,...) DLOG_PREFIX("I2C SYSCALL",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

enum ops {I2C_INIT, I2C_WRITE, I2C_READ};

int
i2c_handler(enum ops operation, int arg1, int arg2, int arg3)
{
  DLOG("op: %u, arg1: %u, arg2: %u, arg3: %u",
      operation, arg1, arg2, arg3);

	switch(operation) {
    case I2C_INIT: {
      int slave_addr = arg1;
      i2c_xfer_init(slave_addr);
    }
		case I2C_WRITE: {
      unsigned char data = arg1; 
      return i2c_write_byte_data(data);
    }
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
