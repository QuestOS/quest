#ifndef __I2C__
#define __I2C__

#include "syscall.h"

enum {I2C_INIT, I2C_WRITE, I2C_READ};

static int
i2c_init(unsigned int slave_addr)
{
	return make_syscall(I2C_INIT, slave_addr, 0, 0);
}

static int
i2c_write_byte(unsigned char data)
{
	return make_i2c_syscall(I2C_WRITE, data, 0, 0);
}

#endif
