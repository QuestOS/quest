#include "i2c.h"
#include <unistd.h>

int main()
{
	i2c_init(0x8);
	while(1) {
		i2c_write_byte(0x38);
		usleep(100000);
	}

	return 0;
}
