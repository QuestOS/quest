#include "gpio.h"
#include <unistd.h>

int main()
{
	gpio_mode(10, 0);
	while(1) {
		gpio_write(10, 1);
		usleep(1000000);
		gpio_write(10, 0);
		usleep(1000000);
	}

	return 0;
}
