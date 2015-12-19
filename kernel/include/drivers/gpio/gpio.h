#include "kernel.h"

/* entry point to gpio driver from syscall */
struct gpio_ops {
	void (*set_value)(uint32, int);
	int (*get_value)(uint32);
	int (*set_drive)(uint32, uint32);
	int (*set_output)(uint32, uint32);
	int (*set_input)(uint32);
};
