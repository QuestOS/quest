#define GPIOF_DRIVE_PULLUP	(1 << 6)
#define GPIOF_DRIVE_PULLDOWN	(1 << 7)
#define GPIOF_DRIVE_STRONG	(1 << 8)
#define GPIOF_DRIVE_HIZ		(1 << 9)
#define PERIOD				191

int cy8c9540a_gpio_get_value(unsigned gpio);
void cy8c9540a_gpio_set_value(unsigned gpio, int val);
int cy8c9540a_gpio_set_drive(unsigned gpio, unsigned mode);
int cy8c9540a_gpio_direction_output(unsigned gpio, int val);
int cy8c9540a_gpio_direction_input(unsigned gpio);
int cy8c9540a_pwm_config(unsigned pwm, int duty_ns, int period_ns);
int cy8c9540a_pwm_switch(unsigned pwm, int enable);
int cy8c9540a_pwm_enable(unsigned pwm);
int cy8c9540a_pwm_disable(unsigned pwm);
void cy8c9540a_register_interrupt(unsigned gpio, int mode, int T);
void cy8c9540a_wait_interrupt(unsigned gpio);
int cy8c9540a_fast_gpio_mux(unsigned gpio);
