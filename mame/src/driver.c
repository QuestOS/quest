#include "driver.h"

extern struct GameDriver pacman_driver;
extern struct GameDriver pacmod_driver;

const struct GameDriver *drivers[] =
{
	&pacman_driver,
	&pacmod_driver,
	0	/* end of array */
};
