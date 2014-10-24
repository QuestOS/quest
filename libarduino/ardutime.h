#include <unistd.h>

void
delay(unsigned long ms)
{
    usleep(ms * 1000);
}
