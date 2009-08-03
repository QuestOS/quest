#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_
#include "types.h"

#define KEYBOARD_BUFFER_SIZE 16

#define KEYBOARD_IRQ 1
#define KEYBOARD_VECTOR 0x21

#define KEYBOARD_DATA_PORT 0x60
#define KEYBOARD_STATUS_PORT 0x64

#endif
