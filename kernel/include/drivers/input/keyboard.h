#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_
#include "types.h"

#define KEYBOARD_BUFFER_SIZE 16

#define KEYBOARD_IRQ 1
#define KEYBOARD_VECTOR 0x21

#define KEYBOARD_DATA_PORT 0x60
#define KEYBOARD_STATUS_PORT 0x64

#define KEY_EVENT_MAX 5

typedef struct
{
  uint16 scancode;
  uint16 present:1;
  uint16 pressed:1;
  uint16 release:1;
  uint16 latest:1;
  uint16 reserved:12;
} key;

typedef struct
{
  key keys[KEY_EVENT_MAX];
} key_event;

void init_keyboard_8042 (void);
void keyboard_8042_next (key_event *);

#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
