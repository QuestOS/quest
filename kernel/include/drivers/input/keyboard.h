/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

bool init_keyboard_8042 (void);
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
