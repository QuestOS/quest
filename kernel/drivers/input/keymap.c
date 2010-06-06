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

#include "drivers/input/keyboard.h"


/* US Keyboard map */
static char lcase_scancode[128] =
    "\0\e1234567890-=\177\tqwertyuiop[]\n\0asdfghjkl;'`\0\\zxcvbnm,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
static char ucase_scancode[128] =
    "\0\e1234567890-=\177\tQWERTYUIOP[]\n\0ASDFGHJKL;'`\0\\ZXCVBNM,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

/* Retrieve the next keyboard event and translate it into an ASCII
 * char, or block. */
uint8
keymap_getchar (void)
{
  int i;
  key_event e;
  bool shiftmod, ctrlmod, altmod;
  uint8 char_code = 0;

  while (!char_code) {
    keyboard_8042_next (&e);

    shiftmod = ctrlmod = altmod = FALSE;  
    for (i=0; i<KEY_EVENT_MAX; i++) {
      if (e.keys[i].present && e.keys[i].pressed) {
        switch (e.keys[i].scancode) {
        case 0x2A:              /* LSHIFT */
        case 0x36:              /* RSHIFT */
          shiftmod = TRUE; 
          break;

        case 0x1D:              /* LCTRL */
        case 0xE01D:            /* RCTRL */
          ctrlmod = TRUE;
          break;

        case 0x38:              /* LALT */
        case 0xE038:            /* RALT */
          altmod = TRUE;
          break;

        default:
          if(e.keys[i].scancode < 128 && e.keys[i].latest) 
            char_code = e.keys[i].scancode;
          break;
        }
      }
    }
  }
  
  if (shiftmod)
    return ucase_scancode[char_code];
  else
    return lcase_scancode[char_code];
}

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
