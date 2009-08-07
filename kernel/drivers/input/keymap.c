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
