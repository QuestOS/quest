#include "drivers/input/keyboard.h"


/* US Keyboard map */
static char lcase_scancode[128] =
    "\0\e1234567890-=\177\tqwertyuiop[]\n\0asdfghjkl;'`\0\\zxcvbnm,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
static char ucase_scancode[128] =
    "\0\e1234567890-=\177\tQWERTYUIOP[]\n\0ASDFGHJKL;'`\0\\ZXCVBNM,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

uint8
keymap_getchar (void)
{
  int i;
  key_event e;
  bool shifted;
  uint8 char_code = 0;

  while (!char_code) {
    keyboard_8042_next (&e);

    shifted = FALSE;  
    for (i=0; i<KEY_EVENT_MAX; i++) {
      if (e.keys[i].present) {
        if ((e.keys[i].scancode == 0x2A || e.keys[i].scancode == 0x36) &&
            e.keys[i].pressed)
          shifted = TRUE;
        else if (e.keys[i].scancode < 128 && e.keys[i].latest && e.keys[i].pressed) 
          char_code = e.keys[i].scancode;
      }
    }
  }
  
  if (shifted)
    return ucase_scancode[char_code];
  else
    return lcase_scancode[char_code];
}
