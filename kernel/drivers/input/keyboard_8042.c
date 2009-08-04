#include "drivers/input/keyboard.h"
#include "arch/i386.h"
#include "sched/sched.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/circular.h"
#include "util/printf.h"

static circular keyb_buffer;
static key_event buffer_space[KEYBOARD_BUFFER_SIZE];

static key_event cur_event;
static uint8 escape;

static inline void
clean_entry (int i)
{
  cur_event.keys[i].scancode = 0;
  cur_event.keys[i].present  = 0;          
  cur_event.keys[i].release  = 0;
  cur_event.keys[i].pressed  = 0;
  cur_event.keys[i].latest   = 0;
}

static uint32
kbd_irq_handler (uint8 vec)
{
  uint8 code;
  int i;

  code = inb (KEYBOARD_DATA_PORT);

  if (escape) {
    code |= (escape << 8);
    escape = 0;
  }

  if (code == 0xE0 || code == 0xE1) 
    escape = code;
  else if (code & 0x80) {
    /* Release key */

    code &= (~0x80);          /* unset "release" bit */
    for (i=0; i<KEY_EVENT_MAX; i++) {
      if (cur_event.keys[i].present == 1 &&
          cur_event.keys[i].scancode == code) {

        /* found previously inserted entry in event buffer: */
        cur_event.keys[i].release = 1;
        cur_event.keys[i].pressed = 0;
        cur_event.keys[i].latest  = 1;

        /* enqueue release event */
        if (circular_insert_nowait (&keyb_buffer, (void *)&cur_event) < 0) {
          lock_kernel();
          com1_printf ("keyboard_8042: dropped break code: %X\n", code);
          unlock_kernel();
        }
          
        clean_entry (i);      /* remove it from cur_event */

        break;
      }
    }

    if (i == KEY_EVENT_MAX) {
      lock_kernel();
      com1_printf ("keyboard_8042: spurious break code: %X\n", code);
      unlock_kernel();
    }

  } else {
    /* Press key */

    for (i=0; i<KEY_EVENT_MAX; i++) {
      /* First, see if it is already in the table: e.g. repeating
       * keystrokes will send a stream of 'make' codes with no 'break'
       * codes. */
      if (cur_event.keys[i].present == 1 &&
          cur_event.keys[i].scancode == code) {
        break;
      }
    }

    if (i == KEY_EVENT_MAX) {
      /* It wasn't in the table already, so, find an empty entry */
      for (i=0; i<KEY_EVENT_MAX; i++) {
        if (cur_event.keys[i].present == 0) {
          break;
        }
      }
    }
      
    if (i == KEY_EVENT_MAX) {
      /* no free entry */
      lock_kernel();
      com1_printf ("keyboard_8042: too many keys: %X\n", code);
      unlock_kernel();

    } else {
      /* operate on entry: 0 <= i < KEY_EVENT_MAX */

      cur_event.keys[i].scancode = code;
      cur_event.keys[i].present  = 1;
      cur_event.keys[i].pressed  = 1;
      cur_event.keys[i].latest   = 1;

      /* enqueue press event */
      if (circular_insert_nowait (&keyb_buffer, (void *)&cur_event) < 0) {
        lock_kernel();
        com1_printf ("keyboard_8042: dropped make code: %X\n", code);
        unlock_kernel();
      }
          
      cur_event.keys[i].latest = 0;
    }
  }
 
  return 0;
}

void
init_keyboard_8042 (void)
{
  int i;

  escape = 0;

  for (i=0; i<KEY_EVENT_MAX; i++)
    clean_entry (i);

  circular_init (&keyb_buffer, 
                 (void *)buffer_space, 
                 KEYBOARD_BUFFER_SIZE, 
                 sizeof (key_event));

  if (mp_ISA_PC) {
    set_vector_handler (KEYBOARD_IRQ, kbd_irq_handler);
  } else {
    IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, KEYBOARD_IRQ),
                    KEYBOARD_VECTOR, 0xFF00000000000800LL);
    set_vector_handler (KEYBOARD_VECTOR, kbd_irq_handler);
  }
}

void
keyboard_8042_next (key_event *e)
{
  circular_remove (&keyb_buffer, (void *)e);
}
