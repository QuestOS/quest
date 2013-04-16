/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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
#include "arch/i386.h"
#include "sched/sched.h"
#include "smp/smp.h"
#include "smp/apic.h"
#include "util/circular.h"
#include "util/printf.h"
#ifdef USE_VMX
#include "vm/shm.h"
#include "vm/vmx.h"
#endif

/* Enable kernel debugging hotkeys */
//#define DEBUG_KEYS

/* Circular buffer storing incoming keyboard events. */
static circular keyb_buffer;
static key_event buffer_space[KEYBOARD_BUFFER_SIZE];

/* Current state of the keyboard. */
static key_event cur_event;

/* Did we just receive an escape scancode, and if so, which one?  Not
 * to be confused with ESC key. */
static uint8 escape;

/* Scancodes are normally 1 byte, except when the code E0 or E1 is
 * received: that is the first byte of a 2-byte scancode. */

static inline void
clean_entry (int i)
{
  cur_event.keys[i].scancode = 0;
  cur_event.keys[i].present  = 0;
  cur_event.keys[i].release  = 0;
  cur_event.keys[i].pressed  = 0;
  cur_event.keys[i].latest   = 0;
}

#ifdef DEBUG_KEYS
static inline void
check_debug_keys (bool have_kernel_lock)
{
  int i;
  for (i=0; i<KEY_EVENT_MAX; i++) {
    if (cur_event.keys[i].present && cur_event.keys[i].pressed) {
      if (cur_event.keys[i].scancode == 0x16) {
        /* pressed 'U' */
      }
    }
  }
}
#endif

static inline void
check_control_alt_del (void)
{
  bool ctrl, alt, del;
  int i;
  ctrl = alt = del = FALSE;

  for (i=0; i<KEY_EVENT_MAX; i++) {
    if (cur_event.keys[i].present && cur_event.keys[i].pressed) {
      switch (cur_event.keys[i].scancode) {
      case 0x001d:
      case 0xe01d:
        ctrl = TRUE; break;
      case 0x0038:
      case 0xe038:
        alt = TRUE; break;
      case 0x0053:
      case 0xe053:
        del = TRUE; break;
      }
    }
  }

  if (ctrl && alt && del) {
#if 0
//extern uint64 ipi_rt_start, ipi_rt_end;
//extern uint64 vm_time_start, vm_time_end;
    //int vec = 200;
    //RDTSC (ipi_rt_start);
    //LAPIC_send_ipi (0x1 << 1, LAPIC_ICR_LEVELASSERT | LAPIC_ICR_DM_LOGICAL | vec);
    //RDTSC (vm_time_start);
    //vm_exit (0);
    //RDTSC (vm_time_end);
    //com1_printf ("VM-Enter Time: 0x%llX\n", vm_time_end - vm_time_start);
    cli ();
    shm->network_transmit_enabled[0] = FALSE;
    shm->network_transmit_enabled[1] = TRUE;
    sti ();
    com1_printf ("Hot backup activated\n");
#else
    uint8 state;
    printf ("REBOOTING...\n");
    com1_printf ("REBOOTING...\n");
    while (((state = inb (KEYBOARD_STATUS_PORT)) & 2) != 0);
    outb (0xFE, KEYBOARD_STATUS_PORT);
    asm volatile ("hlt");
#endif
  }
}

void new_keyboard_code(uint8 code, bool have_kernel_lock)
{
  int i;
  if (escape) {
    /* Received 2-byte scancode. */
    code |= (escape << 8);
    escape = 0;
  }

    
  if (code == 0xE0 || code == 0xE1)
    escape = code;
  else if (code & 0x80) {
    /* Release key */
    code &= (~0x80);          /* unset "release" bit */

    /* If a key is released, there should be a Press event in the
     * cur_event buffer already. */
    for (i=0; i<KEY_EVENT_MAX; i++) {
      if (cur_event.keys[i].present == 1 &&
          cur_event.keys[i].scancode == code) {
        
        /* found previously inserted entry in event buffer: */
        cur_event.keys[i].release = 1;
        cur_event.keys[i].pressed = 0;
        cur_event.keys[i].latest  = 1;

        /* enqueue release event */
        if (circular_insert_nowait (&keyb_buffer, (void *)&cur_event) < 0) {
          if(!have_kernel_lock) lock_kernel();
          com1_printf ("keyboard_8042: dropped break code: %X\n", code);
          if(!have_kernel_lock) unlock_kernel();
        }
        
        clean_entry (i);      /* remove it from cur_event */
        
        break;
      }
    }

    /* No previous Press event found. */
    if (i == KEY_EVENT_MAX) {
      if(!have_kernel_lock) lock_kernel();
      com1_printf ("keyboard_8042: spurious break code: %X\n", code);
      if(!have_kernel_lock) unlock_kernel();
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

    static char lcase_scancode[128] =
    "\0\e1234567890-=\177\tqwertyuiop[]\n\0asdfghjkl;'`\0\\zxcvbnm,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";


    if (i == KEY_EVENT_MAX) {
      /* no free entry */
      if(!have_kernel_lock) lock_kernel();
      com1_printf ("keyboard_8042: too many keys: %X\n", code);
      if(!have_kernel_lock) unlock_kernel();

    }
    else {
      /* operate on entry: 0 <= i < KEY_EVENT_MAX */
      
      cur_event.keys[i].scancode = code;
      cur_event.keys[i].present  = 1;
      cur_event.keys[i].pressed  = 1;
      cur_event.keys[i].latest   = 1;
      
      /* enqueue press event */
      if (circular_insert_nowait (&keyb_buffer, (void *)&cur_event) < 0) {
        if(!have_kernel_lock) lock_kernel();
        com1_printf ("keyboard_8042: dropped make code: %X\n", code);
        if(!have_kernel_lock) unlock_kernel();
      }
      
      cur_event.keys[i].latest = 0;
    }
  }

  check_control_alt_del ();

#ifdef DEBUG_KEYS
  check_debug_keys (have_kernel_lock);
#endif

}

/* Process the keyboard IRQ. */
static uint32
kbd_irq_handler (uint8 vec)
{
  uint8 code;

#ifdef USE_VMX
  uint32 cpu;
  cpu = get_pcpu_id ();
  if (shm_initialized && shm_screen_initialized &&
      (shm->virtual_display.cur_screen != cpu)) {
    return 0;
  }
#endif

  code = inb (KEYBOARD_DATA_PORT);
  new_keyboard_code(code, FALSE);

  return 0;
}

/* Setup the circular buffer and the IRQ handler. */
bool
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
    set_vector_handler (KEYBOARD_IRQ + PIC1_BASE_IRQ, kbd_irq_handler);
  } else {
    IOAPIC_map_GSI (IRQ_to_GSI (mp_ISA_bus_id, KEYBOARD_IRQ),
                    KEYBOARD_VECTOR, 0xFF00000000000800LL);
    set_vector_handler (KEYBOARD_VECTOR, kbd_irq_handler);
  }
  return TRUE;
}

/* Retrieve the next key event or block. */
void
keyboard_8042_next (key_event *e)
{
  circular_remove (&keyb_buffer, (void *)e);
}

int
keyboard_8042_next_no_wait(key_event *e)
{
  return circular_remove_nowait(&keyb_buffer, (void *)e);
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = init_keyboard_8042
};

//DEF_MODULE (input___kb8042, "Keyboard (i8042) driver", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
