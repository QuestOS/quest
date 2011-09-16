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

#include "sched/vcpu.h"
#include "sched/sched.h"
#include "util/debug.h"
#include "util/printf.h"
#include "kernel.h"
#include "vm/shm.h"
#include "mem/physical.h"
#include "mem/virtual.h"

#define DEBUG_MSGT

#ifdef DEBUG_MSGT
#define DLOG(fmt,...) DLOG_PREFIX("Msg Test",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define MSG_HOST    1
#define MB_SIZE     64

static u32 msgt_stack[1024] ALIGNED (0x1000);
static task_id msgt_id = 0;
static u32 phys_channels[4];
static void * virt_channels[4];
static void * channel = NULL;
u8 * mailbox;

static void
msg_thread (void)
{
  int sandbox = 0;
  int i = 0, j = 0;
  u8 index = 0;

  sandbox = get_pcpu_id ();

  if (sandbox == MSG_HOST) {
    /* This is the sender's code */
    DLOG ("Msg Sender: Hello from 0x%x", msgt_id);

    for (i = 0; i < 4; i++) {
      virt_channels[i] = map_virtual_page (phys_channels[i] | 3);
    }

    for (;;) {
      for (i = 0; i < 4; i++) {
        for (j = 0; j < MB_SIZE; j++) {
          ((u8*)(virt_channels[i]))[j] = index;
        }
      }
      DLOG ("Message sent from Sandbox %d : %d", sandbox, index);
      print ("Sent message: ");
      putx (index);
      print ("\n");
      index++;
      /* 2 second interval */
      sched_usleep (2000000);
    }
  } else {
    /* This is the receiver's code */
    DLOG ("Msg Sender: Hello from 0x%x", msgt_id);
    DLOG ("Message Channel for Sandbox %d is 0x%X", sandbox, phys_channels[sandbox]);
    channel = map_virtual_page (phys_channels[sandbox] | 3);
    mailbox = (u8*) channel;
    index = mailbox[MB_SIZE - 1];

    for (;;) {
      while (index == mailbox[MB_SIZE - 1]) {
        //DLOG ("Sandbox %d: No messages in mailbox. Index is %d...", sandbox, index);
        sched_usleep (10000);
      }
      /* Got new message */
      index = mailbox[MB_SIZE - 1];
      if (index == 0xFF) {
        print ("Channel Corrupted!\n");
        continue;
      }
      DLOG ("Sandbox %d got message from Sandbox %d: %d", sandbox, MSG_HOST, index);
      print ("Got message: ");
      putx (index);
      print ("\n");
    }
  }
}

extern bool
msgt_init (void)
{
  int sandbox = 0;
  int i;
  sandbox = get_pcpu_id ();

  if (sandbox == 0) {
    for (i = 0; i < 4; i++) {
      phys_channels[i] = shm_alloc_phys_frame ();
      virt_channels[i] = map_virtual_page (phys_channels[i] | 3);
      memset ((void*) virt_channels[i], 0, 4096);
      unmap_virtual_page (virt_channels[i]);
      DLOG ("Message Channel for Sandbox %d with Physical Address: 0x%X", i, phys_channels[i]);
    }
  }

  msgt_id = start_kernel_thread ((u32) msg_thread, (u32) &msgt_stack[1023]);
  lookup_TSS (msgt_id)->cpu = 1;

  DLOG ("Communication Thread Created on Sandbox %d, Thread ID is: 0x%x...\n", sandbox, msgt_id);
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = msgt_init
};

//DEF_MODULE (msgt, "Message Passing Test", &mod_ops, {});

/* vi: set et sw=2 sts=2: */
