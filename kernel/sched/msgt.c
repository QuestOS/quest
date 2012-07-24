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

#include "sched/vcpu.h"
#include "sched/sched.h"
#include "util/debug.h"
#include "util/printf.h"
#include "kernel.h"
#include "vm/shm.h"
#include "mem/physical.h"
#include "mem/virtual.h"

//#define DEBUG_MSGT

#ifdef DEBUG_MSGT
#define DLOG(fmt,...) DLOG_PREFIX("Msg Test",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

#define MSG_HOST    1
#define MB_SIZE     64 /* In bytes */
#define SND_RATE    50000 /* micro-seconds */

static u32 msgt_stack[1024] ALIGNED (0x1000);
static u32 ipc_stack[1024] ALIGNED (0x1000);
static task_id msgt_id = 0;
static u32 phys_channels[4];
static void * virt_channels[4];
static void * channel = NULL;
static int recv_rate[4] = {100000, 1, 800000, 1000000};
u8 * mailbox;

static uint64 logs[4][50];

static void
msg_thread (void)
{
  int sandbox = 0;
  int i = 0, j = 0;
  u8 index = 0;
  int count = 0;
  uint64 tsc_tmp = 0;

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
      sched_usleep (SND_RATE);
    }
  } else {
    /* This is the receiver's code */
    DLOG ("Msg Receiver: Hello from 0x%x", msgt_id);
    DLOG ("Message Channel for Sandbox %d is 0x%X", sandbox, phys_channels[sandbox]);
    channel = map_virtual_page (phys_channels[sandbox] | 3);
    mailbox = (u8*) channel;
    index = mailbox[MB_SIZE - 1];

    for (;;) {
      while (index == mailbox[MB_SIZE - 1]) {
        //DLOG ("Sandbox %d: No messages in mailbox. Index is %d...", sandbox, index);
        sched_usleep (recv_rate[sandbox]);
      }
      /* Got new message */
      index = mailbox[MB_SIZE - 1];
      if (index == 0xFF) {
        print ("Channel Corrupted!\n");
        continue;
      }
      if (count < 50) {
        RDTSC (tsc_tmp);
        if (count == 0)
          logs[sandbox][count] = tsc_tmp;
        else
          logs[sandbox][count] = tsc_tmp - logs[sandbox][0];
        count++;
      } else {
        logs[sandbox][0] = 0;
        for (count = 0; count < 50; count++)
          com1_printf ("Sandbox %d.%d: %llX\n", sandbox, count,
                       logs[sandbox][count]);
        com1_printf ("Sandbox %d: Msgt done!\n", sandbox);
        exit_kernel_thread ();
      }
      DLOG ("Sandbox %d got message from Sandbox %d: %d", sandbox, MSG_HOST, index);
      //print ("Got message: ");
      //putx (index);
      //print ("\n");
    }
  }
}

#define RECV_SB           1
#define META_PAGE         3
#define IPC_SB_SIZE       1024
#define NUM_ITERATIONS    5000

static void
ipc_send_thread (void)
{
  uint64 start = 0, end = 0;
  int i = 0, j = 0, k = 0, m = 0;
  uint32 index = 0;
  void * channel = NULL;
  void * data = NULL;
  uint32 * meta = NULL;
  volatile uint32 * mailbox = NULL;
  int cpu;
  int count = 1;

  cpu = get_pcpu_id ();
  logger_printf ("Sandbox %d start sending...\n", cpu);

  channel = map_virtual_page (phys_channels[RECV_SB] | 3);
  data = map_virtual_page (phys_channels[META_PAGE] | 3);
  mailbox = (uint32*) channel;
  meta = (uint32*) data;
  meta[0] = 16; /* Start from 16 * 4 = 64 bytes */

  sched_usleep (2000000);

  unlock_kernel ();
  sti ();
  for (i = 1; i < 15; i++) {
    RDTSC (start);
    for (m = 0; m < NUM_ITERATIONS * ((count == 1) ? 100 : 1); m++) {
      for (j = 0; j < count; j++) {
        index++;
        for (k = 0; k < meta[0] && k < IPC_SB_SIZE; k++) {
          mailbox[k] = index;
        }
        meta[1] = index;
        while (index == meta[1]);
        index = meta[1];
      }
    }
    RDTSC (end);
    logger_printf ("Cycles: 0x%llX      Message Size: %d Count: %d\n",
                   end - start, 4 * meta[0], count);
    meta[0] = meta[0] * 2;
    count = ((meta[0] >> 10) == 0) ? 1 : (meta[0] >> 10);
  }

  logger_printf ("Sender Done!\n");
  exit_kernel_thread ();
}

static void
ipc_recv_thread (void)
{
  void * channel = NULL;
  volatile uint32 * mailbox = NULL;
  void * data = NULL;
  uint32 * meta = NULL;
  int k = 0;
  uint32 index = 0;
  int cpu;

  cpu = get_pcpu_id ();
  logger_printf ("Sandbox %d start receiving...\n", cpu);
  channel = map_virtual_page (phys_channels[RECV_SB] | 3);
  data = map_virtual_page (phys_channels[META_PAGE] | 3);
  mailbox = (uint32*) channel;
  meta = (uint32*) data;

  unlock_kernel ();
  sti ();
  for (;;) {
    while (index == meta[1]);
    index = meta[1] + 1;

    for (k = 0; k < meta[0] && k < IPC_SB_SIZE; k++) {
      mailbox[k] = index;
    }
    meta[1] = index;
    //asm volatile ("wbinvd");
  }
}

extern bool
msgt_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif

  msgt_id = start_kernel_thread ((u32) msg_thread, (u32) &msgt_stack[1023], "MSGT init");
  lookup_TSS (msgt_id)->cpu = 1;

  DLOG ("Communication Thread Created on Sandbox %d, Thread ID is: 0x%x...\n", sandbox, msgt_id);
  return TRUE;
}

extern bool
ipc_send_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif
  task_id ipc_id = 0;

  ipc_id = start_kernel_thread ((u32) ipc_send_thread, (u32) &ipc_stack[1023], "IPC send");
  lookup_TSS (ipc_id)->cpu = 1;

  DLOG ("Sandbox %d: Sender 0x%x created", sandbox, ipc_id);

  return TRUE;
}

extern bool
ipc_recv_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif
  task_id ipc_id = 0;

  ipc_id = start_kernel_thread ((u32) ipc_recv_thread, (u32) &ipc_stack[1023], "IPC recv");
  lookup_TSS (ipc_id)->cpu = 1;

  DLOG ("Sandbox %d: Receiver 0x%x created", sandbox, ipc_id);

  return TRUE;
}

extern bool
msgt_mem_init (void)
{
  int i;

  for (i = 0; i < 4; i++) {
    phys_channels[i] = shm_alloc_phys_frame ();
    virt_channels[i] = map_virtual_page (phys_channels[i] | 3);
    memset ((void*) virt_channels[i], 0, 4096);
    unmap_virtual_page (virt_channels[i]);
    DLOG ("Message Channel for Sandbox %d with Physical Address: 0x%X", i, phys_channels[i]);
  }

  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = msgt_init
};

/* vi: set et sw=2 sts=2: */
