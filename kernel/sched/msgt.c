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

#define MSG_HOST    0
#define RECV_HOST   1
#define MB_SIZE     64 /* In bytes */
#define SND_RATE    50000 /* micro-seconds */

static u32 msgt_stack[1024] ALIGNED (0x1000);
static u32 ipc_stack[1024] ALIGNED (0x1000);
static u32 stat_stack[1024] ALIGNED (0x1000);
static task_id msgt_id = 0;
static u32 phys_channels[4];
static void * virt_channels[4];
static void * channel = NULL;
static int recv_rate[4] = {100000, 1, 800000, 1000000};
u8 * mailbox;
u32 msgt_stat_phy_page = 0;

struct msgt_stat_report {
  unsigned int canny_frame_count;
  unsigned int msg_sent;
  unsigned int msg_recv;
  u64 counter1;
  u64 tsc;
};

struct msgt_stat_report * msgt_report = NULL;

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

/* Never co-run with msg_thread, shared stack */
static void
msg_bandwidth_thread (void)
{
  int sandbox = 0, i, index = 1;
  sandbox = get_pcpu_id ();
  volatile uint32 * mailbox = NULL;

  if (!msgt_report)
    msgt_report = (struct msgt_stat_report *) map_virtual_page (msgt_stat_phy_page | 3);

  if (sandbox == MSG_HOST) {
    /* Sender code */
    for (i = 0; i < 4; i++) {
      virt_channels[i] = map_virtual_page (phys_channels[i] | 3);
    }
    mailbox = (uint32 *) virt_channels[MSG_HOST];

    unlock_kernel ();
    sti ();
    for (;;) {
      for (i = 0; i < 128; i++) {
        mailbox[i] = index;
      }
      msgt_report->msg_sent += 1; /* 512B */
      index++;
    }
  } else if (sandbox == RECV_HOST) {
    /* Receiver code */
    for (i = 0; i < 4; i++) {
      virt_channels[i] = map_virtual_page (phys_channels[i] | 3);
    }
    mailbox = (uint32 *) virt_channels[RECV_HOST];

    unlock_kernel ();
    sti ();
    for (;;) {
      for (i = 0; i < 128; i++) {
        mailbox[i] = index;
      }
      msgt_report->msg_recv += 1; /* 512B */
      index++;
    }
  }

  return;
}

static void
hog_thread (void)
{
  int sandbox = 0;
  sandbox = get_pcpu_id ();
  logger_printf ("Hog started in sandbox %d\n", sandbox);

  unlock_kernel ();
  sti ();
  for (;;) {
  }
}

#define RECV_SB           1
#define META_PAGE         3
#define IPC_SB_SIZE       512
#define NUM_ITERATIONS    8000L

static void
ipc_send_thread (void)
{
  uint64 start = 0, end = 0, cost = 0, worst = (0x0L), best = ~(0x0L);
  uint64 local_worst = 0L;
  u64 shift_start = 0, shift_end = 0;
  int i = 0, j = 0, m = 0;
  uint32 index = 0;
  void * channel = NULL;
  void * data = NULL;
  volatile uint32 * meta = NULL;
  volatile uint32 * mailbox = NULL;
  int cpu;
  int count = 1;
  int multiplier = 1;

  cpu = get_pcpu_id ();
  logger_printf ("Sandbox %d start sending...\n", cpu);

  channel = map_virtual_page (phys_channels[RECV_SB] | 3);
  data = map_virtual_page (phys_channels[META_PAGE] | 3);
  mailbox = (uint32*) channel;
  meta = (uint32*) data;
  meta[0] = 16; /* Start from 16 * 4 = 64 bytes */
  meta[1] = 0;

  sched_usleep (5054321);
  //check_copied_threads ();

  unlock_kernel ();
  sti ();
  for (i = 1; i < 14; i++) {
    cost = 0;
    count = ((meta[0] >> 9) == 0) ? 1 : (meta[0] >> 9);
    if (count == 1)
      multiplier = 1;
    else
      multiplier = 1;
    for (m = 0; m < NUM_ITERATIONS * multiplier; m++) {
      //asm volatile ("wbinvd");
      cli ();
      tsc_delay_usec (20000);
      RDTSC (start);
      sti ();
      for (j = 0; j < count; j++) {
        index++;
        if (meta[0] <= IPC_SB_SIZE) {
          memset ((void *) mailbox, index, meta[0] * 4);
        } else {
          memset ((void *) mailbox, index, IPC_SB_SIZE * 4);
        }
        meta[1] = index;
#if 0
        cli ();
        lock_kernel ();
        if (recv_id) wakeup (recv_id);
        sched_usleep (100000000);
        unlock_kernel ();
        sti ();
#else
        while (index == meta[1]);
#endif
        index = meta[1];
      }
      RDTSC (end);
      cost += (end - start);
      if ((end - start) > worst) worst = end - start;
      if ((end - start) < best) best = end - start;
      if ((end - start) > local_worst) local_worst = end - start;

      RDTSC (shift_end);
      if ((shift_end - shift_start) > (16459895000L)) {
        cli ();
        lock_kernel ();
        sched_usleep (112000);
        unlock_kernel ();
        sti ();
        RDTSC (shift_start);
        logger_printf ("Shifted\n");
        logger_printf ("Current Worst: 0x%llX\n", worst);
        logger_printf ("Local Worst: 0x%llX\n", local_worst);
        local_worst = 0;
      } else {
        cli ();
        lock_kernel ();
        sched_usleep (200000);
        unlock_kernel ();
        sti ();     
      }
      if ((m % (NUM_ITERATIONS >> 2)) == 0) logger_printf ("%d Mark!\n", NUM_ITERATIONS >> 2);
    }
    logger_printf ("Cycles: 0x%llX    0x%llX      Message Size: %d    Count: %d   Mult: %d\n",
                   best, worst, 4 * meta[0], count, multiplier);
    meta[0] = meta[0] * 2;
    worst = (0x0L);
    best = ~(0x0L);
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
  volatile uint32 * meta = NULL;
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
    if (meta[0] <= IPC_SB_SIZE) {
      memset ((void *) mailbox, index, meta[0] * 4);
    } else {
      memset ((void *) mailbox, index, IPC_SB_SIZE * 4);
    }
    meta[1] = index;
  }
}

static void
statistics_thread (void)
{
  int cpu = 0, prev_stat = 0, diff = 0, i = 0;
  int prev_sent = 0, diff_sent = 0, prev_recv = 0, diff_recv = 0;
  u64 prev_tsc = 0, diff_tsc = 0;
  u64 prev_c1 = 0, diff_c1 = 0;
  struct msgt_stat_report * report = NULL;

  if (!msgt_stat_phy_page) {
    logger_printf ("stat thread: msgt memory is not initialised\n");
    return;
  }
  report = (struct msgt_stat_report *) map_virtual_page (msgt_stat_phy_page | 3);

  cpu = get_pcpu_id ();
  logger_printf ("Statistics thread started in sandbox %d\n", cpu);

  unlock_kernel ();
  sti ();
  for (;;) {
    if (prev_stat == 0) {
      prev_stat = report->canny_frame_count;
    } else {
      diff = report->canny_frame_count - prev_stat;
      prev_stat = report->canny_frame_count;
    }
    if (prev_sent == 0) {
      prev_sent = report->msg_sent;
    } else {
      diff_sent = report->msg_sent - prev_sent;
      prev_sent = report->msg_sent;
    }
    if (prev_recv == 0) {
      prev_recv = report->msg_recv;
    } else {
      diff_recv = report->msg_recv - prev_recv;
      prev_recv = report->msg_recv;
    }
    if (prev_tsc == 0) {
      prev_tsc = report->tsc;
    } else {
      diff_tsc = report->tsc - prev_tsc;
      prev_tsc = report->tsc;
    }
    if (prev_c1 == 0) {
      prev_c1 = report->counter1;
    } else {
      diff_c1 = report->counter1 - prev_c1;
      prev_c1 = report->counter1;
    }
    if (prev_stat || prev_sent || prev_recv || prev_tsc || prev_c1)
      logger_printf ("Time: %d FPS: %d IPC 1: %d IPC 2: %d MIG: 0x%llX Overrun: 0x%llX\n",
                     i, diff, diff_sent >> 1, diff_recv >> 1, diff_tsc, diff_c1);
    i++;
    tsc_delay_usec (1000000);
  }
}

extern bool
stat_thread_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif

  task_id stat_id = 0;

  stat_id = start_kernel_thread ((u32) statistics_thread, (u32) &stat_stack[1023], "Stats");
  lookup_TSS (stat_id)->cpu = 1;

  DLOG ("Statistics Thread Created on Sandbox %d, Thread ID is: 0x%x...\n", sandbox, stat_id);
  return TRUE;
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
msgt_bandwidth_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif

  int vcpu_id = create_main_vcpu(20, 100, NULL);
  if(vcpu_id < 0) {
    DLOG("Failed to create ipc bandwidth vcpu");
    return FALSE;
  }

  create_kernel_thread_vcpu_args ((u32) msg_bandwidth_thread, (u32) &msgt_stack[1023],
                                  "msg bandwidth", vcpu_id, TRUE, 0);

  DLOG ("IPC Bandwidth Thread Created on Sandbox %d...\n", sandbox);
  return TRUE;
}

extern bool
hog_thread_init (void)
{
  int vcpu_id = create_main_vcpu(20, 100, NULL);
  if(vcpu_id < 0) {
    DLOG("Failed to create ipc hog vcpu");
    return FALSE;
  }
  
  create_kernel_thread_vcpu_args ((u32) hog_thread, (u32) &msgt_stack[1023],
                                  "Hog thread", vcpu_id, TRUE, 0);

  return TRUE;
}

extern bool
ipc_send_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif

  int vcpu_id = create_main_vcpu(20, 100, NULL);
  if(vcpu_id < 0) {
    DLOG("Failed to create sender vcpu");
    return FALSE;
  }

  create_kernel_thread_vcpu_args ((u32) ipc_send_thread, (u32) &ipc_stack[1023],
                                  "IPC send", vcpu_id, TRUE, 0);

  DLOG ("Sandbox %d: Sender created", sandboxd);

  return TRUE;
}

extern bool
ipc_recv_init (void)
{
#ifdef DEBUG_MSGT
  int sandbox = 0;
  sandbox = get_pcpu_id ();
#endif

  int vcpu_id = create_main_vcpu(20, 100, NULL);
  if(vcpu_id < 0) {
    DLOG("Failed to create ipc receiver vcpu");
    return FALSE;
  }

  create_kernel_thread_vcpu_args ((u32) ipc_recv_thread, (u32) &ipc_stack[1023],
                                  "IPC recv", vcpu_id, TRUE, 0);

  DLOG ("Sandbox %d: Receiver created", sandbox);

  return TRUE;
}

extern bool
msgt_mem_init (void)
{
  int i;
  void * stat_page = NULL;

  for (i = 0; i < 4; i++) {
    phys_channels[i] = shm_alloc_phys_frame ();
    virt_channels[i] = map_virtual_page (phys_channels[i] | 0x3);
    memset ((void*) virt_channels[i], 0, 4096);
    unmap_virtual_page (virt_channels[i]);
    DLOG ("Message Channel for Sandbox %d with Physical Address: 0x%X", i, phys_channels[i]);
  }

  msgt_stat_phy_page = shm_alloc_phys_frame ();
  stat_page = map_virtual_page (msgt_stat_phy_page | 3);
  memset ((void*) stat_page, 0, 4096);
  unmap_virtual_page (stat_page);

  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = msgt_init
};

/* vi: set et sw=2 sts=2: */
