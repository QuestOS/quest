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
#include "smp/semaphore.h"

#define DEBUG_IPC

#ifdef DEBUG_IPC
#define DLOG(fmt,...) DLOG_PREFIX("IPC",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

inline int
fast_send_m (task_id dst_id, u32 arg1, u32 arg2)
{
  quest_tss *src = lookup_TSS (str ());
  quest_tss *dst = lookup_TSS (dst_id);

  semaphore_wait (&dst->Msem, 1, -1);
  wakeup (str ());
  dst->M[0] = arg1;
  dst->M[1] = arg2;
  ltr (dst_id);
  asm volatile ("call _sw_ipc"
                :
                :"S" (src), "D" (dst)
                :"eax", "ebx", "ecx", "edx");
  return 0;
}

inline int
fast_call_m (task_id dst_id, u32 arg1, u32 arg2, u32 *r_arg1, u32 *r_arg2)
{
  quest_tss *src = lookup_TSS (str ());
  quest_tss *dst = lookup_TSS (dst_id);

  semaphore_wait (&dst->Msem, 1, -1);
  dst->M[0] = arg1;
  dst->M[1] = arg2;
  semaphore_signal (&src->Msem, 1);
  ltr (dst_id);
  asm volatile ("call _sw_ipc"
                :"+S" (src), "+D" (dst)
                :
                :"eax", "ebx", "ecx", "edx", "cc", "memory");
  /* after _sw_ipc dst and src are swapped */
  *r_arg1 = dst->M[0];
  *r_arg2 = dst->M[1];
  return 0;
}

inline int
fast_recv_m (u32 *arg1, u32 *arg2)
{
  quest_tss *cur;
  semaphore_signal (&lookup_TSS (str ())->Msem, 1);
  asm volatile ("call *%1"
                :"=D" (cur)
                :"m" (schedule)
                :"eax", "ebx", "ecx", "edx", "esi", "cc", "memory");
  *arg1 = cur->M[0];
  *arg2 = cur->M[1];
  return 0;
}

inline int
fast_sendrecv_m (task_id dst_id, u32 arg1, u32 arg2, u32 *r_arg1, u32 *r_arg2)
{
  quest_tss *src = lookup_TSS (str ());
  quest_tss *dst = lookup_TSS (dst_id);

  semaphore_wait (&dst->Msem, 1, -1);
  dst->M[0] = arg1;
  dst->M[1] = arg2;
  semaphore_signal (&src->Msem, 1);
  ltr (dst_id);
  asm volatile ("call _sw_ipc"
                :"+S" (src), "+D" (dst)
                :
                :"eax", "ebx", "ecx", "edx", "cc", "memory");
  /* after _sw_ipc dst and src are swapped */
  *r_arg1 = dst->M[0];
  *r_arg2 = dst->M[1];
  return 0;
}

/* ************************************************** */

static u32 testS_stack[1024] ALIGNED (0x1000);
static u32 testR_stack[1024] ALIGNED (0x1000);

static task_id testR_id = 0, testS_id = 0;
static u64 start, finish;
extern u32 tsc_freq_msec;

static void
testS (void)
{
  u32 arg1, arg2;
  DLOG ("testS: hello from 0x%x", testS_id);
  for (;;) {
    RDTSC (start);
    fast_call_m (testR_id, 0xCAFEBABE, 0xDEADBEEF, &arg1, &arg2);
    RDTSC (finish);
    DLOG ("testS: MEM: cycles=0x%.08llX: answer was 0x%X and 0x%X", finish - start, arg1, arg2);
    sched_usleep (1000000);
  }
}

static void
testR (void)
{
  u32 arg1, arg2;
  DLOG ("testR: hello from 0x%x", testR_id);
  fast_recv_m (&arg1, &arg2);
  for (;;) {
    RDTSC (finish);
    DLOG ("testR: MEM: cycles=0x%.08llX: got 0x%X and 0x%X", finish - start, arg1, arg2);
    RDTSC (start);
    fast_sendrecv_m (testS_id, arg1 + arg2, arg1 | arg2, &arg1, &arg2);
  }
}

extern bool
ipc_init (void)
{
  DLOG ("hello");
  testS_id = start_kernel_thread ((u32) testS, (u32) &testS_stack[1023], "IPC TestS");
  testR_id = start_kernel_thread ((u32) testR, (u32) &testR_stack[1023], "IPC TestR");
  lookup_TSS (testS_id)->cpu = 0;
  lookup_TSS (testR_id)->cpu = 0;
  return TRUE;
}

#include "module/header.h"

static const struct module_ops mod_ops = {
  .init = ipc_init
};

//DEF_MODULE (ipc, "IPC manager", &mod_ops, {});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
