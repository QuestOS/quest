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

#ifndef _PROC_H_
#define _PROC_H_

#include "smp/semaphore.h"
#include "util/cassert.h"
#include "util/circular.h"
#include "linux_socket.h"
#include "types.h"

typedef union
{
  uint32 raw;
  struct {
    uint16 org_sandbox;  /* The original sandbox where this task was created */
    uint16 tid;          /* Task ID that is only unique in each sandbox but not globally */
  };
} quest_id PACKED;
CASSERT (sizeof (task_id) == sizeof (uint32), task_id);

#define NUM_M   32
#define MAX_FD  32

#define FD_TYPE_FILE    0
#define FD_TYPE_UDP     1
#define FD_TYPE_TCP     2
#define FD_TYPE_USB_DEV 3

struct _quest_tss;

typedef struct _fd_table_entry {
  uint8 type;
  uint32 flags;
  void * entry;
  udp_recv_buf_t * udp_recv_buf;
  tcp_recv_buf_t * tcp_recv_buf;
  int * tcp_accept_buf;
  circular * udp_recv_buf_circ;
  circular * tcp_recv_buf_circ;
  circular * tcp_accept_circ;
  struct _quest_tss * task;
} fd_table_entry_t;

typedef struct _fd_table_file_entry
{
  char* pathname;
  int current_pos;
  unsigned char* file_buffer;
  size_t file_length;
} fd_table_file_entry_t;

fd_table_file_entry_t* alloc_fd_table_file_entry(char* pathname, size_t file_length);
void free_fd_table_file_entry(fd_table_file_entry_t* entry);

/* --YL-- We have a cyclic include in kernel.h involves proc.h and vcpu.h */
#ifndef _SCHED_VCPU_REPL_
#define _SCHED_VCPU_REPL_
typedef struct _replenishment {
  u64 t, b;
  struct _replenishment *next;
} replenishment;
#define MAX_REPL 32
#endif

struct _fault_detection_info;

/* A Quest TSS is a software-only construct, a.k.a Thread Control
 * Block (TCB). */
typedef struct _quest_tss
{
  u32 ESP;
  u32 EBP;
  /* The initial instruction pointer is written both to here and the
   * kernel stack.  In the case of the initial user-space application,
   * this field is used to load the first IRET into user-space.  In
   * all other cases, the kernel stack contains the EIP to resume
   * operation. */
  u32 initial_EIP;
  u32 CR3;
  u32 EFLAGS;
  u8 padding[12];
  u8 fpu_state[512];            /* 512 bytes for fpu and mmx state */
  struct _semaphore Msem;
  u32 M[NUM_M];

  task_id next;                 /* selector for next TSS in corresponding queue
                                   (beit the runqueue for the CPU or a waitqueue for
                                   a resource; 0 if task is at end of queue. If a
                                   task is runnable 'next' refers to a TSS selector
                                   on the runqueue; if a task is waiting on a
                                   resource, 'next' refers to a TSS selector on the
                                   corresponding waitqueue; for all other cases,
                                   'next' is irrelevant */
  task_id waitqueue;            /* queue of other tasks waiting for this
                                   one -- either attempting to send IPC to it,
                                   or waiting for it to exit */
  bool busy;                    /* mutex for server: when busy, clients must add themselves to
                                   waitqueue above */
  uint32 priority;
  uint64 time;                  /* A field for time values associated
                                   with task, for example, to be used
                                   by waitqueue managers. */
  u16 cpu;                      /* [V]CPU binding */
  struct _quest_tss * next_tss;
  struct _quest_tss * prev_tss;
  task_id tid;
  char name[32];              /* A simple description or the path of the process */
  uint sandbox_affinity;        /* Sandbox binding. Change this to migrate. */

  /* -- EM -- Machine affinity right now since we do not have a global
     namespace across machines a 0 here indicate stay on current
     machine and a non-zero means migrate to the one other machine */
  uint machine_affinity;
  
  fd_table_entry_t fd_table[MAX_FD];
  /* Array used to back up (main) VCPU replenishment queue if necessary during migration */
  replenishment vcpu_backup[MAX_REPL];
  /* common VCPU scheduling parameters backup */
  u64 C_bak, T_bak, b_bak, usage_bak;
  struct _fault_detection_info* fdi;
  u8 padding2[1];
} PACKED quest_tss;

CASSERT(sizeof(quest_tss) <= 0x1000, quest_tss_size);
CASSERT((sizeof(quest_tss) % 32) == 0, quest_tss_alignment);

static inline int
find_fd (quest_tss *tss)
{
  int i;

  for (i = 3; i < MAX_FD; i++) {
    if (tss->fd_table[i].entry == NULL) {
      tss->fd_table[i].flags = 0; /* Reset flags for new fd */
      return i;
    }
  }

  return -1;
}

extern quest_tss init_tss;

extern task_id
duplicate_TSS (uint32 ebp,
               uint32 *esp,
               uint32 child_eip,
               uint32 child_ebp,
               uint32 child_esp,
               uint32 child_eflags,
               uint32 child_directory);

extern quest_tss * alloc_quest_tss (void);
extern void free_quest_tss (quest_tss *tss);
extern task_id alloc_idle_TSS (int cpu_num);
extern task_id alloc_TSS (void *pPageDirectory, void *pEntry, int mod_num);
extern quest_tss *lookup_TSS (task_id tid);
extern void tss_add_head (quest_tss * new_tss);
extern void tss_add_tail (quest_tss * new_tss);
extern task_id new_task_id (void);
extern void tss_remove (task_id tid);

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
