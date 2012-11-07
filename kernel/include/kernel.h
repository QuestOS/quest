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

#ifndef _KERNEL_H_
#define _KERNEL_H_

//#define ENABLE_GDBSTUB          /* the remote debugger via GDB */

#ifdef ENABLE_GDBSTUB
#define GDBSTUB_TCP             /* using TCP instead of serial port */
#define GDBSTUB_TCP_PORT  1234
#define GDBSTUB_ETHDEV    "en0"
#define GDBSTUB_BUFFER_SIZE 512
#define BREAKPOINT() asm("   int $3");
#else
#define BREAKPOINT() ;
#endif

#define PIT_FREQ 1193181        /* in Hz */
#define HZ 500
#define MAX_CPUS 8

#include "kernel-defs.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#define container_of(ptr, type, member) ({                      \
      const typeof( ((type *)0)->member ) *__mptr = (ptr);      \
      (type *)( (char *)__mptr - offsetof(type,member) );})

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define BIT(nr)                 (1UL << (nr))
#define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#define BITS_PER_BYTE           8
#define BITS_TO_LONGS(nr)       DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(long))


/* Bitmap utility functions for e.g., physical memory map and also
   scheduling priority queues */
#define BITMAP_SET(table,index) ((table)[(index)>>5] |= (1 << ((index) & 31)))
#define BITMAP_CLR(table,index) ((table)[(index)>>5] &= ~(1 << ((index) & 31)))
#define BITMAP_TST(table,index) ((table)[(index)>>5] & (1 << ((index) & 31)))

/* Don't let preprocessed assemby files (*.S) include these lines */
#ifndef __ASSEMBLER__
#include "arch/i386.h"
#include "smp/spinlock.h"
#include "sched/proc.h"
#include "types.h"

//#define USER_STACK_START 0x400000
#define USER_STACK_START 0x40000000
#define USER_STACK_SIZE 16

struct sched_param
{
  int sched_priority;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
  int affinity;                 /* CPU (or Quest-V sandbox affinity) */
};

void map_user_level_stack(uint32_t* plPageDirectory, void* start_addr, int num_frames,
                          uint32_t* frames, bool invalidate_pages);

extern char *kernel_version;
extern uint16 runqueue[];       /* TSS of next runnable task; 0 if none */

extern void *lookup_GDT_selector (uint16 selector);
extern void get_GDT_descriptor (uint16, descriptor *);

extern void panic (char *sz) __attribute__ ((noreturn));

extern void lock_kernel (void);
extern void unlock_kernel (void);

extern void disable_idt (void);
extern void enable_idt (void);
extern void enable_idt_entry (uint16);
extern void set_idt_descriptor_by_addr (uint8, void *, uint8);
extern void get_idt_descriptor (uint8, idt_descriptor *);
extern void set_idt_descriptor (uint8, idt_descriptor *);

typedef uint32 (*vector_handler) (uint8 vector);
extern void set_vector_handler (uint8 vector, vector_handler func);
extern void clr_vector_handler (uint8 vector);
extern vector_handler get_vector_handler (uint8 vector);
#define MINIMUM_VECTOR_PRIORITY 0x4
extern bool vector_used (uint8 vector);
extern u8 find_unused_vector (u8 min_prio);
extern void init_interrupt_handlers (void);

void stacktrace (void);

void tsc_delay_usec (uint32 usec);

task_id start_kernel_thread (uint eip, uint esp, const char * name);
task_id create_kernel_thread_vcpu_args (uint eip, uint esp, const char * name,
                                        u16 vcpu, bool run, uint n, ...);

#define create_kernel_thread_args(eip, esp, name, run, n, ...)    \
        create_kernel_thread_vcpu_args(eip, esp, name, 0xFFFF, run, n, ##__VA_ARGS__)

void exit_kernel_thread (void);

#ifdef USE_VMX
void check_copied_threads (void);
#endif

/* Declare space for a stack */
extern uint32 ul_stack[][1024] __attribute__ ((aligned (4096)));

/* Declare space for a task state segment */
extern uint32 ul_tss[][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page directory */
extern uint32 pg_dir[][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table */
extern uint32 pg_table[][1024] __attribute__ ((aligned (4096)));

/* Declare space for per process kernel stack */
extern uint32 kl_stack[][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table mappings for kernel stacks */
extern uint32 kls_pg_table[][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table for the user stack (only used if different than pg_dir table */
extern uint32 uls_pg_table[NR_MODS][1024] ALIGNED(0x1000);

extern quest_tss idleTSS[MAX_CPUS];

extern tss cpuTSS[MAX_CPUS];

extern task_id idleTSS_selector[MAX_CPUS];

extern uint16 cpuTSS_selector[MAX_CPUS];

extern spinlock screen_lock;

extern uint8 idt_ptr[];

extern uint8 sched_enabled;

static inline uint8
checksum (uint8 * ptr, int length)
{
  uint8 sum = 0;
  while (length-- > 0)
    sum += *ptr++;
  return sum;
}

extern uint get_pcpu_id (void);

#endif /* __ASSEMBLER__ */
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
