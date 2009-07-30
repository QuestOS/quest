#ifndef _KERNEL_H_
#define _KERNEL_H_
#define PIT_FREQ 1193181        /* in Hz */
#define HZ 100
#define MAX_CPUS 8

#include "kernel-defs.h"

/* Bitmap utility functions for e.g., physical memory map and also
   scheduling priority queues */
#define BITMAP_SET(table,index) ((table)[(index)>>5] |= (1 << ((index) & 31)))
#define BITMAP_CLR(table,index) ((table)[(index)>>5] &= ~(1 << ((index) & 31)))
#define BITMAP_TST(table,index) ((table)[(index)>>5] & (1 << ((index) & 31)))

/* Don't let preprocessed assemby files (*.S) include these lines */
#ifndef __ASSEMBLER__
#include "arch/i386.h"
#include "smp/spinlock.h"

struct sched_param
{
  int sched_priority;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
};

typedef struct _quest_tss
{
  tss tss;                      /* hardware defined portion of TSS */
  uint16 next;                  /* selector for next TSS in corresponding queue
                                   (beit the runqueue for the CPU or a waitqueue for
                                   a resource; 0 if task is at end of queue. If a
                                   task is runnable 'next' refers to a TSS selector
                                   on the runqueue; if a task is waiting on a
                                   resource, 'next' refers to a TSS selector on the
                                   corresponding waitqueue; for all other cases,
                                   'next' is irrelevant */
  uint16 waitqueue;             /* queue of other tasks waiting for this
                                   one -- either attempting to send IPC to it,
                                   or waiting for it to exit */
  bool busy;                    /* mutex for server: when busy, clients must add themselves to
                                   waitqueue above */
  uint32 priority;
} quest_tss;

extern char *kernel_version;
extern uint16 runqueue[];       /* TSS of next runnable task; 0 if none */

extern quest_tss *lookup_TSS (uint16 selector);

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
extern void init_interrupt_handlers (void);

void stacktrace (void);

void tsc_delay_usec (uint32 usec);

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

extern tss dummyTSS;

extern uint16 dummyTSS_selector;

extern tss idleTSS[MAX_CPUS];

extern uint16 idleTSS_selector[MAX_CPUS];

typedef uint16 task_id;

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

#endif /* __ASSEMBLER__ */
#endif

/*
 * Overrides for Emacs
 * Emacs will notice this stuff at the end of the file and automatically
 * adjust the settings for this buffer only.  This must remain at the end
 * of the file.
 *
---------------------------------------------------------------------------
 * Local variables:
 * c-indent-level: 4
 * c-brace-imaginary-offset: 0
 * c-brace-offset: -4
 * c-argdecl-indent: 4
 * c-label-offset: -4
 * c-continued-statement-offset: 4
 * c-continued-brace-offset: 0
 * End:
 */
