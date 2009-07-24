#ifndef _KERNEL_H_
#define _KERNEL_H_
#define PIT_FREQ 1193181        /* in Hz */
#define HZ 100
#define MAX_CPUS 8

/* Define some constants for virtual addresses */
#define KERN_STK 0xFF800000     /* Kernel stack */
#define KERN_IDT 0xFFFEF000     /* Kernel IDT */
#define KERN_GDT 0xFFFEF800     /* kernel GDT */
#define KERN_SCR 0xFFFF0000     /* Screen (kernel virtual) memory  */
#define KERN_PGT 0xFFFF1000     /* kernel page table */

#define PHYS_INDEX_MAX 32768

/* Bitmap utility functions for e.g., physical memory map and also
   scheduling priority queues */
#define BITMAP_SET(table,index) ((table)[(index)>>5] |= (1 << ((index) & 31)))
#define BITMAP_CLR(table,index) ((table)[(index)>>5] &= ~(1 << ((index) & 31)))
#define BITMAP_TST(table,index) ((table)[(index)>>5] & (1 << ((index) & 31)))

#define NULL 0

#define NR_MODS 10              /* Establish support for modules loaded by
                                   grub at boot time */

#define MAX_PRIO_QUEUES 32      /* NOTE: linux uses 140 */
#define MIN_PRIO ( MAX_PRIO_QUEUES - 1 )

/* Don't let preprocessed assemby files (*.S) include these lines */
#ifndef __ASSEMBLER__
#include "i386.h"
#include "spinlock.h"

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
extern uint32 mm_table[];       /* Bitmap for free/mapped physical pages */
extern uint32 mm_limit;         /* Actual physical page limit */
extern uint16 runqueue[];       /* TSS of next runnable task; 0 if none */

extern uint32 AllocatePhysicalPage (void);
extern uint32 AllocatePhysicalPages (uint32);
extern void FreePhysicalPage (uint32);
extern void FreePhysicalPages (uint32, uint32);
extern void *MapVirtualPage (uint32 phys_frame);
extern void UnmapVirtualPage (void *virt_addr);
extern void *MapVirtualPages (uint32 * phys_frames, uint32 count);
extern void *MapContiguousVirtualPages (uint32 phys_frame, uint32 count);
extern void UnmapVirtualPages (void *virt_addr, uint32 count);
extern void *get_phys_addr (void *virt_addr);
extern quest_tss *LookupTSS (uint16 selector);
extern int bitmap_find_first_set (unsigned int *table, unsigned int limit);

extern void panic (char *sz) __attribute__ ((noreturn));

extern char *pchVideo;
extern int putchar (int ch);
extern int print (char *pch);
extern void putx (uint32 l);
/* unlocked: */
extern int _putchar (int ch);
extern int _print (char *pch);
extern void _putx (uint32 l);

extern void com1_putc (char);
extern void com1_puts (char *);
extern void com1_putx (uint32);

extern void runqueue_append (unsigned int prio, uint16 selector);
extern void queue_append (uint16 * queue, uint16 selector);
extern uint16 queue_remove_head (uint16 * queue);
extern void schedule (void);
extern void wakeup (uint16);
extern void wakeup_queue (uint16 *);
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

static inline void *
memset (void *p, int ch, uint32 cb)
{

  asm volatile ("rep stosb"::"D" (p), "a" (ch), "c" (cb));
  return p;
}

static inline void *
memcpy (void *pDest, const void *pSrc, uint32 cb)
{

  asm volatile ("rep movsb"::"D" (pDest), "S" (pSrc), "c" (cb));
  return pDest;
}

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

typedef unsigned int pid_t;

extern spinlock screen_lock;

extern uint8 idt_ptr[];

extern uint8 sched_enabled;

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
