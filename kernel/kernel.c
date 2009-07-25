#include "arch/i386.h"
#include "kernel.h"
#include "smp/spinlock.h"
#include "util/printf.h"

/* Declare space for a stack */
uint32 ul_stack[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a task state segment */
uint32 ul_tss[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page directory */
uint32 pg_dir[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table */
uint32 pg_table[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for per process kernel stack */
uint32 kl_stack[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table mappings for kernel stacks */
uint32 kls_pg_table[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a dummy TSS -- used for kernel switch_to/jmp_gate
   semantics */
tss dummyTSS;

/* This is a global index into the GDT for a dummyTSS */
uint16 dummyTSS_selector;

/* Each CPU gets an IDLE task -- something to do when nothing else */
tss idleTSS[MAX_CPUS];
uint16 idleTSS_selector[MAX_CPUS];

char *pchVideo = (char *) KERN_SCR;

/* NB: If limit is not a multiple of the system word size then all bits in
   table beyond limit must be set to zero */
int
bitmap_find_first_set (unsigned int *table, unsigned int limit)
{

  int i;

  for (i = 0; i < (limit >> 5); i++)
    if (table[i])
      return ffs (table[i]) + (i << 5);

  return -1;
}


spinlock screen_lock = { 0 };


int
_putchar (int ch)
{

  static int x, y;

  if (ch == '\n') {
    x = 0;
    y++;

    if (y > 24)
      y = 0;

    return (int) (uint8) ch;
  }

  pchVideo[y * 160 + x * 2] = ch;
  pchVideo[y * 160 + x * 2 + 1] = 7;
  x++;

  return (int) (uint8) ch;
}

int
putchar (int ch)
{
  int x;
  spinlock_lock (&screen_lock);
  x = _putchar (ch);
  spinlock_unlock (&screen_lock);
  return x;
}

int
print (char *pch)
{
  spinlock_lock (&screen_lock);
  while (*pch)
    _putchar (*pch++);
  spinlock_unlock (&screen_lock);
  return 0;
}


void
putx (uint32 l)
{

  int i, li;

  spinlock_lock (&screen_lock);
  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      _putchar ('A' + li - 0x0A);
    else
      _putchar ('0' + li);
  spinlock_unlock (&screen_lock);
}

int
_print (char *pch)
{
  while (*pch)
    _putchar (*pch++);
  return 0;
}


void
_putx (uint32 l)
{

  int i, li;

  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      _putchar ('A' + li - 0x0A);
    else
      _putchar ('0' + li);
}

void
com1_putc (char c)
{
#ifdef COM1_TO_SCREEN
  _putchar (c);
#else
  if (c == '\n') {
    /* output CR before NL */
    while (!(inb (PORT1 + 5) & 0x20));  /* check line status register, empty transmitter bit */
    outb ('\r', PORT1);
  }

  while (!(inb (PORT1 + 5) & 0x20));    /* check line status register, empty transmitter bit */
  outb (c, PORT1);
#endif
}

void
com1_puts (char *p)
{
  while (*p)
    com1_putc (*p++);
}

void
com1_putx (uint32 l)
{
  int i, li;

  for (i = 7; i >= 0; i--)
    if ((li = (l >> (i << 2)) & 0x0F) > 9)
      com1_putc ('A' + li - 0x0A);
    else
      com1_putc ('0' + li);
}

__attribute__ ((noreturn))
     void panic (char *sz)
{

  print ("kernel panic: ");
  print (sz);

  cli ();
  hlt ();
}



extern quest_tss *
lookup_TSS (uint16 selector)
{

  descriptor *ad = (descriptor *) KERN_GDT;

  return (quest_tss *) (ad[selector >> 3].pBase0 |
                        (ad[selector >> 3].pBase1 << 16) |
                        (ad[selector >> 3].pBase2 << 24));
}

void
stacktrace (void)
{
  uint32 esp, ebp;
  extern void com1_putc (char);
  extern void com1_puts (char *);
  extern void com1_putx (uint32);
  asm volatile ("movl %%esp, %0":"=r" (esp));
  asm volatile ("movl %%ebp, %0":"=r" (ebp));
  com1_printf ("Stacktrace:\n");
  while (ebp >= KERN_STK && ebp <= KERN_STK + 0x1000) {
    com1_printf ("%0.8X\n", *((uint32 *) (ebp + 4)));
    ebp = *((uint32 *) ebp);
  }
}

void
idle_task (void)
{
  unlock_kernel ();
  sti ();                       /* when we initially jump here, IF=0 */
  for (;;) {
    asm volatile ("hlt");
  }
}

void
disable_idt (void)
{
  uint16 len = *((uint16 *) idt_ptr);
  idt_descriptor *ptr = *((idt_descriptor **) (idt_ptr + 2));
  uint16 i;

  for (i = 0; i < (len >> 3); i++) {
    if (ptr[i].pBase0)
      ptr[i].fPresent = 0;
  }
}

void
enable_idt (void)
{
  uint16 len = *((uint16 *) idt_ptr);
  idt_descriptor *ptr = *((idt_descriptor **) (idt_ptr + 2));
  uint16 i;

  for (i = 0; i < (len >> 3); i++) {
    if (ptr[i].pBase0)
      ptr[i].fPresent = 1;
  }
}

void
enable_idt_entry (uint16 i)
{
  idt_descriptor *ptr = *((idt_descriptor **) (idt_ptr + 2));
  if (ptr[i].pBase0)
    ptr[i].fPresent = 1;
}

void
set_idt_descriptor_by_addr (uint8 n, void *addr, uint8 dpl)
{
  idt_descriptor *ptr = *((idt_descriptor **) (idt_ptr + 2));

  ptr[n].fPresent = 0;          /* disable */
  ptr[n].pBase1 = ((uint32) addr & 0xFFFF0000) >> 16;
  ptr[n].pBase0 = ((uint32) addr & 0x0000FFFF);
  ptr[n].pSeg = 0x08;
  ptr[n].fZero0 = 0;
  ptr[n].fZero1 = 0;
  ptr[n].fReserved = 0;
  ptr[n].fType = 0x6;
  ptr[n].f32bit = 1;
  ptr[n].uDPL = dpl;
  ptr[n].fPresent = 1;          /* re-enable */
}

void
get_idt_descriptor (uint8 n, idt_descriptor * d)
{
  idt_descriptor *ptr = *((idt_descriptor **) (idt_ptr + 2));

  *d = ptr[n];
#if 0
  d->pBase1 = ptr[n].pBase1;
  d->pBase0 = ptr[n].pBase0;
  d->pSeg = ptr[n].pSeg;
  d->f0 = ptr[n].f0;
  d->fReserved = ptr[n].fReserved;
  d->fType = ptr[n].fType;
  d->f32bit = ptr[n].f32bit;
  d->uDPL = ptr[n].uDPL;
  d->fPresent = ptr[n].fPresent;
#endif
}


void
set_idt_descriptor (uint8 n, idt_descriptor * d)
{
  idt_descriptor *ptr = *((idt_descriptor **) (idt_ptr + 2));

  ptr[n] = *d;
}

void
tsc_delay_usec (uint32 usec)
{
  extern uint64 tsc_freq;
  uint64 f;
  uint32 ticks, f_hi, f_lo;
  uint64 start, value, finish;
  uint32 divisor = 1000000;

  f = tsc_freq * usec;
  f_hi = (uint32) (f >> 32);
  f_lo = (uint32) (f & 0xFFFFFFFF);
  asm volatile ("div %1":"=a" (ticks):"r" (divisor), "a" (f_lo), "d" (f_hi));

  RDTSC (start);

  finish = start + ticks;
  for (;;) {
    RDTSC (value);
    if (value >= finish)
      break;
    asm volatile ("pause");
  }
}
