#include "i386.h"
#include "kernel.h"
#include "spinlock.h"
#include "printf.h"

extern unsigned _kernelstart;


/* Declare space for a stack */
unsigned ul_stack[NR_MODS][1024] __attribute__ ((aligned (4096))); 

/* Declare space for a task state segment */
unsigned ul_tss[NR_MODS][1024] __attribute__ ((aligned (4096))); 

/* Declare space for a page directory */
unsigned pg_dir[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table */
unsigned pg_table[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for per process kernel stack */
unsigned kl_stack[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a page table mappings for kernel stacks */
unsigned kls_pg_table[NR_MODS][1024] __attribute__ ((aligned (4096)));

/* Declare space for a dummy TSS -- used for kernel switch_to/jmp_gate
   semantics */
tss dummyTSS;

/* This is a global index into the GDT for a dummyTSS */
unsigned short dummyTSS_selector;

/* Each CPU gets an IDLE task -- something to do when nothing else */
tss idleTSS[MAX_CPUS];
unsigned short idleTSS_selector[MAX_CPUS];

/* Declare space for bitmap (physical) memory usage table.
 * PHYS_INDEX_MAX entries of 32-bit integers each for a 4K page => 
 * 4GB memory limit when PHYS_INDEX_MAX=32768
 */
unsigned mm_table[PHYS_INDEX_MAX] __attribute__ ((aligned (4096)));
unsigned mm_limit;       /* Actual physical page limit */

char *pchVideo = (char *) KERN_SCR;

/* NB: If limit is not a multiple of the system word size then all bits in
   table beyond limit must be set to zero */
int bitmap_find_first_set( unsigned int *table, unsigned int limit ) {
  
  int i;

  for( i = 0; i < ( limit >> 5 ); i++ )
    if( table[i] )
      return ffs( table[i] ) + ( i << 5 );

  return -1;
}


struct spinlock screen_lock = {0};


int _putchar( int ch ) {

    static int x, y;
    
    if( ch == '\n' ) {
	x = 0;
	y++;

	if( y > 24 )
	    y = 0;
	
	return (int) (unsigned char) ch;
    }

    pchVideo[ y * 160 + x * 2 ] = ch;
    pchVideo[ y * 160 + x * 2 + 1 ] = 7;
    x++;

    return (int) (unsigned char) ch;
}

int putchar( int ch ) {
  int x;
  spinlock_lock(&screen_lock);
  x = _putchar (ch);
  spinlock_unlock(&screen_lock);
  return x;
}

int print( char *pch ) {
    spinlock_lock(&screen_lock);
    while( *pch )
	_putchar( *pch++ );
    spinlock_unlock(&screen_lock);
    return 0;
}


void putx( unsigned long l ) {

    int i, li;

    spinlock_lock(&screen_lock);
    for( i = 7; i >= 0; i-- )
	if( ( li = ( l >> ( i << 2 ) ) & 0x0F ) > 9 )
	    _putchar( 'A' + li - 0x0A );
	else
	    _putchar( '0' + li );
    spinlock_unlock(&screen_lock);
}

int _print( char *pch ) {
    while( *pch )
	_putchar( *pch++ );
    return 0;
}


void _putx( unsigned long l ) {

    int i, li;

    for( i = 7; i >= 0; i-- )
	if( ( li = ( l >> ( i << 2 ) ) & 0x0F ) > 9 )
	    _putchar( 'A' + li - 0x0A );
	else
	    _putchar( '0' + li );
}

void com1_putc(char c) {
#ifdef COM1_TO_SCREEN
  _putchar(c);
#else
  if(c == '\n') {
    /* output CR before NL */
    while(!(inb(PORT1+5) & 0x20)); /* check line status register, empty transmitter bit */
    outb('\r', PORT1);
  }

  while(!(inb(PORT1+5) & 0x20)); /* check line status register, empty transmitter bit */
  outb(c, PORT1);
#endif
}

void com1_puts(char *p) {
  while (*p) 
    com1_putc(*p++);
}

void com1_putx(unsigned long l) {
    int i, li;

    for( i = 7; i >= 0; i-- )
	if( ( li = ( l >> ( i << 2 ) ) & 0x0F ) > 9 )
	    com1_putc( 'A' + li - 0x0A );
	else
	    com1_putc( '0' + li );
}



/* Find free page in mm_table 
 *
 * Returns physical address rather than virtual, since we
 * we don't want user-level pages mapped into kernel page tables in all cases
 */
unsigned AllocatePhysicalPage( void ) {

  int i;
  
  for( i = 0; i < mm_limit; i++ )
    if( BITMAP_TST( mm_table, i) ) { /* Free page */
      BITMAP_CLR( mm_table, i );
      return ( i << 12 );	/* physical byte address of free page/frame */
    }

  return -1;			/* Error -- no free page? */
}

unsigned AllocatePhysicalPages(unsigned count) {

  int i, j;
  
  for( i = 0; i < mm_limit - count + 1; i++ ) {
    for(j=0;j<count;j++) {
      if(!BITMAP_TST(mm_table, i+j)) { /* Is not free page? */
        i=i+j;
        goto keep_searching;
      }
    }
    /* found window: */
    for(j=0;j<count;j++) {
      BITMAP_CLR(mm_table, i+j); 
    }
    return ( i << 12 ); /* physical byte address of free frames */
  keep_searching:
    ;
  }
  return -1;			/* Error -- no free page? */
}

void FreePhysicalPage(unsigned frame) {
  BITMAP_SET(mm_table, frame >> 12);
}

void FreePhysicalPages(unsigned frame, unsigned count) {
  int i;
  frame >>= 12;
  for(i = 0; i < count; i++)
    BITMAP_SET(mm_table, frame+i);
}


/* Find free virtual page and map it to a corresponding physical frame 
 *
 * Returns virtual address
 *
 */
void *MapVirtualPage( unsigned phys_frame ) {

  unsigned *page_table = (unsigned *) KERN_PGT;
  int i;
  void *va;

  for( i = 0; i < 0x400; i++ )
    if( !page_table[ i ] ) {	/* Free page */
      page_table[ i ] = phys_frame;

      va = (char *)&_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      invalidate_page( va ); 

      return va;
    }

  return NULL;			/* Invalid address */
}

/* Map contiguous physical to virtual memory */
void *MapContiguousVirtualPages(unsigned phys_frame, unsigned count) {
  unsigned *page_table = (unsigned *) KERN_PGT;
  int i,j;
  void *va;

  if(count == 0) return NULL;

  for( i = 0; i < 0x400 - count + 1; i++ ) {
    if( !page_table[ i ] ) {	/* Free page */
      for(j=0; j<count; j++) {
        if (page_table[i+j]) {
          /* Not enough pages in this window */
          i = i+j;
          goto keep_searching;
        }
      }

      for(j=0; j<count; j++) {
        page_table[ i+j ] = phys_frame + j*0x1000;
      }

      va = (char *)&_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      for(j=0; j<count; j++) {
        invalidate_page( va + j*0x1000 ); 
      }

      return va;
    }
  keep_searching:
    ;
  }

  return NULL;			/* Invalid address */
}

/* Map non-contiguous physical memory to contiguous virtual memory */
void *MapVirtualPages(unsigned *phys_frames, unsigned count) {
  unsigned *page_table = (unsigned *) KERN_PGT;
  int i,j;
  void *va;

  if(count == 0) return NULL;

  for( i = 0; i < 0x400 - count + 1; i++ ) {
    if( !page_table[ i ] ) {	/* Free page */
      for(j=0; j<count; j++) {
        if (page_table[i+j]) {
          /* Not enough pages in this window */
          i = i+j;
          goto keep_searching;
        }
      }

      for(j=0; j<count; j++) {
        page_table[ i+j ] = phys_frames[j];
      }

      va = (char *)&_kernelstart + (i << 12);

      /* Invalidate page in case it was cached in the TLB */
      for(j=0; j<count; j++) {
        invalidate_page( va + j*0x1000 ); 
      }

      return va;
    }
  keep_searching:
    ;
  }

  return NULL;			/* Invalid address */
}


/* 
 * Release previously mapped virtual page 
 */
void UnmapVirtualPage( void *virt_addr ) {

    unsigned *page_table = (unsigned *) KERN_PGT;

    page_table[ ((unsigned)virt_addr >> 12) & 0x3FF ] = 0;

    /* Invalidate page in case it was cached in the TLB */
    invalidate_page( virt_addr ); 
}

void UnmapVirtualPages(void *virt_addr, unsigned count) {
  int j;
  for(j=0; j<count; j++)
    UnmapVirtualPage(virt_addr + j*0x1000);
}

void *get_phys_addr( void *virt_addr ) {

  void *pa;
  unsigned phys_frame;
  unsigned va = (unsigned)virt_addr;

  unsigned *kernel_pdbr = (unsigned *)get_pdbr(); /* --WARN-- Assumes
						   * virtual and phys addrs
						   * are the same. Okay to
						   * use at boot time up
						   * until we activate the
						   * first dynamically
						   * created address space
						   */
  unsigned *kernel_ptbr;

  kernel_ptbr = (unsigned *)( kernel_pdbr[va >> 22] & 0xFFFFF000 );

  phys_frame = kernel_ptbr[(va >> 12) & 0x3FF] & 0xFFFFF000;
  
  pa = (void *)( phys_frame + ( va & 0x00000FFF ));

  return pa;
}


__attribute__((noreturn)) void panic( char *sz ) {

    print( "kernel panic: " );
    print( sz );

    cli();
    hlt();
}



extern quest_tss *LookupTSS( unsigned short selector ) {

    descriptor *ad = (descriptor *) KERN_GDT;
    
    return (quest_tss *) ( ad[ selector >> 3 ].pBase0 |
			   ( ad[ selector >> 3 ].pBase1 << 16 ) |
			   ( ad[ selector >> 3 ].pBase2 << 24 ));    
}

void stacktrace(void) {
  unsigned esp,ebp;
  extern void com1_putc(char);
  extern void com1_puts(char *);
  extern void com1_putx(unsigned long);
  asm volatile("movl %%esp, %0" : "=r"(esp));
  asm volatile("movl %%ebp, %0" : "=r"(ebp));
  com1_printf("Stacktrace:\n");
  while(ebp >= KERN_STK && ebp <= KERN_STK+0x1000) {
    com1_printf("%0.8X\n", *((unsigned *)(ebp+4)));
    ebp = *((unsigned *)ebp);
  }
}

void idle_task(void) {
  unlock_kernel();
  sti();                        /* when we initially jump here, IF=0 */
  for(;;) {
    asm volatile ("hlt");
  }
}

void disable_idt(void) {
  WORD len = *((WORD *)idt_ptr);
  idt_descriptor *ptr = *((idt_descriptor **)(idt_ptr + 2));
  WORD i;
  
  for (i=0;i<(len >> 3);i++) {
    if(ptr[i].pBase0)
      ptr[i].fPresent = 0;
  }
}

void enable_idt(void) {
  WORD len = *((WORD *)idt_ptr);
  idt_descriptor *ptr = *((idt_descriptor **)(idt_ptr + 2));
  WORD i;
  
  for (i=0;i<(len >> 3);i++) {
    if(ptr[i].pBase0)
      ptr[i].fPresent = 1;
  }
}

void enable_idt_entry(WORD i) {
  idt_descriptor *ptr = *((idt_descriptor **)(idt_ptr + 2));
  if(ptr[i].pBase0)
    ptr[i].fPresent = 1;
}  

void set_idt_descriptor_by_addr(BYTE n, void *addr, BYTE dpl) {
  idt_descriptor *ptr = *((idt_descriptor **)(idt_ptr + 2));

  ptr[n].fPresent = 0;          /* disable */
  ptr[n].pBase1 = ((unsigned)addr & 0xFFFF0000) >> 16;
  ptr[n].pBase0 = ((unsigned)addr & 0x0000FFFF);
  ptr[n].pSeg = 0x08;
  ptr[n].fZero0 = 0;
  ptr[n].fZero1 = 0;
  ptr[n].fReserved = 0;
  ptr[n].fType = 0x6;
  ptr[n].f32bit = 1;
  ptr[n].uDPL = dpl;
  ptr[n].fPresent = 1;          /* re-enable */
}

void get_idt_descriptor(BYTE n, idt_descriptor *d) {
  idt_descriptor *ptr = *((idt_descriptor **)(idt_ptr + 2));

  *d = ptr[n];
#if 0
  d->pBase1           = ptr[n].pBase1;
  d->pBase0           = ptr[n].pBase0;
  d->pSeg             = ptr[n].pSeg;
  d->f0               = ptr[n].f0;
  d->fReserved        = ptr[n].fReserved;
  d->fType            = ptr[n].fType;
  d->f32bit           = ptr[n].f32bit;
  d->uDPL             = ptr[n].uDPL;
  d->fPresent         = ptr[n].fPresent;
#endif
}


void set_idt_descriptor(BYTE n, idt_descriptor *d) {
  idt_descriptor *ptr = *((idt_descriptor **)(idt_ptr + 2));

  ptr[n] = *d;
}

void tsc_delay_usec(DWORD usec) {
  extern QWORD tsc_freq;
  QWORD f;
  DWORD ticks, f_hi, f_lo;
  QWORD start, value, finish;
  DWORD divisor = 1000000;

  f = tsc_freq * usec;
  f_hi = (DWORD) (f >> 32);
  f_lo = (DWORD) (f & 0xFFFFFFFF);
  asm volatile("div %1" : "=a"(ticks) : "r"(divisor), "a"(f_lo), "d"(f_hi));

  RDTSC(start);

  finish = start+ticks;
  for(;;) {
    RDTSC(value);
    if(value >= finish) break;
    asm volatile("pause");
  }
}
