/*
 *
 * Interrupt handler code: interrupt_handler.c
 *
 */

#include "i386.h"
#include "kernel.h"
#include "elf.h"
#include "filesys.h"
#include "smp.h"

static char kernel_ver[] = "0.1a";
char *kernel_version = kernel_ver;
unsigned long tick;		/* Software clock tick */

extern unsigned ul_tss[][1024];

/* Duplicate parent TSS -- used with fork */
static unsigned short DuplicateTSS( unsigned ebp, 
				    unsigned *esp,
				    unsigned child_directory ) {

    int i;
    descriptor *ad = (descriptor *) KERN_GDT; 
    tss *pTSS;
    unsigned pa;

    pa = AllocatePhysicalPage(); /* --??-- For now, whole page per tss */
                                 /* --??-- Error checking in the future */

    /* Establish space for a new TSS: +3 declares present and r/w */

    /* Note, we rely on page being initialised to 0 since EAX contains
     * return value for child
     */

    pTSS = MapVirtualPage( pa+3 );

    /* Clear virtual page before use.
       NOTE: This implicitly clears EAX used as the return value 
       for the child indicating a pid of 0 */
    memset( pTSS, 0, 4096 );
    
    /* Search 2KB GDT for first free entry */
    for( i = 1; i < 256; i++ )
	if( !( ad[ i ].fPresent ) )
	    break;

    if( i == 256 )
	panic( "No free selector for TSS" );

    /* See pp 6-7 in IA-32 vol 3 docs for meanings of these assignments */
    ad[ i ].uLimit0 = 0xFFF;	/* --??-- Right now, a page per TSS */
    ad[ i ].uLimit1 = 0;
    ad[ i ].pBase0 = (unsigned long) pTSS & 0xFFFF;
    ad[ i ].pBase1 = ( (unsigned long) pTSS >> 16 ) & 0xFF;
    ad[ i ].pBase2 = (unsigned long) pTSS >> 24;
    ad[ i ].uType = 0x09;	/* 32-bit tss */
    ad[ i ].uDPL = 0;		/* Only let kernel perform task-switching */
    ad[ i ].fPresent = 1;
    ad[ i ].f0 = 0;
    ad[ i ].fX = 0;
    ad[ i ].fGranularity = 0;	/* Set granularity of tss in bytes */

    pTSS->pCR3 = (void *) child_directory;

    /* Inherit these fields from parent */
    pTSS->ulEIP = esp[2]; /* See pp 5-15 IA-32 vol 3: This accesses the
			   * parent's return address on the kernel stack
			   * after entry via interrupt.S
			   */
    pTSS->ulEFlags = esp[4] & 0xFFFFBFFF; /* Disable NT flag */
    pTSS->ulESP = esp[5];
    pTSS->ulEBP = ebp;		/* Inherited from parent */
    pTSS->usES = 0x23;		/* --??-- boot.S defaults for ES,DS,FS & GS */
    pTSS->usCS = esp[3];
    pTSS->usSS = esp[6];	
    pTSS->usDS = 0x23;
    pTSS->usFS = 0x23;
    pTSS->usGS = 0x23;
    pTSS->usIOMap = 0xFFFF;
    pTSS->usSS0 = 0x10;		/* Kernel stack segment */
    pTSS->ulESP0 = (unsigned)KERN_STK + 0x1000;

    /* Return the index into the GDT for the segment */
    return i << 3;
}

char *exception_messages[] = {
  "Division By Zero",
  "Debug",
  "Non Maskable Interrupt",
  "Breakpoint",
  "Into Detected Overflow",
  "Out of Bounds",
  "Invalid Opcode",
  "No Coprocessor",
  "Double Fault",
  "Coprocessor Segment Overrun",
  "Bad TSS",
  "Segment Not Present",
  "Stack Fault",
  "General Protection Fault",
  "Page Fault",
  "Unknown Interrupt",
  "Coprocessor Fault",
  "Alignment Check",
  "Machine Check",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved",
  "Reserved"
};

extern void HandleInterrupt( unsigned long fs_gs, unsigned long ds_es, 
			     unsigned long ulInt, unsigned long ulCode ) {

    unsigned long eax, ebx, ecx, edx, esi, edi, eflags, eip, esp, ebp, cr2;

    asm volatile(
	"movl %%eax, %0\n"
	"movl %%ebx, %1\n"
	"movl %%ecx, %2\n"
	"movl %%edx, %3\n"
	"movl %%esi, %4\n"
	"movl %%edi, %5\n"
	"movl (%%ebp), %%eax\n"
	"movl %%eax, %6\n"
	"movl 0x18(%%ebp),%%eax\n"
	"movl %%eax, %7\n"
	"movl 0x20(%%ebp),%%eax\n"
	"movl %%eax, %8\n"
	"movl 0x24(%%ebp),%%eax\n"
	"movl %%eax, %9\n"
        "movl %%cr2, %%eax\n"
        "movl %%eax, %10\n"
        : "=m" (eax), "=m" (ebx), "=m" (ecx), "=m" (edx),
	"=m" (esi), "=m" (edi), "=m" (ebp), "=m" (eip), "=m" (eflags),
	"=m" (esp), "=m" (cr2) : );

    spinlock_lock(&screen_lock);
    _putchar( 'I' );
    _putx( ulInt );
    _putchar( ' ' );
    _putchar( 'c' );
    _putx( ulCode );
    _putchar( ' ' );
    if (ulInt < 32) _print( exception_messages[ulInt] );
    _putchar( '\n' );
    
    _putchar( 'A' );
    _putx( eax );
    _putchar( '\n' );
    _putchar( 'B' );
    _putx( ebx );
    _putchar( '\n' );
    _putchar( 'C' );
    _putx( ecx );
    _putchar( '\n' );
    _putchar( 'D' );
    _putx( edx );
    _putchar( '\n' );
    _putchar( 'S' );
    _putx( esi );
    _putchar( '\n' );
    _putchar( 'D' );
    _putx( edi );
    _putchar( '\n' );
    _putchar( 'B' );
    _putx( ebp ); 
    _putchar( '\n' );
    _putchar( 'S' );
    _putx( esp );   
    _putchar( '\n' );
    _putchar( 'I' );
    _putx( eip );
    _putchar( '\n' );
    _putchar( 'F' );
    _putx( eflags );
    _putchar( '\n' );
    spinlock_unlock(&screen_lock);
    
    if( ulInt < 0x20 )
	/* unhandled exception - die */
	/* asm volatile( "xorl %eax, %eax; movl %eax, %cr0" ); */
      while (1);
    
    send_eoi ();
    /******************************************************
     * asm volatile( "movb $0x20, %al; out %al, $0xA0" ); *
     * asm volatile( "movb $0x20, %al; out %al, $0x20" ); *
     ******************************************************/
}

extern void _interrupt3f(void) {
  BYTE phys_id = LAPIC_get_physical_ID();
  spinlock_lock(&screen_lock);
  _print ("CPU ");
  _putx (phys_id);
  _print (" received IPI vector=0x3f\n");
  spinlock_unlock(&screen_lock);
  send_eoi();
}


void HandleSyscall0 (int eax, int ebx) {

  quest_tss *pTSS = (quest_tss *)ul_tss[1];	/* --??-- tss index hard-coded to 1 for now */
  unsigned short head;

  if (eax) {			/* eax should be 0 for syscall0 */
    print ("Invalid syscall number!\n");
    return;
  }

  /* NB: The code below relies on atomic execution.  Currently, that
     atomicity is guaranteed by assuming we are running on a
     uniprocessor with interrupts disabled. */

  if( pTSS->busy ) {
      /* somebody else is already using the server; block */
      queue_append( &pTSS->waitqueue, str() ); /* add ourselves to the wait
						  queue -- this operation
						  must be atomic with the
						  busy check above */
      schedule();
      /* We can now safely assume that we have exclusive access to the
	 server (since the only way we could possibly have woken up after
	 the schedule() call is by the previous head task placing us on
	 the run queue -- see below). */
  } else
      pTSS->busy = 1; /* mark the server busy -- this set must be atomic
			 with the test above */

  pTSS->tss.ulEBX = ebx; /* pass arg in EBX from client to server */
	  	  
  call_gate( 0x30 ); /* --??-- Hard-coded for the segment selector for
			terminal server task */

  if( ( head = queue_remove_head( &pTSS->waitqueue ) ) )
      /* Somebody else is waiting for the server -- wake them up (and leave
	 the busy flag set).  This will eventually cause the schedule() call
	 in the waiting task to return (see above). */
      runqueue_append( LookupTSS( head )->priority, head );
  else
      /* We were the last task using the server; mark it as available.
	 Clearing this flag must be atomic with the queue_remove_head()
	 check. */
      pTSS->busy = 0;
}


/* Used for handling fork calls
 *
 * esp argument used to find info about parent's eip and other registers
 * inherited by child
 */
pid_t _fork ( unsigned ebp, unsigned *esp ) {

  unsigned short child_gdt_index;
  void *phys_addr;
  unsigned *virt_addr;
  int i, j;
  unsigned *child_directory, *child_page_table, *parent_page_table;
  void *child_page, *parent_page;
  unsigned tmp_dir, tmp_page, tmp_page_table;
  unsigned priority;

  child_directory = MapVirtualPage( (tmp_dir = AllocatePhysicalPage()) | 3 );

  child_gdt_index = DuplicateTSS( ebp, esp, tmp_dir );

  /* Allocate physical memory for new address space 
   *
   */
  phys_addr = get_pdbr();	/* Parent page dir base address */

  virt_addr = MapVirtualPage( (unsigned)phys_addr | 3 ); /* Temporary virtual address */

  for ( i = 0; i < 0x3FF; i++ ) { /* Walk user-level regions of pgd */
    
    if ( virt_addr[i] ) {	/* Valid page table found */
      child_page_table = MapVirtualPage( (tmp_page_table = AllocatePhysicalPage()) | 3 );
      parent_page_table = MapVirtualPage( (virt_addr[i] & 0xFFFFF000) | 3 );
      
      /* Copy parent's page table mappings to child */
      for ( j = 0; j < 1024; j++ ) {
	if ( parent_page_table[j] ) {/* --??-- Assume non-zero means present */
	  child_page =  MapVirtualPage( (tmp_page = AllocatePhysicalPage()) | 3);
	  parent_page = MapVirtualPage( (parent_page_table[j] & 0xFFFFF000) | 3 );

	  /* --??-- TODO: Copy-on-write style forking and support
	   for physical page frame sharing */
	  memcpy( child_page, parent_page, 0x1000 );
	  
	  child_page_table[j] = tmp_page | 
	    (parent_page_table[j] & 0xFFF );

	  UnmapVirtualPage( child_page );
	  UnmapVirtualPage( parent_page );
	}
	else {
	  child_page_table[j] = 0;
	}
      }

      /* Create page directory for child */
      child_directory[i] = tmp_page_table | ( virt_addr[i] & 0xFFF );

      UnmapVirtualPage( child_page_table );
      UnmapVirtualPage( parent_page_table );
      
    }
    else {
      child_directory[i] = 0;
    }
  }

  /* Copy kernel mappings into child's page directory */
  child_directory[1023] = virt_addr[1023];

  UnmapVirtualPage( virt_addr );

  /* Inherit priority from parent */
  priority = LookupTSS( child_gdt_index )->priority = LookupTSS( str() )->priority;

  runqueue_append( priority, child_gdt_index );
  
  /* --??-- Duplicate any other parent resources as necessary */

  return child_gdt_index;	/* Use this index for child ID for now */
}


char *strncpy( char *s1, const char *s2, int length ) {

  while( ( length-- ) && ( *s1++ = *s2++ ) );
  
  if( length < 0 )
    *(s1-1) = '\0';
  
  return s1;
}


/* --??-- TODO: Rewrite _exec to create a temporary new address space
 before overwriting the old one in case of errors */

/* _exec: replace address space of caller with new memory areas, in part
 * populated by program image on disk
 */
int _exec( char *filename, char *argv[], unsigned *curr_stack ) {

  unsigned long *plPageDirectory = MapVirtualPage( (unsigned) get_pdbr() | 3 );
  unsigned long *plPageTable;
  unsigned long pStack;
  Elf32_Ehdr *pe = (Elf32_Ehdr *)0xFF400000; /* 4MB below KERN_STK virt address */
  Elf32_Phdr *pph;
  void *pEntry;
  int filesize;
  /* Temporary storage for frame pointers for a file image up to 4MB
     discounting bss */
  unsigned long phys_addr = AllocatePhysicalPage() | 3;
  /* frame_map is a 1024 bit bitmap to mark frames not needed for 
     file of specific size when not all sections need loading into RAM */
  unsigned long frame_map[32];
  unsigned long *frame_ptr = MapVirtualPage( phys_addr );
  unsigned long *tmp_page;
  int i, j, c;
  char command_args[80];

  /* --??-- Checks should be added here for valid argv[0] etc...
     Allocate space for argument vector passed via exec call.
     Right now, assume max size for prog name and arguments.
  */
  strncpy( command_args, argv[0], 80 );
  

  /* Read filename from disk -- essentially a basic open call */
  if ( (filesize = ext2fs_dir( filename )) < 0 ) { /* Error */
    BITMAP_SET( mm_table, phys_addr >> 12 );
    UnmapVirtualPage( plPageDirectory );
    UnmapVirtualPage( frame_ptr );
    return -1;
  }

  /* Free frames used for old address space before _exec was called
   *
   * Reuse page directory
   */
  for( i = 0; i < 1019; i++ ) {	/* Skip freeing kernel pg table mapping and
				   kernel stack space. */
    if( plPageDirectory[i] ) {	/* Present in currrent address space */
      tmp_page = MapVirtualPage( plPageDirectory[i] | 3 );
      for( j = 0; j < 1024; j++ ) {
	if( tmp_page[j] ) {	/* Present in current address space */
	  if( (j < 0x200) || (j > 0x20F) || i ) {/* --??-- Don't free
						    temp video memory */
	    BITMAP_SET( mm_table, tmp_page[j] >> 12 ); /* Free frame */
	    tmp_page[j] = 0;
	  }
	}
      }
      UnmapVirtualPage( tmp_page );
      BITMAP_SET( mm_table, plPageDirectory[i] >> 12 );
      plPageDirectory[i] = 0;
    }
  }

  /* Allocate space for new page table */
  plPageDirectory[0] = AllocatePhysicalPage() | 7;
  plPageTable = MapVirtualPage( plPageDirectory[0] );
  memset( plPageTable, 0, 0x1000 );
  
  for( i = 0; i < filesize; i+= 4096 ) {
    frame_ptr[i >> 12] = AllocatePhysicalPage() | 3;
  }

#ifdef DEBUG
  /* Test that mm_table is setup correct */
  for (i = 320; i < 640; i++) 
    putchar (BITMAP_TST(mm_table,i) ? '1' : '0');
#endif

  /* Temporary dir entry for mapping file image into virtual address space */
  plPageDirectory[ (unsigned) pe >> 22 ] = phys_addr;
  
  flush_tlb_all();

  /* Read into virtual address corresponding to plPageDirectory[1021] */
  if ( filesize != ext2fs_read( (void *) pe, filesize ) )
    panic( "File size mismatch on read" );

  pph = (void *) pe + pe->e_phoff;
  pEntry = (void *) pe->e_entry;

  memset( frame_map, 0, 32 * sizeof( unsigned long ) );

  /* Walk ELF header */
  for( i = 0; i < pe->e_phnum; i++ ) {
    if( pph->p_type == PT_LOAD ) {
	if( ( pph->p_offset & 0xFFF ) != ( pph->p_vaddr & 0xFFF ) )
	    panic( "Misalignment in program header" );
	
	/* map pages loaded from file */
	c = ( ( pph->p_offset + pph->p_filesz - 1 ) >> 12 ) -
	    ( pph->p_offset >> 12 ) + 1; /* #pages to load for module */
	
	for( j = 0; j < c; j++ ) {
	    if( j == c - 1 ) {
		/* Page is the last of this header, and needs to be
		   zero-padded, but unfortunately the page may be
		   shared with the next phdr.  We copy it to avoid any
		   conflicts. */
		unsigned frame = AllocatePhysicalPage();
		char *buf = MapVirtualPage( frame | 3 );
		int partial = ( pph->p_offset + pph->p_filesz ) & 0xFFF;
		    
		memcpy( buf, (char *) pe + ( pph->p_offset & ~0xFFF )  +
			( j << 12 ), partial );
		memset( buf + partial, 0, 0x1000 - partial );
		
		UnmapVirtualPage( buf );
		
		plPageTable[ ( (unsigned long) pph->p_vaddr >> 12 ) + j ] =
		    frame | 7;
	    } else {
		BITMAP_SET( frame_map, j + (pph->p_offset >> 12) );
		plPageTable[ ( (unsigned long) pph->p_vaddr >> 12 ) + j ] =
		    frame_ptr[j + (pph->p_offset >> 12)] | 7;
	    }
	}

	/* map additional zeroed pages */
	c = ( ( pph->p_offset + pph->p_memsz - 1 ) >> 12 ) -
	    ( pph->p_offset >> 12 ) + 1; /* page limit to clear for module */

	/* Allocate space for bss section.  Use temporary virtual memory for
	 * memset call to clear physical frame(s)
	 */
	for( ; j < c; j++ ) {
	    unsigned long page_frame = (unsigned long) AllocatePhysicalPage();
	    void *virt_page = MapVirtualPage( page_frame | 3 );
	    plPageTable[ ( (unsigned long) pph->p_vaddr >> 12 ) + j ] =
		page_frame | 7;
	    memset( virt_page, 0, 0x1000 );
	    UnmapVirtualPage( virt_page );
	}
    } 
	
    pph = (void *) pph + pe->e_phentsize;
  }

  /* Deallocate unsued frames for file that were not loaded with contents */
  for( i = 0; i < filesize; i+= 4096 ) {
    if( !BITMAP_TST( frame_map, i >> 12 ) )
      BITMAP_SET( mm_table, frame_ptr[i >> 12] >> 12 );
  }

  /* --??-- temporarily map video memory into exec()ed process */
  for( i = 0; i < 16; i++ )
      plPageTable[ 0x200 + i ] = 0xA0000 | ( i << 12 ) | 7;
  
  /* map stack and clear its contents -- Here, setup 16 pages for stack */
  for( i = 0; i < 16; i++ ) {
    pStack = AllocatePhysicalPage();
    plPageTable[1023-i] = pStack | 7;
    invalidate_page( (void *) ( ( 1023 - i ) << 12 ) );
  }
  memset( (void *)0x3F0000, 0, 0x10000 ); /* Clear 16 page stack */

  plPageDirectory[1021] = 0;
  UnmapVirtualPage( plPageDirectory );
  UnmapVirtualPage( plPageTable );
  UnmapVirtualPage( frame_ptr );
  BITMAP_SET( mm_table, phys_addr >> 12 );

  flush_tlb_all();

  /* Copy command-line arguments to top of new stack */
  memcpy( (void *)( 0x400000 - 80 ), command_args, 80 );
  
  /* Push onto stack argument vector for when we call _start in our "libc"
     library. Here, we work with user-level virtual addresses for when we
     return to user. */
  *( unsigned *) ( 0x400000 - 84 ) = 0;	/* argv[1] -- not used right now */
  *( unsigned *) ( 0x400000 - 88 ) = 0x400000 - 80; /* argv[0] */
  *( unsigned *) ( 0x400000 - 92 ) = 0x400000 - 88; /* argv */
  *( unsigned *) ( 0x400000 - 96 ) = 1;	/* argc -- hard-coded right now */

  /* Dummy return address placed here for the simulated "call" to our
     library */
  *( unsigned *) ( 0x400000 - 100 ) = 0; /* NULL return address -- never
					    used */

  /* Patch up kernel stack with new values so that we can start new program
     on return to user-level  */
  curr_stack[0] = 0x00230023;	/* fs/gs selectors */
  curr_stack[1] = 0x00230023;	/* ds/es selectors */
  curr_stack[2] = (unsigned int) pEntry; /* eip */
  curr_stack[3] = 0x1B;		/* cs selector */
  /* --??-- Temporarily set IOPL 3 in exec()ed program for VGA/keyboard testing */
  curr_stack[4] = F_1 | F_IF | 0x3000; /* EFLAGS */
  curr_stack[5] = 0x400000 - 100; /* -100 after pushing command-line args */
  curr_stack[6] = 0x23;		/* ss selector */

  return 0;
}

char _getchar ( void ) {

  char c;

  static char scancode[128] = "\0\e1234567890-=\177\tqwertyuiop[]\n\0asdfghjkl;'`\0\\zxcvbnm,./\0*\0 \0\0\0\0\0\0\0\0\0\0\0\0\000789-456+1230.\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

  sti();			/* This is so that we can get soundcard
				 * interrupts during initial boot, while the
				 * shell is waiting for keyboard input */

  /* Poll keyboard status register at port 0x64 checking bit 0 to see if
   * output buffer is full. We continue to poll if the msb of port 0x60
   * (data port) is set, as this indicates out-of-band data or a release
   * keystroke
   */
  while( !(inb( 0x64 ) & 0x1) || ( ( c = inb( 0x60 ) ) & 0x80 ) );

  return scancode[ (int)c ];

}


void _switch_to( unsigned pid ) {
  
  jmp_gate( pid );
}


/* --??-- Flags not used for now...a crude open call as it stands  */
int _open( char *pathname, int flags ) {

    return( ext2fs_dir( pathname ) );

}


int _read( char *pathname, void *buf, int count ) {

    return( ext2fs_read( buf, count ) );

}


int _uname( char *name ) {

  /* --??-- Error check in the future */
  memcpy( name, kernel_version, sizeof( kernel_ver ) );
  
  return 0;
}


int _meminfo( void ) {

  int i, j = 0;

  for( i = 0; i < mm_limit; i++ )
    if( BITMAP_TST( mm_table, i ))
      j++;
  
  return j << 12;
}


unsigned _time( void ) {

  return tick;
}


/* IRQ0 system timer interrupt handler: simply updates the system clock
   tick for now */
void _timer( void ) {
  
  tick++;
  
  /* Need to issue an EOI "end of interrupt" to be ready for further
     interrupts */
  send_eoi ();

  runqueue_append( LookupTSS( str() )->priority, str() ); /* add the current task to the back of the run queue */
  schedule(); /* find a task to execute */
}


void __exit( int status ) {

    void *phys_addr;
    unsigned *virt_addr;
    unsigned *tmp_page;
    int i, j;
    unsigned short tss;
    descriptor *ad = (descriptor *) KERN_GDT; 
    unsigned *kern_page_table = (unsigned *) KERN_PGT;
    quest_tss *ptss;
    int waiter;
    
    /* For now, simply free up memory used by calling process address
       space.  We will pass the exit status to the parent process in the
       future. */

    bochs_instr_trace();	/* --??-- */
    bochs_regs_trace();		/* --??-- */

    phys_addr = get_pdbr();
    virt_addr = MapVirtualPage( (unsigned)phys_addr | 3 );

    /* Free user-level virtual address space */
    for( i = 0; i < 1023; i++ ) {
      if( virt_addr[i] ) { 	/* Free page directory entry */
	tmp_page = MapVirtualPage( virt_addr[i] | 3 );
	for( j = 0; j < 1024; j++ ) {
	  if( tmp_page[j] ) { /* Free frame */
	    if( (j < 0x200) || (j > 0x20F) || i ) {/* --??-- Skip releasing
						      video memory */
	      BITMAP_SET( mm_table, tmp_page[j] >> 12 );
	    }
	  }
	}	    
	UnmapVirtualPage( tmp_page );
	BITMAP_SET( mm_table, virt_addr[i] >> 12 );
      }
    }
    BITMAP_SET( mm_table, (unsigned)phys_addr >> 12 ); /* Free up page for page directory */
    UnmapVirtualPage( virt_addr );

    /* --??-- Need to release TSS used by exiting process. Here, we need a way
       to index GDT based on current PID returned from original fork call.

       NOTE: Here' we shouldn't really release the TSS until the parent has
       been able to check the status of the child... */

    tss = str();
    ltr( dummyTSS_selector [LAPIC_get_physical_ID()] );

    /* Remove space for tss -- but first we need to construct the linear
       address of where it is in memory from the TSS descriptor */
    ptss = LookupTSS( tss );

    /* All tasks waiting for us now belong on the runqueue. */
    while( ( waiter = queue_remove_head( &ptss->waitqueue ) ) )
	runqueue_append(  LookupTSS( waiter )->priority, waiter );
    
    BITMAP_SET( mm_table, kern_page_table[ ( (unsigned)ptss >> 12 ) & 0x3FF ] >> 12 );

    /* Remove tss descriptor entry in GDT */
    memset( ad + (tss >> 3), 0, sizeof( descriptor ) );

    UnmapVirtualPage( ptss );

    schedule();
}


extern int _waitpid( int pid ) {

    quest_tss *ptss = LookupTSS( pid );

    if( ptss ) {
	/* Destination task exists.  Add ourselves to the queue of tasks
	   waiting for it. */
	queue_append( &ptss->waitqueue, str() );
	/* We have to go to sleep now -- find another task. */
	schedule();
	/* We have been woken up (see __exit).  Return successfully. */
	return 0;
    } else
	/* Destination task does not exist.  Return an error. */
	return -1;
}


extern int _sched_setparam( int pid, const struct sched_param *p ) {

  quest_tss *ptss = LookupTSS( pid );

  if( ptss ) {
    if( p->sched_priority == -1 )  /* Assume window-constrained task */
      ptss->priority = ( p->k * p->T ) / p->m;
    else
      ptss->priority = p->sched_priority;

    runqueue_append( (LookupTSS( str() ))->priority, str() );
    schedule();
    
    return 0;
    
  } else
    /* Destination task does not exist.  Return an error. */
    return -1;
}
