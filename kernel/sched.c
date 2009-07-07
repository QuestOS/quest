#include "i386.h"
#include "kernel.h"
#include "smp.h"

unsigned short runqueue[MAX_PRIO_QUEUES];
unsigned short waitqueue[MAX_PRIO_QUEUES]; /* For tasks having expired
					      their current quanta */
static unsigned int runq_bitmap[( MAX_PRIO_QUEUES + 31 ) / 32];

static struct spinlock kernel_lock = SPINLOCK_INIT;
 
extern void queue_append( unsigned short *queue, unsigned short selector ) {

    quest_tss *tssp;

    /* NB: This code assumes atomic execution, and therefore cannot be
       called with interrupts enabled. */

    tssp = LookupTSS( selector );

    tssp->next = 0;

    if( *queue ) {
	for( tssp = LookupTSS( *queue ); tssp->next;
	     tssp = LookupTSS( tssp->next ) )
	    ;

	tssp->next = selector;
    } else
	*queue = selector;
}


extern void runqueue_append( unsigned int prio, unsigned short selector ) {
  queue_append( &runqueue[prio], selector );
  
  BITMAP_SET( runq_bitmap, prio );
}

extern unsigned short queue_remove_head( unsigned short *queue ) {

    quest_tss *tssp;
    unsigned short head;
    
    /* NB: This code assumes atomic execution, and therefore cannot be
       called with interrupts enabled. */

    if( !( head = *queue ) )
	return 0;

    tssp = LookupTSS( head );

    *queue = tssp->next;

    return head;
}

extern void schedule ( void ) {
  spinlock_lock(&kernel_lock);
  locked_schedule();
}

/* Pick from the highest priority non-empty queue */
extern void locked_schedule( void ) {

  unsigned short next;
  unsigned int prio;

  if( ( prio = bitmap_find_first_set( runq_bitmap, MAX_PRIO_QUEUES ) ) != -1) { /* Got a task to execute */
    
    next = queue_remove_head( &runqueue[prio] );
    if( !runqueue[prio] )
      BITMAP_CLR( runq_bitmap, prio );
    
    if( next == str() ) {
      /* no task switch required */
      spinlock_unlock(&kernel_lock);
      return;
    }

#if 0
    com1_puts("CPU "); com1_putx(LAPIC_get_physical_ID());
    com1_puts(" jmp_gate: "); com1_putx(next); 
    {                           /* print runqueue to com1 */
      quest_tss *tssp;
      int sel = runqueue[prio];
      while(sel) {
        tssp = LookupTSS(sel);
        com1_putc(' '); com1_putx(sel);
        sel = tssp->next;
      }
    }
    com1_putc('\n');
#endif

    jmp_gate( next );
    spinlock_unlock(&kernel_lock); 
  }
  else {			/* Replenish timeslices for expired
				   tasks */
    /***************************************
     * com1_putx(LAPIC_get_physical_ID()); *
     * com1_puts(" is bored\n");           *
     ***************************************/

    /* 
     * If a task calls schedule() and is selected from the runqueue,
     * then it must be switched out.  Go to IDLE task if nothing else. 
     */
    unsigned short idle_sel = idleTSS_selector[LAPIC_get_physical_ID()];

    /* Only switch tasks to IDLE if we are not already running IDLE. */
    if(str() != idle_sel) jmp_gate(idle_sel);
    
    spinlock_unlock(&kernel_lock);
  }
}

void lock_kernel(void) {
  spinlock_lock(&kernel_lock);
}

void unlock_kernel(void) {
  spinlock_unlock(&kernel_lock);
}
