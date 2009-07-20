#include "i386.h"
#include "kernel.h"
#include "smp.h"
#include "printf.h"

//#define DEBUG_SCHED

unsigned short runqueue[MAX_PRIO_QUEUES];
unsigned short waitqueue[MAX_PRIO_QUEUES]; /* For tasks having expired
					      their current quanta */
static unsigned int runq_bitmap[( MAX_PRIO_QUEUES + 31 ) / 32];

static struct spinlock kernel_lock = SPINLOCK_INIT;
 
extern void queue_append( unsigned short *queue, unsigned short selector ) {

  quest_tss *tssp;

  /* NB: This code assumes atomic execution, and therefore cannot be
     called with interrupts enabled. */

  if( *queue ) {
    if(*queue == selector) return; /* already on queue */

    for( tssp = LookupTSS( *queue ); tssp->next;
         tssp = LookupTSS( tssp->next ) )
      if(tssp->next == selector)
        /* already on queue */
        return;

    /* add to end of queue */
    tssp->next = selector;

  } else
    *queue = selector;

  tssp = LookupTSS( selector );
  tssp->next = 0;

}


extern void runqueue_append( unsigned int prio, unsigned short selector ) {
#ifdef DEBUG_SCHED
  com1_printf("runqueue_append(%x, %x)\n", prio, selector);
#endif
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

extern void wakeup(unsigned short selector) {
  quest_tss *tssp;
  tssp = LookupTSS(selector);
  runqueue_append(tssp->priority, selector);
}

extern void wakeup_list(unsigned short selector) {
  quest_tss *tssp;

  while(selector) {
    tssp = LookupTSS(selector);
    runqueue_append(tssp->priority, selector);
    selector = tssp->next;    
  }
}

/* Pick from the highest priority non-empty queue */
extern void schedule( void ) {

  unsigned short next;
  unsigned int prio;

  if( ( prio = bitmap_find_first_set( runq_bitmap, MAX_PRIO_QUEUES ) ) != -1) { /* Got a task to execute */
    
    next = queue_remove_head( &runqueue[prio] );
    if( !runqueue[prio] )
      BITMAP_CLR( runq_bitmap, prio );

#ifdef DEBUG_SCHED
    com1_printf("CPU %x: switching to task: %x runqueue(%x):", 
                LAPIC_get_physical_ID(), next, prio);
    {                           /* print runqueue to com1 */
      quest_tss *tssp;
      int sel = runqueue[prio];
      while(sel) {
        tssp = LookupTSS(sel);
        com1_printf(" %x", sel);
        sel = tssp->next;
      }
    }
    com1_putc('\n');
#endif
    
    if( next == str() ) {
      /* no task switch required */
      return;
    }



    jmp_gate( next );
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
    BYTE phys_id = LAPIC_get_physical_ID();
    unsigned short idle_sel = idleTSS_selector[phys_id];
    
#ifdef DEBUG_SCHED
    com1_printf("CPU %x: idling\n", phys_id);
#endif


    /* Only switch tasks to IDLE if we are not already running IDLE. */
    if(str() != idle_sel) jmp_gate(idle_sel);
  }
}

void lock_kernel(void) {
  spinlock_lock(&kernel_lock);
}

void unlock_kernel(void) {
  spinlock_unlock(&kernel_lock);
}
