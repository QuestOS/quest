#include "i386.h"
#include "kernel.h"

unsigned short runqueue[MAX_PRIO_QUEUES];
unsigned short waitqueue[MAX_PRIO_QUEUES]; /* For tasks having expired
					      their current quanta */
static unsigned int runq_bitmap[( MAX_PRIO_QUEUES + 31 ) / 32];
 
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


/* Pick from the highest priority non-empty queue */
extern void schedule( void ) {

  unsigned short next;
  unsigned int prio;
    
  if( ( prio = bitmap_find_first_set( runq_bitmap, MAX_PRIO_QUEUES ) ) != -1) { /* Got a task to execute */
    
    next = queue_remove_head( &runqueue[prio] );
    if( !runqueue[prio] )
      BITMAP_CLR( runq_bitmap, prio );
    
    if( next == str() )
      /* no task switch required */
      return;
      
    jmp_gate( next );
  }
  else {			/* Replenish timeslices for expired
				   tasks */
  }
}
