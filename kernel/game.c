#include "syscall.h"

static inline void outb( uint8 uch, uint16 usPort ) {

    asm volatile( "outb %0,%1" : : "a" (uch), "Nd" (usPort) );
}

extern void _start( void ) {

    int i;
    
    /* misc output write register: colour, enable memory, sync polarity=480 */
    outb( 0xE3, 0x3C2 );

    /* sequencer: disable reset */
    outb( 0x00, 0x3C4 );
    outb( 0x03, 0x3C5 );
    
    /* sequencer: clock mode = 8 pixels/char */
    outb( 0x01, 0x3C4 );
    outb( 0x01, 0x3C5 );

    outb( 0x02, 0x3C4 );
    outb( 0x0F, 0x3C5 );

    outb( 0x03, 0x3C4 );
    outb( 0x00, 0x3C5 );

    /* sequencer: memory mode = extended */
    outb( 0x04, 0x3C4 );
    outb( 0x06, 0x3C5 );

    /* attribute controller: mode = graphics */
    outb( 0x10, 0x3C0 );
    outb( 0x01, 0x3C0 );

    outb( 0x11, 0x3C0 );
    outb( 0x01, 0x3C0 );

    outb( 0x12, 0x3C0 );
    outb( 0x0F, 0x3C0 );

    outb( 0x13, 0x3C0 );
    outb( 0x03, 0x3C0 );

    outb( 0x34, 0x3C0 );
    outb( 0x00, 0x3C0 );

    /* graphics controller */
    outb( 0x00, 0x3CE );
    outb( 0x00, 0x3CF );

    outb( 0x01, 0x3CE );
    outb( 0x00, 0x3CF );

    outb( 0x02, 0x3CE );
    outb( 0x00, 0x3CF );

    outb( 0x03, 0x3CE );
    outb( 0x00, 0x3CF );

    outb( 0x04, 0x3CE );
    outb( 0x00, 0x3CF );

    outb( 0x05, 0x3CE );
    outb( 0x00, 0x3CF );

    outb( 0x06, 0x3CE );
    outb( 0x05, 0x3CF );

    outb( 0x07, 0x3CE );
    outb( 0x0F, 0x3CF );

    outb( 0x08, 0x3CE );
    outb( 0xFF, 0x3CF );

    /* CRT controller: horizontal total = 95 (800 pixels) */
    outb( 0x00, 0x3D4 );
    outb( 95, 0x3D5 );
    
    /* CRT controller: horizontal display = 79 (640 pixels) */
    outb( 0x01, 0x3D4 );
    outb( 79, 0x3D5 );
    
    /* CRT controller: horizontal blank start = 79 (640 pixels) */
    outb( 0x02, 0x3D4 );
    outb( 80, 0x3D5 );

    /* CRT controller: horizontal blank end = 2 */
    outb( 0x03, 0x3D4 );
    outb( 0x82, 0x3D5 );

    /* CRT controller: horizontal sync start = 82 */
    outb( 0x04, 0x3D4 );
    outb( 84, 0x3D5 );

    /* CRT controller: horizontal sync end = 2 */
    outb( 0x05, 0x3D4 );
    outb( 0x80, 0x3D5 );

    /* CRT controller: vertical total = 522 (524 lines) */
    outb( 0x06, 0x3D4 );
    outb( 0x0B, 0x3D5 );

    outb( 0x07, 0x3D4 );
    outb( 0x3E, 0x3D5 );

    /* CRT controller: overflow */
    outb( 0x08, 0x3D4 );
    outb( 0x00, 0x3D5 );

    outb( 0x09, 0x3D4 );
    outb( 0x40, 0x3D5 );

    outb( 0x0A, 0x3D4 );
    outb( 0x00, 0x3D5 );

    outb( 0x0B, 0x3D4 );
    outb( 0x00, 0x3D5 );

    outb( 0x0C, 0x3D4 );
    outb( 0x00, 0x3D5 );

    outb( 0x0D, 0x3D4 );
    outb( 0x00, 0x3D5 );

    outb( 0x0E, 0x3D4 );
    outb( 0x01, 0x3D5 );

    outb( 0x0F, 0x3D4 );
    outb( 0x0E, 0x3D5 );

    /* CRT controller: vertical retrace start = 490 */
    outb( 0x10, 0x3D4 );
    outb( 0xEA, 0x3D5 );

    /* CRT controller: vertical retrace end = 492 */
    outb( 0x11, 0x3D4 );
    outb( 0x8C, 0x3D5 );

    /* CRT controller: vertical display end = 479 (480 lines) */
    outb( 0x12, 0x3D4 );
    outb( 0xDF, 0x3D5 );

    outb( 0x13, 0x3D4 );
    outb( 0x28, 0x3D5 );

    outb( 0x14, 0x3D4 );
    outb( 0x00, 0x3D5 );

    /* CRT controller: vertical blanking start = 479 (480 lines) */
    outb( 0x15, 0x3D4 );
    outb( 0xE7, 0x3D5 );

    /* CRT controller: vertical blanking end = 523 */
    outb( 0x16, 0x3D4 );
    outb( 0x04, 0x3D5 );

    /* CRT controller: mode control = enable retrace, byte, wrap */
    outb( 0x17, 0x3D4 );
    outb( 0xE3, 0x3D5 );

    outb( 0x18, 0x3D4 );
    outb( 0xFF, 0x3D5 );

    outb( 0x02, 0x3C4 );
    outb( 0x0F, 0x3C5 );

    /* Setup graphics mode */
    outb( 0x05, 0x3CE );
    outb( 0x02, 0x3CF );

    /* write to screen */
    for( i = 0; i < 0x10000; i++ )
	( (char *) 0x200000 )[ i ] = 0x55;
    
    for(;;);
}
