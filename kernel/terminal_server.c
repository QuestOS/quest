#include <stdio.h>
#include <string.h>

static char *pchVideo = (char *) 0x000200000; /* in middle of page table */

static inline unsigned char inb( unsigned short usPort ) {

    unsigned char uch;
    
    asm volatile( "inb %1,%0" : "=a" (uch) : "Nd" (usPort) );
    return uch;
}


static inline void outb( unsigned char uch, unsigned short usPort ) {

    asm volatile( "outb %0,%1" : : "a" (uch), "Nd" (usPort) );
}


static int _putchar( int ch, int attribute ) {

    static int x, y;

    if( ch == '\n' ) {
	x = 0;
	y++;

	if( y > 24 ) {
	  memcpy( pchVideo, pchVideo + 160, 24 * 160 );
	  memset( pchVideo + ( 24 * 160 ), 0, 160 );
	  y = 24;
	}
	return (int) (unsigned char) ch;
    }

    pchVideo[ y * 160 + x * 2 ] = ch;
    pchVideo[ y * 160 + x * 2 + 1 ] = attribute; 
    x++;

    pchVideo[ y * 160 + x * 2 ] = ' ';
    pchVideo[ y * 160 + x * 2 + 1 ] = attribute; 

    /* Move cursor */
    outb( 0x0E, 0x3D4 );		 /* CRTC Cursor location high index */
    outb( ( y * 80 + x ) >> 8, 0x3D5 );  /* CRTC Cursor location high data */
    outb( 0x0F, 0x3D4 );		 /* CRTC Cursor location low index */
    outb( ( y * 80 + x ) & 0xFF, 0x3D5 );/* CRTC Cursor location low data */

    return (int) (unsigned char) ch;
}


void splash_screen( void ) {

  char line1[80];
  char line2[80];
  char *p;
  int i;

  uname( line2 );

  sprintf( line1, "**** Quest kernel version: %s *****   //---\\ \\\\  \\ //-- //--\\ \\\\---\\ \n", line2 );
  for( p=line1; *p;p++ )
    _putchar( *p, 4 );

  sprintf( line1, "* Copyright Boston University, 2005 *   ||   | ||  | ||-- \\\\--\\   || \n" );
  for( p=line1; *p;p++ )
    _putchar( *p, 4 );

  sprintf( line1, "%u", meminfo() );
  i = strlen( line1 );

  for( i = strlen( line1 ); i < 16; i++ )
      line2[ i - strlen( line1 ) ] = '*';

  sprintf( line1, "******** %u bytes free %s   \\\\__\\_  \\\\_/ \\\\__ \\\\__/   || \n", meminfo(), line2 );
  for( p=line1; *p;p++ )
    _putchar( *p, 4 );
}


int main () {

  int arg;

  splash_screen();

  /* Setup cursor to be a full block in foreground colour */
  outb( 0x0A, 0x3D4 );		/* CRTC Cursor start index */
  outb( 0x00, 0x3D5 );		/* CRTC Cursor start data */
  outb( 0x0B, 0x3D4 );		/* CRTC Cursor end index */
  outb( 0x0E, 0x3D5 );		/* CRTC Cursor end data */

  while (1) {
    asm volatile( "movl %%ebx, %0" : "=m" (arg) );

    _putchar( arg, 7 );

    asm volatile( "iret" );
  }

  return 0;

}
