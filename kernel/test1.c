#include "syscall.h"

void putx( unsigned long l ) {

  int i, li;

  for( i = 7; i >= 0; i-- )
    if( ( li = ( l >> ( i << 2 ) ) & 0x0F ) > 9 )
      putchar( 'A' + li - 0x0A );
    else
      putchar( '0' + li );
}

void print(char *s) {
  while (*s) {
    putchar(*s++);
  }
}

void _start() {
  int pid;
  if ((pid=fork ())) {
    print("parent\n");
    waitpid(pid);
    _exit(0);
  } else {
    print("child\n");
    _exit(0);
  }
}

