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
    print("00000000");
    waitpid(pid);
    print("\n");
    _exit(0);
  } else if ((pid=fork())) {
    print("11111111");
    waitpid(pid);
    _exit(0);
  } else if ((pid=fork())) {
    print("22222222");
    waitpid(pid);
    _exit(0);
  } else if ((pid=fork())) {
    print("33333333");
    waitpid(pid);
    _exit(0);
  } else if ((pid=fork())) {
    print("44444444");
    waitpid(pid);
    _exit(0);
  } else {
    print("55555555");
    _exit(0);
  }
}

