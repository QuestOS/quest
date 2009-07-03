

struct sched_param {
  int sched_priority;

  /* Below are paramters used for window-constrained scheduling */
  int C;			/* service quantum */
  int T;			/* period */
  int m;			/* mandatory instance count in a window */
  int k;			/* window of requests  */
};


/* Syscall 0 used as a test syscall 
 *
 * Simply passes character arguments to the kernel for use in 
 * writing to video RAM
 */
static inline void putchar (int c) {

  asm volatile(
       "movl $0, %%eax\n"
       "int $0x30\n" : : "b" (c) );

}


static inline int fork ( void ) {

  int retval;

  asm volatile( "int $0x31\n" : "=a" (retval) );

  return retval;
}


static inline void switch_to ( unsigned pid ) {

  asm volatile( "int $0x32\n" : : "a" (pid) );

}


static inline void exec( char *file, char *argv[] ) {

  asm volatile( "int $0x33\n" : : "a" (file), "b" (argv) );
}


static inline char getchar( void ) {

  char c;

  asm volatile( "int $0x34\n" : "=a" (c) : );

  return c;
}

static inline int open( const char *pathname, int flags ) {

  int c;

  asm volatile( "int $0x35\n" : "=a" (c) : "a" (pathname), "b" (flags) );

  return c;
}

static inline int read( char *pathname, void *buf, int count ) {

  int c;

  asm volatile( "int $0x36\n" : "=a" (c) : "a" (pathname), "b" (buf), "c" (count) );

  return c;
}


static inline int uname( char *name ) {

  int c;

  asm volatile( "int $0x37\n" : "=a" (c) : "a" (name) );

  return c;
}


static inline unsigned meminfo( void ) {

  unsigned c;

  asm volatile( "int $0x38\n" : "=a" (c) : "a" (0) );

  return c;
}

static inline unsigned shared_mem_alloc( void ) {
  unsigned c;

  asm volatile( "int $0x38\n" : "=a" (c) : "a" (1) );

  return c;
}

static inline void *shared_mem_attach( unsigned id ) {
  unsigned c;

  asm volatile( "int $0x38\n" : "=a" (c) : "a" (2), "d" (id) );

  return (void *)c;
}

static inline unsigned shared_mem_detach( void *addr ) {
  unsigned c;

  asm volatile( "int $0x38\n" : "=a" (c) : "a" (3), "d" ((unsigned)addr) );

  return c;
}

static inline unsigned shared_mem_free( unsigned id ) {
  unsigned c;

  asm volatile( "int $0x38\n" : "=a" (c) : "a" (4), "d" (id) );

  return c;
}



static inline unsigned time( void ) {

  unsigned c;

  asm volatile( "int $0x39\n" : "=a" (c) : );

  return c;
}


static inline __attribute__((noreturn)) void _exit( int status ) {

  asm volatile( "int $0x3a\n" : : "a" (status) );

  while( 1 );			/* Shouldn't get here but stops gcc warning */
}

static inline int waitpid( int pid ) {

    int ret;
    
    asm volatile( "int $0x3B\n" : "=a" (ret) : "a" (pid) );

    return ret;
}


static inline int sched_setparam( int pid, const struct sched_param *p ) {

    int ret;
    
    asm volatile( "int $0x3C\n" : "=a" (ret) : "a" (pid), "b" (p) );

    return ret;
}


