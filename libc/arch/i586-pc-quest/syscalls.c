/* note these headers are all provided by newlib - you don't need to provide them */
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/times.h>
#include <sys/errno.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdint.h>
#include <vcpu.h>

#define CLOBBERS1 "memory","cc","%ebx","%ecx","%edx","%esi","%edi"
#define CLOBBERS2 "memory","cc","%ecx","%edx","%esi","%edi"
#define CLOBBERS3 "memory","cc","%ebx","%edx","%esi","%edi"
#define CLOBBERS4 "memory","cc","%ebx","%ecx","%esi","%edi"
#define CLOBBERS5 "memory","cc","%edx","%esi","%edi"
#define CLOBBERS6 "memory","cc","%esi","%edi"
#define CLOBBERS7 "memory","cc","%edi"

void _exit (int) __attribute__ ((noreturn));
void _exit (int status)
{
  asm volatile ("int $0x3a\n"::"a" (status):CLOBBERS1);
  while (1);                    /* Shouldn't get here but stops gcc warning */
}

int close(int file)
{
  int ret;
  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (1), "b" (file), "c" (0), "d" (0), "S" (0), "D" (0)
                :"memory", "cc");
  return ret;

}


/* -- EM -- Need to fix this exec to use env */
int execve(char *name, char **argv, char **env)
{
  asm volatile ("int $0x33\n"::"a" (name), "b" (argv):CLOBBERS2);
}

int _execve(char *name, char **argv, char **env)
{
  execve(name, argv, env);
}

int exec(char *name, char **argv)
{
  execve(name, argv, NULL);
}


int fork()
{
  return vcpu_fork(BEST_EFFORT_VCPU);
}

int vcpu_fork(vcpu_id_t vcpu_id)
{
  int retval;
  asm volatile ("int $0x31\n":"=a" (retval):"a"(vcpu_id):CLOBBERS1);
  return retval;
}



int open(const char *name, int flags, ...)
{
  int c;
  asm volatile ("int $0x35\n":"=a" (c):"a" (name), "b" (flags):CLOBBERS2);
  return c;
}

int write(int file, char *ptr, int len)
{
  int ret;
  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (6), "b" (file), "c" (ptr), "d" (len), "S" (0), "D" (0)
                :"memory", "cc");
  return ret;
}


/* Quest currently has a read syscall but it takes a file path, not a
   file descriptor */
int read(int file, char *ptr, int len)
{
  
  int c;

  asm volatile ("int $0x36\n":"=a" (c):"a" (file), "b" (ptr),
                "c" (len):CLOBBERS5);

  return c;
}

int isatty(int file)
{
  return file < 3;
}


int waitpid(int pid, int *status, int options)
{
  int ret;
  asm volatile ("int $0x3B\n":"=a" (ret):"a" (pid):CLOBBERS1);
  return ret;
}

int gettimeofday (struct timeval *tp, void *tzp)
{
  return get_time (tp);
}

/* The following are no-ops */

/* -- EM -- This is just a minimal implementation mostly to get libc
      to flush stdout and stderr before a read to stdin */
int fstat(int file, struct stat *st)
{
  st->st_nlink = 1;
  st->st_uid = 0;
  st->st_gid = 0;
  st->st_rdev = 0;
  st->st_size = 0;
  st->st_blocks = 0;
  st->st_dev = 0;
  st->st_ino = 0;
    
  if(file < 3) {
    st->st_mode = 8592;
    st->st_blksize = 1024;

  }
  else {
    /* -- EM -- Assume file (not true because it could be a socket) */
    st->st_mode = 35552;
    st->st_blksize = 4096;
  }
  return 0;
}

int getpid()
{
  int pid;
  asm volatile ("int $0x30\n":"=a"(pid):"a" (3L):CLOBBERS1);
  return pid;
}

vcpu_id_t vcpu_create(struct sched_param* sched_param)
{
  vcpu_id_t res;
  asm volatile ("int $0x30\n":"=a"(res):"a" (4L), "b"(sched_param):CLOBBERS2);
  return res;
}

int vcpu_bind_task(vcpu_id_t vcpu_id)
{
  int res;
  asm volatile ("int $0x30\n":"=a"(res):"a" (5L), "b"(vcpu_id):CLOBBERS2);
  return res;
}

int vcpu_destroy(vcpu_id_t vcpu_id, uint force)
{
  int res;
  asm volatile ("int $0x30\n":"=a"(res):"a" (6L), "b"(vcpu_id), "c"(force):CLOBBERS5);
  return res;
}

int vcpu_getparams(struct sched_param* sched_param)
{
  int res;
  asm volatile ("int $0x30\n":"=a"(res):"a" (7L), "b"(sched_param):CLOBBERS2);
  return res;
}

int vcpu_setparams(vcpu_id_t vcpu_id, struct sched_param* sched_param)
{
  int res;
  asm volatile ("int $0x30\n":"=a"(res):"a" (8L), "b"(vcpu_id), "c"(sched_param):CLOBBERS5);
  return res;
}

int kill(int pid, int sig)
{
  write(1, "In kill which is a no op\n", sizeof("In kill which is a no op\n"));
  while(1);
  errno = ENOSYS;
  return -1;
}

int link(char *old, char *new)
{
  write(1, "In link which is a no op\n", sizeof("In link which is a no op\n"));
  while(1);
  errno = ENOSYS;
  return -1;
}

int lseek(int file, int ptr, int dir)
{
  write(1, "In lseek which is a no op\n", sizeof("In lseek which is a no op\n"));
  while(1);
  errno = ENOSYS;
  return -1;
}

#define HEAPSIZE (0x100000)

unsigned char _heap[HEAPSIZE];

/* Need to implement an sbrk syscall */
caddr_t sbrk(int incr)
{
   static unsigned char *heap_end;
   unsigned char *prev_heap_end;

   /* initialize */
   if( heap_end == 0 ) heap_end = _heap;

   prev_heap_end = heap_end;

   if( heap_end + incr - _heap > HEAPSIZE ) {
      /* heap overflow—announce on stderr */
      write( 2, "Heap overflow!\n", 15 );
      abort();
   }

   heap_end += incr;

   return (caddr_t) prev_heap_end;
}

int stat(const char *file, struct stat *st)
{
  write(1, "In stat which is a no op\n", sizeof("In stat which is a no op\n"));
  while(1);
  errno = ENOSYS;
  return -1;
}

clock_t times(struct tms *buf)
{
  write(1, "In times which is a no op\n", sizeof("In times which is a no op\n"));
  while(1);
  errno = ENOSYS;
  return -1;
}

int unlink(char *name)
{
  write(1, "In unlink which is a no op\n", sizeof("In unlink which is a no op\n"));

  while(1);
  errno = ENOSYS;
  return -1;
}

int wait(int *status)
{
  write(1, "In wait which is a no op\n", sizeof("In wait which is a no op\n"));
  while(1);
  errno = ENOSYS;
  return -1;
}




/* The following are not required by newlib (as far as I know) */

inline void
usleep (unsigned usec)
{

  asm volatile ("int $0x30\n"::"a" (1L), "b" (usec):CLOBBERS2);

}

inline int
usb_syscall(int device_id, int operation, void* buf, int data_len)
{
  int ret;
  asm volatile ("int $0x30\n":"=a" (ret) : "a" (2L), "b"(device_id), "c" (operation),
                "d" (buf), "S" (data_len) : CLOBBERS7);
  return ret;
}

inline int
get_time (void *tp)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (10), "b" (tp), "c" (0), "d" (0), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline void
switch_to (unsigned pid)
{

  asm volatile ("int $0x32\n"::"a" (pid):CLOBBERS1);

}

inline unsigned int
getcode (void)
{

  unsigned int c;

  asm volatile ("int $0x34\n":"=a" (c): "b" (1):CLOBBERS2);

  return c;
}


inline int
uname (char *name)
{
  int c;
  asm volatile ("int $0x37\n":"=a" (c):"a" (name):CLOBBERS1);
  return c;
}


inline unsigned
meminfo (void)
{

  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (0L):CLOBBERS1);

  return c;
}

inline unsigned
shared_mem_alloc (void)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (1L):CLOBBERS1);

  return c;
}

inline void *
shared_mem_attach (unsigned id)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (2L), "d" (id):CLOBBERS4);

  return (void *) c;
}

inline unsigned
shared_mem_detach (void *addr)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (3L), "d" ((unsigned) addr):CLOBBERS4);

  return c;
}

inline unsigned
shared_mem_free (unsigned id)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (4L), "d" (id):CLOBBERS4);

  return c;
}


inline int
sched_setparam (int pid, const struct sched_param *p)
{

  int ret;

  asm volatile ("int $0x3C\n":"=a" (ret):"a" (pid), "b" (p):CLOBBERS2);

  return ret;
}

inline int
open_socket (int domain, int type, int protocol)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (0), "b" (domain), "c" (type), "d" (protocol), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}


inline int
socket_bind (int sockfd, uint32_t addr, uint16_t port)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (2), "b" (sockfd), "c" (addr), "d" (port), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline int
socket_connect (int sockfd, uint32_t addr, uint16_t port)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (3), "b" (sockfd), "c" (addr), "d" (port), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline int
socket_listen (int sockfd, int backlog)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (4), "b" (sockfd), "c" (backlog), "d" (0), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline int
socket_accept (int sockfd, void *addr, void *len)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (5), "b" (sockfd), "c" (addr), "d" (len), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline ssize_t
socket_sendto (int sockfd, const void *buf, size_t nbytes, uint32_t destaddr, uint16_t port)
{
  ssize_t ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (7), "b" (sockfd), "c" (buf), "d" (nbytes), "S" (destaddr), "D" (port)
                :"memory", "cc");

  return ret;
}

inline ssize_t
socket_recv (int sockfd, void *buf, size_t nbytes, void *addr, void *addrlen)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (8), "b" (sockfd), "c" (buf), "d" (nbytes), "S" (addr), "D" (addrlen)
                :"memory", "cc");

  return ret;
}

inline int
socket_select (int maxfdp1, fd_set * readfds, fd_set * writefds,
               fd_set * exceptfds, struct timeval * tvptr)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (9), "b" (maxfdp1), "c" (readfds), "d" (writefds), "S" (exceptfds), "D" (tvptr)
                :"memory", "cc");

  return ret;
}

inline int
socket_get_sb_id ()
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (11), "b" (0), "c" (0), "d" (0), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline int
socket_getsockname (int sockfd, void *addr, void *len)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (12), "b" (sockfd), "c" (addr), "d" (len), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline int
socket_recovery (int arg)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (13), "b" (arg), "c" (0), "d" (0), "S" (0), "D" (0)
                :"memory", "cc");

  return ret;
}

inline int
switch_screen (int dir)
{
  int ret;

  asm volatile ("int $0x3F\n":"=a" (ret):"a" (dir):CLOBBERS1);
  
  return ret;
}
