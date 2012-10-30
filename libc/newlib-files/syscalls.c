/* note these headers are all provided by newlib - you don't need to provide them */
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/times.h>
#include <sys/errno.h>
#include <sys/time.h>
#include <stdio.h>


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


int fork()
{
  unsigned int retval;
  asm volatile ("int $0x31\n":"=a" (retval)::CLOBBERS1);
  return (int) retval;
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
  errno = ENOSYS;
  return -1;
}

/* The following are no-ops */

int fstat(int file, struct stat *st)
{
  errno = ENOSYS;
  return -1;
}

int getpid()
{
  errno = ENOSYS;
  return -1;
}

int isatty(int file)
{
  errno = ENOSYS;
  return 0;
}

int kill(int pid, int sig)
{
  errno = ENOSYS;
  return -1;
}

int link(char *old, char *new)
{
  errno = ENOSYS;
  return -1;
}

int lseek(int file, int ptr, int dir)
{
  errno = ENOSYS;
  return -1;
}

/* Need to implement an sbrk syscall */
caddr_t sbrk(int incr)
{
  return -1;
}

int stat(const char *file, struct stat *st)
{
  errno = ENOSYS;
  return -1;
}

clock_t times(struct tms *buf)
{
  errno = ENOSYS;
  return -1;
}

int unlink(char *name)
{
  errno = ENOSYS;
  return -1;
}

int wait(int *status)
{
  errno = ENOSYS;
  return -1;
}
