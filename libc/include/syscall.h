/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */



struct sched_param
{
  int sched_priority;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
};

#define CLOBBERS1 "memory","cc","%ebx","%ecx","%edx","%esi","%edi"
#define CLOBBERS2 "memory","cc","%ecx","%edx","%esi","%edi"
#define CLOBBERS3 "memory","cc","%ebx","%edx","%esi","%edi"
#define CLOBBERS4 "memory","cc","%ebx","%ecx","%esi","%edi"
#define CLOBBERS5 "memory","cc","%edx","%esi","%edi"


/* Syscall 0 used as a test syscall 
 *
 * Simply passes character arguments to the kernel for use in 
 * writing to video RAM
 */

static inline void
putchar (int c)
{

  asm volatile ("int $0x30\n"::"a" (0L), "b" (c):CLOBBERS2);

}

static inline void
usleep (unsigned usec)
{

  asm volatile ("int $0x30\n"::"a" (1L), "b" (usec):CLOBBERS2);

}

static inline unsigned short
fork (void)
{

  unsigned int retval;

  asm volatile ("int $0x31\n":"=a" (retval)::CLOBBERS1);

  return (unsigned short) retval;
}


static inline void
switch_to (unsigned pid)
{

  asm volatile ("int $0x32\n"::"a" (pid):CLOBBERS1);

}


static inline void
exec (char *file, char *argv[])
{

  asm volatile ("int $0x33\n"::"a" (file), "b" (argv):CLOBBERS2);
}


static inline char
getchar (void)
{

  char c;

  asm volatile ("int $0x34\n":"=a" (c): "b" (0):CLOBBERS2);

  return c;
}

static inline unsigned int
getcode (void)
{

  unsigned int c;

  asm volatile ("int $0x34\n":"=a" (c): "b" (1):CLOBBERS2);

  return c;
}

static inline int
open (const char *pathname, int flags)
{

  int c;

  asm volatile ("int $0x35\n":"=a" (c):"a" (pathname), "b" (flags):CLOBBERS2);

  return c;
}

static inline int
read (char *pathname, void *buf, int count)
{

  int c;

  asm volatile ("int $0x36\n":"=a" (c):"a" (pathname), "b" (buf),
                "c" (count):CLOBBERS5);

  return c;
}


static inline int
uname (char *name)
{

  int c;

  asm volatile ("int $0x37\n":"=a" (c):"a" (name):CLOBBERS1);

  return c;
}


static inline unsigned
meminfo (void)
{

  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (0L):CLOBBERS1);

  return c;
}

static inline unsigned
shared_mem_alloc (void)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (1L):CLOBBERS1);

  return c;
}

static inline void *
shared_mem_attach (unsigned id)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (2L), "d" (id):CLOBBERS4);

  return (void *) c;
}

static inline unsigned
shared_mem_detach (void *addr)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (3L), "d" ((unsigned) addr):CLOBBERS4);

  return c;
}

static inline unsigned
shared_mem_free (unsigned id)
{
  unsigned c;

  asm volatile ("int $0x38\n":"=a" (c):"a" (4L), "d" (id):CLOBBERS4);

  return c;
}



static inline unsigned
time (void)
{

  unsigned c;

  asm volatile ("int $0x39\n":"=a" (c):);

  return c;
}


static inline void _exit (int) __attribute__ ((noreturn));
static inline void
_exit (int status)
{

  asm volatile ("int $0x3a\n"::"a" (status):CLOBBERS1);

  while (1);                    /* Shouldn't get here but stops gcc warning */
}

static inline int
waitpid (int pid)
{

  int ret;

  asm volatile ("int $0x3B\n":"=a" (ret):"a" (pid):CLOBBERS1);

  return ret;
}


static inline int
sched_setparam (int pid, const struct sched_param *p)
{

  int ret;

  asm volatile ("int $0x3C\n":"=a" (ret):"a" (pid), "b" (p):CLOBBERS2);

  return ret;
}

#ifdef USE_VMX
static inline int
switch_screen (int dir)
{
  int ret;

  asm volatile ("int $0x3F\n":"=a" (ret):"a" (dir):CLOBBERS1);
  
  return ret;
}
#endif

/* 
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End: 
 */

/* vi: set et sw=2 sts=2: */
