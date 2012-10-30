/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
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

#include "syscall.h"
#include "errno.h"

#define CLOBBERS1 "memory","cc","%ebx","%ecx","%edx","%esi","%edi"
#define CLOBBERS2 "memory","cc","%ecx","%edx","%esi","%edi"
#define CLOBBERS3 "memory","cc","%ebx","%edx","%esi","%edi"
#define CLOBBERS4 "memory","cc","%ebx","%ecx","%esi","%edi"
#define CLOBBERS5 "memory","cc","%edx","%esi","%edi"
#define CLOBBERS6 "memory","cc","%esi","%edi"
#define CLOBBERS7 "memory","cc","%edi"

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


/*
inline unsigned
time (void)
{

  unsigned c;

  asm volatile ("int $0x39\n":"=a" (c):);

  return c;
}
*/


inline int
waitpid (int pid)
{

  int ret;

  asm volatile ("int $0x3B\n":"=a" (ret):"a" (pid):CLOBBERS1);

  return ret;
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
close (int filedes)
{
  int ret;

  asm volatile ("int $0x3D\n"
                :"=a" (ret)
                :"a" (1), "b" (filedes), "c" (0), "d" (0), "S" (0), "D" (0)
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

#ifdef USE_VMX
inline int
switch_screen (int dir)
{
  int ret;

  asm volatile ("int $0x3F\n":"=a" (ret):"a" (dir):CLOBBERS1);
  
  return ret;
}
#endif
