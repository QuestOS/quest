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

#ifndef _SYSCALL_H_
#define _SYSCALL_H_


#include "types.h"
#include "sys/socket.h"
#include "sys/select.h"
#include "netinet/in.h"
#include "stat.h"


struct sched_param
{
  int sched_priority;

  /* Below are paramters used for window-constrained scheduling */
  int C;                        /* service quantum */
  int T;                        /* period */
  int m;                        /* mandatory instance count in a window */
  int k;                        /* window of requests  */
  int affinity;                 /* CPU (or Quest-V sandbox) affinity */
};



/* Syscall 0 used as a test syscall 
 *
 * Simply passes character arguments to the kernel for use in 
 * writing to video RAM
 */

inline void putchar (int c);
inline void usleep (unsigned usec);
inline int usb_syscall(int device_id, int operation, void* buf, int data_len);


inline unsigned short fork (void);
inline int get_time (void *tp);


inline void switch_to (unsigned pid);

inline void exec (char *file, char *argv[]);

inline char getchar (void);

inline unsigned int getcode (void);

inline int open (const char *pathname, int flags);

inline int read (char *pathname, void *buf, int count);

inline int uname (char *name);

inline unsigned meminfo (void);

inline unsigned shared_mem_alloc (void);

inline void * shared_mem_attach (unsigned id);

inline unsigned shared_mem_detach (void *addr);

inline unsigned shared_mem_free (unsigned id);

inline unsigned time (void);

inline void _exit (int) __attribute__ ((noreturn));
inline void _exit (int status);


inline int waitpid (int pid);


inline int sched_setparam (int pid, const struct sched_param *p);

inline int open_socket (int domain, int type, int protocol);

inline int close (int filedes);

inline int socket_bind (int sockfd, uint32_t addr, uint16_t port);

inline int socket_connect (int sockfd, uint32_t addr, uint16_t port);

inline int socket_listen (int sockfd, int backlog);

inline int socket_accept (int sockfd, void *addr, void *len);

inline ssize_t write (int fd, const void *buf, size_t nbytes);

inline ssize_t socket_sendto (int sockfd, const void *buf, size_t nbytes,
                              uint32_t destaddr, uint16_t port);

inline ssize_t socket_recv (int sockfd, void *buf, size_t nbytes, void *addr,
                            void *addrlen);

inline int socket_select (int maxfdp1, fd_set * readfds, fd_set * writefds,
               fd_set * exceptfds, struct timeval * tvptr);

inline int socket_get_sb_id ();

inline int socket_getsockname (int sockfd, void *addr, void *len);

inline int socket_recovery (int arg);

#ifdef USE_VMX
  inline int switch_screen (int dir);
#endif

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
