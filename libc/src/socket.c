/*                    The Quest Operating System
 *  Portions Copyright (C) 2005-2010  Richard West, Boston University
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "syscall.h"

int
socket (int domain, int type, int protocol)
{
  int sockfd = open_socket (domain, type, protocol);
  return sockfd;
}

int
shutdown (int sockfd, int how)
{
  int val;

  if (how == SHUT_RDWR) {
    return close (sockfd);
  } else {
    return 0;
  }
}

int
bind (int sockfd, const struct sockaddr *addr, socklen_t len)
{
  struct sockaddr_in * addr_in = (struct sockaddr_in *) addr;
  if (len < sizeof(struct sockaddr_in)) return -1;
  return socket_bind (sockfd, (uint32_t)(addr_in->sin_addr.s_addr),
                      (uint16_t)(addr_in->sin_port));
}

int
connect (int sockfd, const struct sockaddr *addr, socklen_t len)
{
  struct sockaddr_in * addr_in = (struct sockaddr_in *) addr;
  if (len < sizeof(struct sockaddr_in)) return -1;
  return socket_connect (sockfd, (uint32_t)(addr_in->sin_addr.s_addr),
                         (uint16_t)(addr_in->sin_port));
}

int
listen (int sockfd, int backlog)
{
  return socket_listen (sockfd, backlog);
}

int
accept (int sockfd, struct sockaddr *addr, socklen_t *len)
{
  int new_sockfd = 0;
  if (!addr)
    new_sockfd = socket_accept (sockfd, (void *) 0, (void *) 0);
  else
    new_sockfd = socket_accept (sockfd, addr, len);

  return new_sockfd;
}

ssize_t
send (int sockfd, const void *buf, size_t nbytes, int flags)
{
  /* Ignore flag. Only blocking write is supported. */
  return write (sockfd, buf, nbytes);
}

ssize_t
sendto (int sockfd, const void *buf, size_t nbytes, int flags,
        const struct sockaddr *destaddr, socklen_t destlen)
{
  /* Ignore flag. Only blocking write is supported. */
  if (destaddr == 0) {
    /* Assume sockfd is already connected */
    return write (sockfd, buf, nbytes);
  } else {
    struct sockaddr_in * addr_in = (struct sockaddr_in *) destaddr;
    if (destlen < sizeof(struct sockaddr_in)) return -1;
    return socket_sendto (sockfd, buf, nbytes, addr_in->sin_addr.s_addr, addr_in->sin_port);
  }
}

ssize_t
recv (int sockfd, void *buf, size_t nbytes, int flags)
{
  /* Ignore flag. Only blocking write is supported. */
  return socket_recv (sockfd, buf, nbytes, (void *) 0, (void *) 0);
}

ssize_t
recvfrom (int sockfd, void *buf, size_t len, int flags,
          struct sockaddr *addr, socklen_t *addrlen)
{
  return socket_recv (sockfd, buf, len, (void *) addr, (void *) addrlen);
}

int
setsockopt (int sockfd, int level, int option, const void *val, socklen_t len)
{
  return 0;
}

int
getsockopt (int sockfd, int level, int option, void *val, socklen_t *lenp)
{
  return 0;
}

int
getsockname (int sockfd, struct sockaddr *addr, socklen_t *alenp)
{
  return 0;
}

int
getaddrinfo (const char *host, const char *service,
             const struct addrinfo *hint, struct addrinfo **res)
{
  return 0;
}

void
freeaddrinfo (struct addrinfo *ai)
{
  return;
}

int
gettimeofday (struct timeval *tp, void *tzp)
{
  return get_time (tp);
}

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
