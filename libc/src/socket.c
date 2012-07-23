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
#include <sys/select.h>
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
  return socket_getsockname (sockfd, addr, alenp);
}

int
getaddrinfo (const char *host, const char *service,
             const struct addrinfo *hint, struct addrinfo **res)
{
  struct addrinfo *internal_addrinfo =
    (struct addrinfo *) malloc (sizeof (struct addrinfo));
  struct sockaddr_in *internal_sockaddr =
    (struct sockaddr_in *) malloc (sizeof (struct sockaddr_in));

  int cpu = socket_get_sb_id ();
  /* Assign ports... sort of... */
  static unsigned short p = 13452;
  unsigned int ip4_addr = 0;
  internal_addrinfo->ai_family = AF_INET;
  /* Local host or IP_ANY? Let's assume we have only one interface for each sandbox */
  if ((strcmp (host, "localhost") == 0) || (strcmp (host, "0.0.0.0") == 0)) {

    if (cpu == 0)
      inet_pton (AF_INET, "192.168.2.11", &ip4_addr);
    else if (cpu == 1)
      inet_pton (AF_INET, "192.168.2.12", &ip4_addr);
    else if (cpu == 2)
      inet_pton (AF_INET, "192.168.2.13", &ip4_addr);
    else if (cpu == 3)
      inet_pton (AF_INET, "192.168.2.14", &ip4_addr);

    internal_sockaddr->sin_family = AF_INET;
    internal_sockaddr->sin_addr.s_addr = ip4_addr;
    internal_sockaddr->sin_port = htons (p++);
    internal_addrinfo->ai_addr = (struct sockaddr *)internal_sockaddr;
  } else {
    /* Remote host? OK, hack. */
    inet_pton (AF_INET, "192.168.2.1", &ip4_addr);
    internal_sockaddr->sin_family = AF_INET;
    internal_sockaddr->sin_addr.s_addr = ip4_addr;
    if (cpu == 0)
      internal_sockaddr->sin_port = htons (12865);
    else if (cpu == 1)
      internal_sockaddr->sin_port = htons (12866);
    else if (cpu == 2)
      internal_sockaddr->sin_port = htons (12867);
    else if (cpu == 3)
      internal_sockaddr->sin_port = htons (12868);
    internal_addrinfo->ai_addr = (struct sockaddr *)internal_sockaddr;
  }

  internal_addrinfo->ai_addrlen = sizeof (struct sockaddr_in);
  internal_addrinfo->ai_canonname = "Quest-V";
  internal_addrinfo->ai_next = (struct addrinfo *) 0;

  switch (hint->ai_socktype) {
    case SOCK_STREAM :
      internal_addrinfo->ai_socktype = SOCK_STREAM;
      internal_addrinfo->ai_protocol = IPPROTO_TCP;
      break;
    case SOCK_DGRAM :
      internal_addrinfo->ai_socktype = SOCK_DGRAM;
      internal_addrinfo->ai_protocol = IPPROTO_UDP;
      break;
    default :
      printf ("Hint giving unsupported socket type %d\n", hint->ai_socktype);
      return 1;
  }

  *res = internal_addrinfo;
  
  return 0;
}

void
freeaddrinfo (struct addrinfo *ai)
{
  free (ai->ai_addr);
  free (ai);
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
