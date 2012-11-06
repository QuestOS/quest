#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>

int
socket (int domain, int type, int protocol)
{
  int sockfd = open_socket (domain, type, protocol);
  return sockfd;
}

int
select (int maxfdp1, fd_set * readfds, fd_set * writefds,
        fd_set * exceptfds, struct timeval * tvptr)
{
  return socket_select (maxfdp1, readfds, writefds, exceptfds, tvptr);
}

uint32_t
htonl (uint32_t hostint32)
{
  return ((hostint32 & 0xff) << 24) |
    ((hostint32 & 0xff00) << 8) |
    ((hostint32 & 0xff0000UL) >> 8) |
    ((hostint32 & 0xff000000UL) >> 24);
}

uint16_t
htons (uint16_t hostint16)
{
  return ((hostint16 & 0xff) << 8) | ((hostint16 & 0xff00) >> 8);
}

uint32_t
ntohl (uint32_t netint32)
{
  return htonl(netint32);
}

uint16_t
ntohs (uint16_t netint16)
{
  return htons(netint16);
}

int
inet_pton(int domain, const char *cp, void *addr)
{
  uint32_t val;
  uint8_t base;
  char c;
  uint32_t parts[4];
  uint32_t *pp = parts;

  c = *cp;
  for (;;) {
    /*
     * Collect number up to ``.''.
     * Values are specified as for C:
     * 0x=hex, 0=octal, 1-9=decimal.
     */
    if (!isdigit(c))
      return (0);
    val = 0;
    base = 10;
    if (c == '0') {
      c = *++cp;
      if (c == 'x' || c == 'X') {
        base = 16;
        c = *++cp;
      } else
        base = 8;
    }
    for (;;) {
      if (isdigit(c)) {
        val = (val * base) + (int)(c - '0');
        c = *++cp;
      } else if (base == 16 && isxdigit(c)) {
        val = (val << 4) | (int)(c + 10 - (islower(c) ? 'a' : 'A'));
        c = *++cp;
      } else
        break;
    }
    if (c == '.') {
      /*
       * Internet format:
       *  a.b.c.d
       *  a.b.c   (with c treated as 16 bits)
       *  a.b (with b treated as 24 bits)
       */
      if (pp >= parts + 3)
        return (0);
      *pp++ = val;
      c = *++cp;
    } else
      break;
  }
  /*
   * Check for trailing characters.
   */
  if (c != '\0' && !isspace(c))
    return (0);
  /*
   * Concoct the address according to
   * the number of parts specified.
   */
  switch (pp - parts + 1) {

  case 0:
    return (0);       /* initial nondigit */

  case 1:             /* a -- 32 bits */
    break;

  case 2:             /* a.b -- 8.24 bits */
    if (val > 0xffffffUL)
      return (0);
    val |= parts[0] << 24;
    break;

  case 3:             /* a.b.c -- 8.8.16 bits */
    if (val > 0xffff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16);
    break;

  case 4:             /* a.b.c.d -- 8.8.8.8 bits */
    if (val > 0xff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
    break;
  }
  if (addr)
    *((uint32_t *) addr) = htonl(val);
  return (1);
}

const char *
inet_ntop(int domain, const void *addr, char *str, socklen_t size)
{
  uint32_t s_addr = *((uint32_t *) addr);
  char inv[3];
  char *rp;
  uint8_t *ap;
  uint8_t rem;
  uint8_t n;
  uint8_t i;

  rp = str;
  ap = (uint8_t *)&s_addr;
  for(n = 0; n < 4; n++) {
    i = 0;
    do {
      rem = *ap % (uint8_t)10;
      *ap /= (uint8_t)10;
      inv[i++] = '0' + rem;
    } while(*ap);
    while(i--)
      *rp++ = inv[i];
    *rp++ = '.';
    ap++;
  }
  *--rp = 0;
  return str;
}

int
inet_aton(const char *cp, struct in_addr *addr)
{
  uint32_t val;
  uint8_t base;
  char c;
  uint32_t parts[4];
  uint32_t *pp = parts;

  c = *cp;
  for (;;) {
    /*
     * Collect number up to ``.''.
     * Values are specified as for C:
     * 0x=hex, 0=octal, 1-9=decimal.
     */
    if (!isdigit(c))
      return (0);
    val = 0;
    base = 10;
    if (c == '0') {
      c = *++cp;
      if (c == 'x' || c == 'X') {
        base = 16;
        c = *++cp;
      } else
        base = 8;
    }
    for (;;) {
      if (isdigit(c)) {
        val = (val * base) + (int)(c - '0');
        c = *++cp;
      } else if (base == 16 && isxdigit(c)) {
        val = (val << 4) | (int)(c + 10 - (islower(c) ? 'a' : 'A'));
        c = *++cp;
      } else
        break;
    }
    if (c == '.') {
      /*
       * Internet format:
       *  a.b.c.d
       *  a.b.c   (with c treated as 16 bits)
       *  a.b (with b treated as 24 bits)
       */
      if (pp >= parts + 3)
        return (0);
      *pp++ = val;
      c = *++cp;
    } else
      break;
  }
  /*
   * Check for trailing characters.
   */
  if (c != '\0' && !isspace(c))
    return (0);
  /*
   * Concoct the address according to
   * the number of parts specified.
   */
  switch (pp - parts + 1) {

  case 0:
    return (0);       /* initial nondigit */

  case 1:             /* a -- 32 bits */
    break;

  case 2:             /* a.b -- 8.24 bits */
    if (val > 0xffffffUL)
      return (0);
    val |= parts[0] << 24;
    break;

  case 3:             /* a.b.c -- 8.8.16 bits */
    if (val > 0xffff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16);
    break;

  case 4:             /* a.b.c.d -- 8.8.8.8 bits */
    if (val > 0xff)
      return (0);
    val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
    break;
  }
  if (addr)
    addr->s_addr = htonl(val);
  return (1);
}

char *
inet_ntoa(struct in_addr addr)
{
  static char str[16];
  uint32_t s_addr = addr.s_addr;
  char inv[3];
  char *rp;
  uint8_t *ap;
  uint8_t rem;
  uint8_t n;
  uint8_t i;

  rp = str;
  ap = (uint8_t *)&s_addr;
  for(n = 0; n < 4; n++) {
    i = 0;
    do {
      rem = *ap % (uint8_t)10;
      *ap /= (uint8_t)10;
      inv[i++] = '0' + rem;
    } while(*ap);
    while(i--)
      *rp++ = inv[i];
    *rp++ = '.';
    ap++;
  }
  *--rp = 0;
  return str;
}


uint32_t
inet_addr(const char *cp)
{
  struct in_addr val;

  if (inet_aton(cp, &val)) {
    return (val.s_addr);
  }
  return (INADDR_NONE);
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
