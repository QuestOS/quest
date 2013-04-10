#include "our_func.h"
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>


int getpeername(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
  struct sockaddr_in *internal_sockaddr = (struct sockaddr_in *)addr;

  int cpu = socket_get_sb_id ();
  /* Assign ports... sort of... */
  unsigned int ip4_addr = 0;

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

  *addrlen = sizeof(internal_sockaddr);
  return 0;
}
