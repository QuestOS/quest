#include "our_func.h"
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>

int gethostname(char *name, size_t len)
{
  int cpu = socket_get_sb_id ();
  snprintf(name, len, "Quest-V-%d", cpu);
  return 0;
}
