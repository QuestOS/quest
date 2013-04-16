#include <sys/socket.h>
#include <netdb.h>
#include "getnameinfo.h"

#define unimplemented_function_called()                                  \
  do {                                                                  \
    printf("\n\nExiting because %s in %s is unimplemented\n", __FUNCTION__, __FILE__); \
    exit(-1);                                                           \
  } while(0)

int getnameinfo(const struct sockaddr *sa, socklen_t salen,
		char *host, size_t hostlen,
		char *serv, size_t servlen, int flags)
{
	// TODO make this do something
	unimplemented_function_called();
	return 0;
}
