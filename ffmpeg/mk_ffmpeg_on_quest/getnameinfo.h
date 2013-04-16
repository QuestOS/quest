#ifndef GET_NAME_INFO_H
#define GET_NAME_INFO_H

#include <sys/socket.h>
#include <netdb.h>

int getnameinfo(const struct sockaddr *sa, socklen_t salen,
		char *host, size_t hostlen,
		char *serv, size_t servlen, int flags);

#endif
