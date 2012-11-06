/*                    The Quest Operating System
 *  Portions Copyright (C) 2005-2012  Richard West, Boston University
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

#ifndef _ARPA_INET_H
#define _ARPA_INET_H

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

extern uint32_t htonl (uint32_t hostint32);
extern uint16_t htons (uint16_t hostint16);
extern uint32_t ntohl (uint32_t netint32);
extern uint16_t ntohs (uint16_t netint16);
extern const char *inet_ntop (int domain, const void *addr, char *str, socklen_t size);
extern int inet_pton (int domain, const char *str, void *addr);
extern uint32_t inet_addr (const char *cp);
extern char *inet_ntoa(struct in_addr addr);
extern int inet_aton(const char *cp, struct in_addr *addr);

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
