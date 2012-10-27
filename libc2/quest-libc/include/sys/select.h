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

#ifndef _SYS_SELECT_H
#define _SYS_SELECT_H

#ifndef MAX_FD
#define MAX_FD  128
#endif

#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>

//#define FD_SETSIZE  MAX_FD

//typedef unsigned char fd_set[MAX_FD];

/*
#define FD_ZERO(set)                        \
  do {                                      \
    int __i;                                \
    for (__i = 0; __i < MAX_FD; __i++)      \
      (*(set))[__i] = 0;                    \
  } while(0)

  
#define FD_SET(fd, set)                     \
  do {                                      \
    (*(set))[(fd)] = 1;                     \
  } while(0)

#define FD_CLR(fd, set)                     \
  do {                                      \
    (*(set))[(fd)] = 0;                     \
  } while(0)

#define FD_ISSET(fd, set)  ((int) ((*(set))[(fd)]))
*/
/*
struct timeval {
  long int tv_sec;    // Seconds 
  long int tv_usec;   // Microseconds 
};
*/

extern int select (int maxfdp1, fd_set * readfds, fd_set * writefds,
                   fd_set * exceptfds, struct timeval * tvptr);

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
