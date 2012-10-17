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

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* Modified by: Matthew Danish (md) while porting to Quest */

#define LWIP_DHCP         1

#define NO_SYS            1
#define LWIP_SOCKET       0
#define LWIP_NETCONN      0
#define LWIP_RAW          0

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE                32000

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#define MEMP_NUM_PBUF           60

/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE          92

#if 0
#define LWIP_DEBUG
#define LWIP_DBG_TYPES_ON               (~0)
#define ETHARP_DEBUG                    LWIP_DBG_ON
#define IP_DEBUG                        LWIP_DBG_ON
#define UDP_DEBUG                       LWIP_DBG_ON
#endif

#endif /* __LWIPOPTS_H__ */
