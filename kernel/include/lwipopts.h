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
#define LWIP_DBG_TYPES_ON               1
#define ETHARP_DEBUG                    LWIP_DBG_ON
#define IP_DEBUG                        LWIP_DBG_ON
#define UDP_DEBUG                       LWIP_DBG_ON
#endif

#endif /* __LWIPOPTS_H__ */
