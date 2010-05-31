/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/* Ported to Quest by: Matthew Danish (md)
 *
 * Changes Copyright (c) 2010 Boston University
 */

#include "lwip/init.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "lwip/netif.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include "lwip/dhcp.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"
#include "netif/ppp_oe.h"

#include "types.h"
#include "string.h"
#include "drivers/net/pcnet.h"
#include "util/debug.h"
#include "util/printf.h"

#define DEBUG_NETIF

#ifdef DEBUG_NETIF
#define DLOG(fmt,...) DLOG_PREFIX("netif",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
  /* current location and size of received buffer */
  uint8* cur_buf;
  uint cur_len;
};

/* Forward declarations. */
static void  ethernetif_input(struct netif *netif);

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
  uint error;
  /*struct ethernetif *ethernetif = reinterpret_cast<struct ethernetif*> (netif->state);*/

  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;

  pcnet_get_hwaddr (netif->hwaddr);

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

  /* Do whatever else is needed to initialize interface. */
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
  /*struct ethernetif *ethernetif = netif->state;*/
  struct pbuf *q;
  uint error;
  uint8 buffer[1600], *ptr; /* this is a kludge for the moment */

#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

  ptr = buffer;
  for(q = p; q != NULL; q = q->next) {
    /* Send the data from the pbuf to the interface, one pbuf at a
       time. The size of the data in each pbuf is kept in the ->len
       variable. */
    memcpy(ptr, q->payload, q->len);
    ptr += q->len;
  }

  if (pcnet_transmit (buffer, p->tot_len) != p->tot_len)
    return ERR_BUF;

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

  LINK_STATS_INC(link.xmit);

  return ERR_OK;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *
low_level_input(struct netif *netif)
{
  struct ethernetif *ethernetif = netif->state;
  struct pbuf *p, *q;
  uint8 *buffer;
  uint len, start, err;

  len = ethernetif->cur_len;
  buffer = ethernetif->cur_buf;

#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

  if (p != NULL) {

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif


    /* Read enough bytes to fill this pbuf in the chain. The
     * available data in the pbuf is given by the q->len
     * variable.
     * This does not necessarily have to be a memcpy, you can also preallocate
     * pbufs for a DMA-enabled MAC and after receiving truncate it to the
     * actually received size. In this case, ensure the tot_len member of the
     * pbuf is the sum of the chained pbuf len members.
     */
    start=0;
    for (q = p; q != NULL; q = q->next) {
      /* Read enough bytes to fill this pbuf in the chain. The
         available data in the pbuf is given by the q->len
         variable. */
      /* read data into(q->payload, q->len); */
      memcpy(q->payload, (uint8 *)&buffer[start], q->len);
      start += q->len;
      len -= q->len;
      if (len<=0) {
        break;
      }

    }

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.recv);
  } else {
    LINK_STATS_INC(link.memerr);
    LINK_STATS_INC(link.drop);
  }

  return p;
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
static void
ethernetif_input(struct netif *netif)
{
  /*struct ethernetif *ethernetif;*/
  struct eth_hdr *ethhdr;
  struct pbuf *p = low_level_input (netif);

  if (!p) return;

  /*ethernetif = netif->state;*/

  ethhdr = (struct eth_hdr*) (p->payload);

  switch (htons(ethhdr->type)) {
    /* IP or ARP packet? */
  case ETHTYPE_IP:
  case ETHTYPE_ARP:
    /* full packet send to tcpip_thread to process */
    if (netif->input(p, netif)!=ERR_OK) {
      LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
      pbuf_free(p);
      p = NULL;
    }
    break;

  default:
    pbuf_free(p);
    p = NULL;
    break;
  }

}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
ethernetif_init(struct netif *netif)
{
  struct ethernetif *ethernetif;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

  ethernetif = (struct ethernetif*) (mem_malloc(sizeof(struct ethernetif)));
  if (ethernetif == NULL) {
   LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
   return ERR_MEM;
  }

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->state = ethernetif;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output = etharp_output;
  netif->linkoutput = low_level_output;

  /*ethernetif->ethaddr = (struct eth_addr *)&(netif->hwaddr[0]);*/

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/* ************************************************** */

/* Demo Echo server on port 7 */

static void 
echo_close (struct tcp_pcb* pcb)
{
  tcp_close (pcb);
}

static err_t 
echo_sent (void* arg, struct tcp_pcb* pcb, u16_t len)
{
  DLOG ("echo_sent (%p, %p, %p)", arg, pcb, len);
  return ERR_OK;
}

static err_t 
echo_send (struct tcp_pcb* pcb, struct pbuf* p)
{
  err_t wr_err = ERR_OK;
  struct pbuf* q;
  u16_t plen;

  while (wr_err == ERR_OK &&
         p != NULL &&
         p->len <= tcp_sndbuf (pcb)) {
    q = p;
    wr_err = tcp_write (pcb, p->payload, p->len, 1);
    if (wr_err == ERR_OK) {
      plen = p->len;
      p = p->next;
      if (p)
        pbuf_ref (p);
      while (pbuf_free (q) == 0);
      tcp_recved (pcb, plen);
    }
  }
  return ERR_OK;
}

static err_t 
echo_recv (void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err)
{
  err_t ret_err = ERR_OK;
  DLOG ("echo_recv (%p, %p, %p, %p)", arg, pcb, p, err);
  if (p == NULL) {
    if (p) pbuf_free (p);
    echo_close (pcb);
  } else if (err != ERR_OK) {
    if (p) pbuf_free (p);
  } else {
    tcp_sent(pcb, echo_sent);
    echo_send(pcb, p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

static err_t 
echo_accept (void* arg, struct tcp_pcb* pcb, err_t err)
{
  DLOG ("echo_accept (%p, (%p, %d, %d), %p)",
        arg, pcb, pcb->local_port, pcb->remote_port, err);
  tcp_accepted (pcb);
  tcp_recv (pcb, echo_recv);

  return ERR_OK;
}

static void
echo_init (void)
{
  struct tcp_pcb* echo_pcb = tcp_new ();
  tcp_bind(echo_pcb, IP_ADDR_ANY, 7);
  echo_pcb = tcp_listen(echo_pcb);
  tcp_accept (echo_pcb, echo_accept);
}

/* ************************************************** */

/* external init routine */

static struct netif netif;

void dispatch(uint8* buf, uint len)
{
  struct ethernetif *ethernetif = netif.state;
  ethernetif->cur_buf = buf;
  ethernetif->cur_len = len;
  ethernetif_input (&netif);
}

void init_interface(bool dhcp, char *myip_s, char *gwip_s, char *netmask_s)
{
  struct in_addr inaddr;
  struct ip_addr ipaddr, netmask, gw;

  if (dhcp)
    DLOG ("init_interface: dhcp");
  else
    DLOG ("init_interface: %s, %s, %s:", myip_s, gwip_s, netmask_s);

  pcnet_dispatch_packet = dispatch;

  if (!dhcp) {
    inet_aton (myip_s, &inaddr);
    ipaddr.addr = inaddr.s_addr;

    inet_aton (gwip_s, &inaddr);
    gw.addr = inaddr.s_addr;

    inet_aton (netmask_s, &inaddr);
    netmask.addr = inaddr.s_addr;

    DLOG ("    %p, %p, %p", ipaddr.addr, gw.addr, netmask.addr);

    netif_add (&netif, &ipaddr, &netmask, &gw, 
               NULL, ethernetif_init, ethernet_input);
  } else {
    netif_add (&netif, IP_ADDR_ANY, IP_ADDR_ANY, IP_ADDR_ANY,
               NULL, ethernetif_init, ethernet_input);
  }

  DLOG ("init_interface netif_add complete");
  netif_set_default (&netif);
  DLOG ("init_interface netif_set_default complete");
  netif_set_up (&netif);
  DLOG ("init_interface netif_set_up: complete");

  if (dhcp) {
    DLOG ("init_interface dhcp_start");
    dhcp_start (&netif);
  }

  DLOG ("init_interface netif configured.");
}

void net_init(void)
{
  lwip_init ();
  init_interface (TRUE, NULL, NULL, NULL);
  echo_init ();
}

void net_tmr_process(void)
{
  extern volatile uint32 tick;
  static uint32 next_tcp_time = 0;
  static uint32 next_etharp_time = 0;
#if LWIP_DHCP
  static uint32 next_dhcp_coarse_time = 0;
  static uint32 next_dhcp_fine_time = 0;
#endif

  uint32 now = tick;

  /* assume HZ=100 */

  if (now >= next_tcp_time) {
    tcp_tmr ();
    next_tcp_time = now + (TCP_TMR_INTERVAL / 10);
  }

  if (now >= next_etharp_time) {
    etharp_tmr ();
    next_etharp_time = now + (ARP_TMR_INTERVAL / 10);
  }

#if LWIP_DHCP
  if (now >= next_dhcp_coarse_time) {
    dhcp_coarse_tmr ();
    next_dhcp_coarse_time = now + (DHCP_COARSE_TIMER_MSECS / 10);
  }

  if (now >= next_dhcp_fine_time) {
    dhcp_fine_tmr ();
    next_dhcp_fine_time = now + (DHCP_FINE_TIMER_MSECS / 10);
  }
#endif
}
