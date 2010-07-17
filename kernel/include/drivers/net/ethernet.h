/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#ifndef _ETHERNET_H_
#define _ETHERNET_H_

#include "types.h"
#include "lwip/netif.h"

#define MAX_FRAME_SIZE 1600
#define ETH_ADDR_LEN 6

struct _ethernet_device;

typedef void (*packet_recv_func_t)(struct _ethernet_device *dev,
                                   uint8* buffer, sint len);
typedef sint (*packet_send_func_t)(uint8* buffer, sint len);
typedef bool (*get_hwaddr_func_t)(uint8 addr[ETH_ADDR_LEN]);
typedef void (*packet_poll_func_t)(void);

typedef struct _ethernet_device {
  /* ethernet device number */
  uint num;
  /* function that should be invoked by the driver when a packet
   * arrives on its device */
  packet_recv_func_t recv_func;
  /* function that should be invoked by other subsystems to send a
   * packet out on this device */
  packet_send_func_t send_func;
  /* function that populates a buffer with the hardware address */
  get_hwaddr_func_t  get_hwaddr_func;
  /* function that attempts to poll the network device */
  packet_poll_func_t poll_func;
  /* lwip network interface struct */
  struct netif netif;
} ethernet_device;

void net_init (void);
bool net_register_device (ethernet_device *);
bool net_set_default (char *devname);
bool net_dhcp_start (char *devname);
bool net_set_up (char *devname);
bool net_static_config(char *devname, char *myip_s, char *gwip_s, char *netmask_s);


/* From Linux */

/**
 * is_zero_ether_addr - Determine if give Ethernet address is all zeros.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is all zeroes.
 */
static inline int
is_zero_ether_addr(const u8 *addr)
{
  return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

/**
 * is_multicast_ether_addr - Determine if the Ethernet address is a multicast.
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is a multicast address.
 * By definition the broadcast address is also a multicast address.
 */
static inline int
is_multicast_ether_addr(const u8 *addr)
{
  return (0x01 & addr[0]);
}

/**
 * is_local_ether_addr - Determine if the Ethernet address is locally-assigned one (IEEE 802).
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is a local address.
 */
static inline int
is_local_ether_addr(const u8 *addr)
{
  return (0x02 & addr[0]);
}

/**
 * is_broadcast_ether_addr - Determine if the Ethernet address is broadcast
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Return true if the address is the broadcast address.
 */
static inline int
is_broadcast_ether_addr(const u8 *addr)
{
  return (addr[0] & addr[1] & addr[2] & addr[3] & addr[4] & addr[5]) == 0xff;
}

/**
 * is_valid_ether_addr - Determine if the given Ethernet address is valid
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Check that the Ethernet address (MAC) is not 00:00:00:00:00:00, is not
 * a multicast address, and is not FF:FF:FF:FF:FF:FF.
 *
 * Return true if the address is valid.
 */
static inline int
is_valid_ether_addr(const u8 *addr)
{
  /* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
   * explicitly check for it here. */
  return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

#endif
