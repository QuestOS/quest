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

#endif
