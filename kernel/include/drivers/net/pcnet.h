#ifndef _PCNET_H_
#define _PCNET_H_
#include "types.h"
#include "drivers/net/ethernet.h"

bool pcnet_init (void);
bool pcnet_get_hwaddr (uint8 addr[ETH_ADDR_LEN]);
sint pcnet_transmit (uint8* buf, sint len);

#endif
