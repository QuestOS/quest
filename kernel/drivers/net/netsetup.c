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

#include "kernel.h"
#include "module/header.h"
#include "drivers/net/ethernet.h"
#include "sched/sched.h"

#define STATIC_IP_ASSIGNMENT

static u32 netsetup_stack[1024] ALIGNED (0x1000);
static u32 netsetup_custom_stack[1024] ALIGNED (0x1000);
static task_id netsetup_id = 0;

static void netsetup_thread (void);
static void netsetup_custom_thread (struct ip_addr, struct ip_addr, struct ip_addr, int num);

extern void socket_sys_call_init (void);

#ifdef STATIC_IP_ASSIGNMENT
  struct ip_addr ipaddr, netmask, gw;
#endif

/* hard-code the configuration for now */
bool
netsetup_init (void)
{
  net_set_default ("en0");
  socket_sys_call_init ();
  netsetup_id =
      start_kernel_thread ((u32) netsetup_thread,
                           (u32) &netsetup_stack[1023]);
  return TRUE;
}

bool
netsetup_custom_init (struct ip_addr ip,
                      struct ip_addr nm,
                      struct ip_addr gw,
                      int num)
{
  socket_sys_call_init ();
  netsetup_id =
      create_kernel_thread_args ((u32) netsetup_custom_thread,
                                 (u32) &netsetup_custom_stack[1023],
                                 TRUE, 4, ip, nm, gw, num);
  return TRUE;
}

static void
netsetup_custom_thread (struct ip_addr ip,
                        struct ip_addr nm,
                        struct ip_addr gw,
                        int num)
{
  int cpu;
  char ifn[4];
  cpu = get_pcpu_id ();
  ifn[0] = 'e';
  ifn[1] = 'n';
  ifn[2] = '0' + (char) num; /* Let's restrict num to 0 .. 9 */
  ifn[3] = '\0';

  logger_printf ("netsetup (%d): Configuring interface (%s) ...\n", cpu, ifn);
  for (;;) {
    struct netif *netif = netif_find (ifn);
    if (netif) {
      netif_set_addr (netif, &ip, &nm, &gw);
      netif_set_up (netif);
    } else {
      logger_printf ("Cannot find interface\n");
    }
    logger_printf ("netsetup (%d): %s configured.\n", cpu, ifn);
    exit_kernel_thread ();
  }
}

static void
netsetup_thread (void)
{
  int cpu;
  cpu = get_pcpu_id ();

  logger_printf ("netsetup (%d): Network set up thread started\n", cpu);
  for (;;) {
#ifdef STATIC_IP_ASSIGNMENT
    struct netif *netif = netif_find ("en0");
    if (netif) {
      IP4_ADDR(&ipaddr, 192, 168, 2, 11 + cpu);
      //IP4_ADDR(&ipaddr, 192, 168, 2, 11);
      IP4_ADDR(&netmask, 255, 255, 255, 0);
      IP4_ADDR(&gw, 192, 168, 2, 1);
      netif_set_addr (netif, &ipaddr, &netmask, &gw);
      //net_static_config("en0", "192.168.2.11", "192.168.2.1", "255.255.255.0");
      netif_set_up (netif);
    } else {
      logger_printf ("Cannot find interface\n");
    }
#else
    net_dhcp_start ("en0");
#endif
    logger_printf ("netsetup (%d): Network set up thread exited\n", cpu);
    exit_kernel_thread ();
  }
}

static const struct module_ops mod_ops = {
  .init = netsetup_init
};

DEF_MODULE (netsetup, "Network configuration", &mod_ops, {"net___"});

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
