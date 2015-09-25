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

/* MAC 802.11 implements the link-layer protocol for 802.11-based
 * wireless networking. */

#include <kernel.h>
#include <drivers/net/ieee80211.h>
#include <drivers/net/mac80211.h>
#include <drivers/net/ieee80211_standard.h>
#include <drivers/net/ethernet.h>
#include <util/debug.h>
#include <mem/malloc.h>
#include <sched/sched.h>
#include <arch/i386.h>
#include <string.h>

typedef uint8 ethaddr_t[ETH_ADDR_LEN];
#define SSID "quest"
ethaddr_t quest_bssid = {
  0xb6, 0x27, 0x07, 0xb6, 0x73, 0x7E
};
#define TIME_UNIT 1024          /* usec */

#define DEBUG_MAC80211

#ifdef DEBUG_MAC80211
#define DLOG(fmt,...) DLOG_PREFIX("mac80211",fmt,##__VA_ARGS__)
#else
#define DLOG(fmt,...) ;
#endif


#define MAX_MACS 4

struct ieee80211_local {
  struct ieee80211_hw hw;
  const struct ieee80211_ops *ops;
  ethernet_device ethdev;
  char *ssid;
  u64 prev_time_update;
  struct ieee80211_vif vif;
  char priv[0] ALIGNED(32);
};
typedef struct ieee80211_local local_t;

static inline struct ieee80211_local *
hw_to_local(struct ieee80211_hw *hw)
{
  return container_of(hw, struct ieee80211_local, hw);
}

static inline struct ieee80211_hw *
local_to_hw(struct ieee80211_local *local)
{
  return &local->hw;
}

struct wiphy *
wiphy_new (int sizeof_priv)
{
  return kmalloc(sizeof_priv + sizeof (struct wiphy));
}

struct ieee80211_hw *
ieee80211_alloc_hw (size_t priv_data_len,
                    const struct ieee80211_ops *ops)
{
  local_t *local;
  struct wiphy *wiphy;

  wiphy = wiphy_new (priv_data_len + sizeof (local_t));
  if (!wiphy) return NULL;

  local = (void *)wiphy->priv;

  local->ops = ops;

  return &local->hw;
}

static int num_macs = 0;
static local_t *hw_table[MAX_MACS];

static struct ieee80211_channel channels[] = {
  { .center_freq = 2412 },
  { .center_freq = 2417 },
  { .center_freq = 2422 },
  { .center_freq = 2427 },
  { .center_freq = 2432 },
  { .center_freq = 2437 },
  { .center_freq = 2442 },
  { .center_freq = 2447 },
  { .center_freq = 2452 },
  { .center_freq = 2457 },
  { .center_freq = 2462 },
  { .center_freq = 2467 },
  { .center_freq = 2472 },
  { .center_freq = 2484 },
};

static uint32 beacon_stack[1024] ALIGNED(0x1000);
static void beacon_thread (local_t *);
local_t *hack_local = NULL;
sint mac80211_tx (uint8* buffer, sint len);
bool mac80211_get_hwaddr (uint8 addr[ETH_ADDR_LEN]);
void mac80211_poll (void);
static void init_tsc_ufreq (void);

bool
ieee80211_register_hw (struct ieee80211_hw *hw)
{
  local_t *local = hw_to_local (hw);
  struct ieee80211_if_init_conf if_conf;
  struct ieee80211_tx_queue_params tx_q_p = {
    .txop = 94,
    .cw_min = 7,
    .cw_max = 15,
    .aifs = 2
  };

  if (num_macs < MAX_MACS) {
    hw_table[num_macs] = local;
    num_macs++;
  } else return FALSE;

  /* initialize things */

  DLOG ("register_hw: initialize");
  if (!local->ops->add_interface (hw, &if_conf))
    return FALSE;

  local->ops->conf_tx (hw, 0, &tx_q_p);
  local->ops->conf_tx (hw, 1, &tx_q_p);
  local->ops->conf_tx (hw, 2, &tx_q_p);
  local->ops->conf_tx (hw, 3, &tx_q_p);

  if (!local->ops->start (hw))
    return FALSE;

  hw->conf.channel = &channels[0];

  if (!local->ops->config (hw, 0))
    return FALSE;

  local->vif.bss_conf.bssid = quest_bssid;
  local->vif.bss_conf.timestamp = 0;
  local->vif.bss_conf.beacon_int = 0x64;       /* FIXME */
  local->vif.bss_conf.assoc_capability = 0x22; /* FIXME */
  local->ssid = SSID;

  local->ethdev.recv_func = NULL;
  local->ethdev.send_func = mac80211_tx;
  local->ethdev.get_hwaddr_func = mac80211_get_hwaddr;
  local->ethdev.poll_func = mac80211_poll;
  hack_local = local;           /* until I fix ethernet_device */
  if (!net_register_device (&local->ethdev)) {
    DLOG ("registration failed");
    return FALSE;
  }

  init_tsc_ufreq ();

  create_kernel_thread_args ((u32) beacon_thread, (u32) &beacon_stack[1023],
                             "mac80211 beacon", TRUE, 1, local);

  return TRUE;
}

/* ************************************************** */

static u32 tsc_ufreq;           /* tsc per usec */
extern u64 tsc_freq;

static void
init_tsc_ufreq (void)
{
  u64 f;
  u32 f_hi, f_lo;
  u32 divisor = 1000000;
  f = tsc_freq;

  f_hi = (u32) (f >> 32);
  f_lo = (u32) (f & 0xFFFFFFFF);
  asm volatile ("div %1":"=a" (tsc_ufreq):"r" (divisor),"a" (f_lo),"d" (f_hi));

  DLOG ("tsc_ufreq=%d", tsc_ufreq);
}

static void
update_timestamp (local_t *local)
{
  u64 new_time_update, diff;
  RDTSC (new_time_update);
  if (local->prev_time_update == 0) {
    local->prev_time_update = new_time_update;
  } else {
    diff = new_time_update - local->prev_time_update;
    u32 f_hi, f_lo, udiff;
    f_hi = (u32) (diff >> 32);
    f_lo = (u32) (diff & 0xFFFFFFFF);
    asm volatile ("div %1":"=a" (udiff):"r" (tsc_ufreq),"a" (f_lo),"d" (f_hi));
    local->vif.bss_conf.timestamp += udiff;
  }
}

/* ************************************************** */

#if 0
static bool
make_mgmt_pkt (u8 subtype, u8 control,
               ethaddr_t da, ethaddr_t sa, ethaddr_t bssid);
#endif

static u8 *insert_eid (u8 *ptr, u8 eid, u8 len, void *data)
{
  *(ptr++) = eid;
  *(ptr++) = len;
  memcpy (ptr, data, len);
  return ptr + len;
}

static u32
make_beacon_pkt (local_t *local,
                 ethaddr_t sa,
                 u8 *rates, u8 rates_len,
                 u8 chan,
                 u8 *buf, u32 len)
{
  const uint8 *bssid = local->vif.bss_conf.bssid;
  char *ssid = local->ssid;
  struct ieee80211_mgmt *m = (struct ieee80211_mgmt *)buf;
  int i;
  u8 *ptr, ssid_len = strlen (ssid);
  u32 req = ((u32) m->u.beacon.variable - (u32) m)
    + 2 + ssid_len + 2 + rates_len + 3;

  if (len < req) return req;

  m->frame_control =
    IEEE80211_FTYPE_MGMT |
    IEEE80211_STYPE_BEACON;
  m->duration = 0;               /* FIXME */
  for (i=0;i<ETH_ADDR_LEN;i++) {
    m->da[i] = 0xFF;
    m->sa[i] = sa[i];
    m->bssid[i] = bssid[i];
  }
  m->u.beacon.timestamp = local->vif.bss_conf.timestamp;
  m->u.beacon.beacon_int = local->vif.bss_conf.beacon_int;
  m->u.beacon.capab_info = local->vif.bss_conf.assoc_capability;
  ptr = m->u.beacon.variable;
  ptr = insert_eid (ptr, WLAN_EID_SSID, ssid_len, ssid);
  ptr = insert_eid (ptr, WLAN_EID_SUPP_RATES, rates_len, rates);
  ptr = insert_eid (ptr, WLAN_EID_DS_PARAMS, 1, &chan);

  return req;
}

static u32
make_probe_req_pkt (ethaddr_t sa, ethaddr_t bssid,
                    char *ssid,
                    u8 *rates, u8 rates_len,
                    u8 *buf, u32 len)
{
  struct ieee80211_mgmt *m = (struct ieee80211_mgmt *)buf;
  int i;
  u8 *ptr, ssid_len = strlen (ssid);
  u32 req = ((u32) m->u.probe_req.variable - (u32) m)
    + 2 + ssid_len + 2 + rates_len;

  if (len < req) return req;

  m->frame_control =
    IEEE80211_FTYPE_MGMT |
    IEEE80211_STYPE_PROBE_REQ;
  m->duration = 0x013A;         /* FIXME */
  for (i=0;i<ETH_ADDR_LEN;i++) {
    m->da[i] = 0xFF;
    m->sa[i] = sa[i];
    m->bssid[i] = bssid[i];
  }
  m->seq_ctrl = 0x4640;         /* FIXME */
  ptr = m->u.probe_req.variable;
  ptr = insert_eid (ptr, WLAN_EID_SSID, ssid_len, ssid);
  ptr = insert_eid (ptr, WLAN_EID_SUPP_RATES, rates_len, rates);

  return req;
}

static u32
make_ack_pkt (ethaddr_t ra, u8 *buf, u32 len)
{
  struct ieee80211_hdr *h = (struct ieee80211_hdr *) buf;
  int i;
  if (len < 10) return 10;
  h->frame_control =
    IEEE80211_FTYPE_CTL |
    IEEE80211_STYPE_ACK;
  h->duration_id = 0;
  for (i=0; i<ETH_ADDR_LEN; i++)
    h->addr1[i] = ra[i];
  return 10;
}

static uint8 rates[8] = {
  0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
};

static void
tx_beacon (struct ieee80211_local *local)
{
  uint8 pkt[128];
  ethaddr_t sa = {
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E
  };
  u32 len = make_beacon_pkt (local, sa, rates, 8, 1, pkt, 128);
  if (len > 128) { DLOG ("make_beacon_pkt wants %d bytes", len); return; }
  struct sk_buff skb = {
    .data = pkt,
    .len = len,
  };
  local->ops->tx (local_to_hw (local), &skb);
}

static void
beacon_thread (local_t *local)
{
  DLOG ("beacon: hello from 0x%x", str ()->tid);
  for (;;) {
    sched_usleep (1000000);
    DLOG ("sending beacon timestamp=0x%.08X %.08X",
          (u32) (local->vif.bss_conf.timestamp >> 32),
          (u32) local->vif.bss_conf.timestamp);
    tx_beacon (local);
  }
}

static bool
find_eid (struct ieee80211_mgmt *m, s32 len, u8 eid,
          u8 *output, u8 max_len, u8 *act_len)
{
  //char *what;
  uint8 *ptr;
  u32 hdr_len;
#define CASE(x)                                         \
  if (ieee80211_is_##x (m->frame_control)) {            \
    ptr = m->u.x.variable;                              \
    hdr_len = ((u32) m->u.x.variable) - ((u32) m);      \
  }
  CASE(beacon);
  CASE(probe_req);
  CASE(probe_resp);
  CASE(assoc_req);
  CASE(assoc_resp);
#undef CASE
  //DLOG ("find_eid: eid=%d len=%d hdr_len=%d what=%s", eid, len, hdr_len, what);
  len -= hdr_len;
  for (;len > 0;) {
    //DLOG ("len=%d ptr[0]=%d ptr[1]=%d", len, ptr[0], ptr[1]);
    if (ptr[0] == eid) {
      if (ptr[1] > max_len) {
        return FALSE;
      } if (ptr[1] + 2 > len) {
        return FALSE;
      } else {
        memcpy (output, &ptr[2], ptr[1]);
        *act_len = ptr[1];
        return TRUE;
      }
    }
    len -= (ptr[1] + 2);
    ptr += (ptr[1] + 2);
  }
  return FALSE;
}

static void
tx_probe_resp (struct ieee80211_local *local)
{
  uint8 pkt[] = {
    0x50, 0x00, 0x3A, 0x01,             /* PROBE RESPONSE */
    0x00, 0x23, 0x4D, 0xAF, 0x00, 0x3C, /* DA */

    //0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    //0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E, /* SA */
    0xb6, 0x27, 0x07, 0xb6, 0x73, 0x7E, /* BSS ID */

    0x20, 0x00,

    0x8E, 0x91, 0x42, 0x11, 0x67, 0x00, 0x00, 0x00, /* timestamp */
    0x64, 0x00,                                     /* interval */
    0x01, 0x04,                                     /* capability */

    0x00, 0x05, 'q', 'u', 'e', 's', 't',

    0x01, 0x08, 0x82, 0x84, 0x8B, 0x96, 0x0C, 0x12, 0x18, 0x24
  };
  struct sk_buff skb = {
    .data = pkt,
    .len = sizeof (pkt)
  };
  struct ieee80211_mgmt *m = (struct ieee80211_mgmt *) pkt;
  m->u.probe_resp.timestamp = __cpu_to_le64 (local->vif.bss_conf.timestamp);
  local->ops->tx (local_to_hw (local), &skb);
}

/*

18:16:40.153738 314us arp who-has 169.254.237.67 tell 0.0.0.0
        0x0000:  0800 3a01 ffff ffff ffff 0023 4daf 003c
        0x0010:  b627 07b6 737e 4002 aaaa 0300 0000 0806
        0x0020:  0001 0800 0604 0001 0023 4daf 003c 0000
        0x0030:  0000 0000 0000 0000 a9fe ed43

18:16:40.181611 314us IP6 (hlim 1, next-header: Options (0), length: 36) fe80::e196:3567:8822:ed43 > ff02::16: HBH (rtalert: 0x0000) (padn)[icmp6 sum ok] ICMP6, multicast listener report v2, length 28, 1 group record(s) [gaddr ff02::1:3 to_ex { }]
        0x0000:  0800 3a01 3333 0000 0016 0023 4daf 003c
        0x0010:  b627 07b6 737e 5002 aaaa 0300 0000 86dd
        0x0020:  6000 0000 0024 0001 fe80 0000 0000 0000
        0x0030:  e196 3567 8822 ed43 ff02 0000 0000 0000
        0x0040:  0000 0000 0000 0016 3a00 0502 0000 0100
        0x0050:  8f00 e3a2 0000 0001 0400 0000 ff02 0000
        0x0060:  0000 0000 0000 0000 0001 0003

18:16:40.184486 314us IP (tos 0x0, ttl   1, id 10136, offset 0, flags [none], proto: IGMP (2), length: 40, options ( RA (148) len 4 )) 169.254.237.67 > IGMP.MCAST.NET: igmp v3 report, 1 group record(s) [gaddr 224.0.0.252 to_ex { }]
        0x0000:  0800 3a01 0100 5e00 0016 0023 4daf 003c
        0x0010:  b627 07b6 737e 6002 aaaa 0300 0000 0800
        0x0020:  4600 0028 2798 0000 0102 85df a9fe ed43
        0x0030:  e000 0016 9404 0000 2200 f901 0000 0001
        0x0040:  0400 0000 e000 00fc

18:16:40.196611 314us IP6 (hlim 1, next-header: UDP (17), length: 32) fe80::e196:3567:8822:ed43.49845 > ff02::1:3.hostmon: [udp sum ok] UDP, length 24
        0x0000:  0800 3a01 3333 0001 0003 0023 4daf 003c
        0x0010:  b627 07b6 737e 7002 aaaa 0300 0000 86dd
        0x0020:  6000 0000 0020 1101 fe80 0000 0000 0000
        0x0030:  e196 3567 8822 ed43 ff02 0000 0000 0000
        0x0040:  0000 0000 0001 0003 c2b5 14eb 0020 3ecf
        0x0050:  1115 0000 0001 0000 0000 0000 0663 686f
        0x0060:  7069 6e00 00ff 0001

18:16:40.197611 314us IP (tos 0x0, ttl   1, id 10137, offset 0, flags [none], proto: UDP (17), length: 52) 169.254.237.67.55152 > 224.0.0.252.hostmon: [udp sum ok] UDP, length 24
        0x0000:  0800 3a01 0100 5e00 00fc 0023 4daf 003c
        0x0010:  b627 07b6 737e 8002 aaaa 0300 0000 0800
        0x0020:  4500 0034 2799 0000 0111 19e2 a9fe ed43
        0x0030:  e000 00fc d770 14eb 0020 3bc1 1115 0000
        0x0040:  0001 0000 0000 0000 0663 686f 7069 6e00
        0x0050:  00ff 0001


18:33:49.213779 314us arp who-has 169.254.237.1 tell 169.254.237.67
        0x0000:  0800 3a01 ffff ffff ffff 0023 4daf 003c
        0x0010:  b627 07b6 737e 9094 aaaa 0300 0000 0806
        0x0020:  0001 0800 0604 0001 0023 4daf 003c a9fe
        0x0030:  ed43 0000 0000 0000 a9fe ed01

 */

static void
debug_buf (char *prefix, uint8 *buf, u32 len)
{
  s32 i, j;

  for (i=0;i<len;i+=8) {
    logger_printf ("%s: ", prefix);
    for (j=0;j<8;j++) {
      if (i+j >= len) break;
      logger_printf ("%.02X ", buf[i+j]);
    }
    logger_printf ("\n");
  }
}

extern void
ieee80211_rx (struct ieee80211_hw *hw, struct sk_buff *skb)
{
  local_t *local = hw_to_local (hw);
  struct ieee80211_hdr *hdr;
  char ssid[IEEE80211_MAX_SSID_LEN+1];
  u8 act_len;
  u8 pkt[128];
  u32 len, i;
  ethaddr_t sa = {
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E
  };
  ethaddr_t bssid = {
    0xB6, 0x27, 0x07, 0xB6, 0x73, 0x7E
  };

  update_timestamp (local);

  hdr = (struct ieee80211_hdr *)skb->data;

  if (ieee80211_is_mgmt (hdr->frame_control)) {
    struct ieee80211_mgmt *m = (struct ieee80211_mgmt *)skb->data;
    if (ieee80211_is_beacon (hdr->frame_control)) {
      if (find_eid (m, skb->len, WLAN_EID_SSID,
                    (u8 *)ssid, IEEE80211_MAX_SSID_LEN, &act_len)) {
        ssid[act_len] = 0;
        //DLOG ("beacon %s", ssid);
        if (strncmp (SSID, ssid, act_len) == 0) {
          local->vif.bss_conf.timestamp = m->u.beacon.timestamp + TIME_UNIT;
          local->vif.bss_conf.beacon_int = m->u.beacon.beacon_int;
          local->vif.bss_conf.assoc_capability = m->u.beacon.capab_info;
          DLOG ("beacon %s; synchronizing beacon_int=0x%x capab=0x%x",
                ssid, local->vif.bss_conf.beacon_int,
                local->vif.bss_conf.assoc_capability);
          DLOG ("  new timestamp=0x%.08X %.08X",
                (u32) (local->vif.bss_conf.timestamp >> 32),
                (u32) local->vif.bss_conf.timestamp);
        }
#if 0
        len = make_probe_req_pkt (sa, m->bssid, ssid, rates, 8,
                                  pkt, sizeof (pkt));
        if (len <= sizeof (pkt)) {
          struct sk_buff skb = {
            .data = pkt,
            .len = len,
          };
          local->ops->tx (local_to_hw (local), &skb);
        }
#endif
      } else {
        //DLOG ("beacon, no SSID");
      }
    } else if (ieee80211_is_probe_req (hdr->frame_control)) {
      if (find_eid (m, skb->len, WLAN_EID_SSID,
                    (u8 *)ssid, IEEE80211_MAX_SSID_LEN, &act_len)) {
        ssid[act_len] = 0;
        DLOG ("probe_req %s", ssid);
        if (strncmp (SSID, ssid, act_len) == 0) {
          tx_probe_resp (local);
        }
      } else {
        DLOG ("probe_req, no SSID");
      }
    } else if (ieee80211_is_probe_resp (hdr->frame_control)) {
      if (find_eid (m, skb->len, WLAN_EID_SSID,
                    (u8 *)ssid, IEEE80211_MAX_SSID_LEN, &act_len)) {
        ssid[act_len] = 0;
        DLOG ("probe_resp %s", ssid);
      } else {
        DLOG ("probe_resp, no SSID");
      }
      len = make_ack_pkt (m->sa, pkt, sizeof (pkt));
      if (len <= sizeof (pkt)) {
        struct sk_buff skb = {
          .data = pkt,
          .len = len,
        };
        local->ops->tx (local_to_hw (local), &skb);
      }
    }
  } else if (ieee80211_is_data (hdr->frame_control)) {
    if ((hdr->seq_ctrl & 0xF) == 0) {
      DLOG ("data for %.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
            hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
            hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
      for (i=0; i<ETH_ADDR_LEN; i++)
        if (hdr->addr3[i] != bssid[i])
          return;
#ifdef DEBUG_MAC80211
      debug_buf ("mac80211: data", skb->data, skb->len);
#endif
      /* hack in an "ethernet frame" */
      memcpy (skb->data + 18, skb->data + 4, ETH_ADDR_LEN);
      memcpy (skb->data + 24, skb->data + 10, ETH_ADDR_LEN);
      local->ethdev.recv_func (&local->ethdev,
                               skb->data + 18,
                               skb->len - 18);
    }
  } else if (ieee80211_is_ctl (hdr->frame_control)) {
    if (ieee80211_is_ack (hdr->frame_control)) {
      DLOG ("ack");
    }
  }
}


static u8 tx_buf[2500];

sint
mac80211_tx (uint8* buffer, sint len)
{
  struct ieee80211_hw *hw = local_to_hw (hack_local);
  bool is_arp=FALSE;
  u8 hdr[] = {
    0x08, 0x00, 0x3a, 0x01,
    0x00, 0x23, 0x4d, 0xaf, 0x00, 0x3c,
    0x00, 0x1e, 0x2a, 0x42, 0x73, 0x7e,
    0xb6, 0x27, 0x07, 0xb6, 0x73, 0x7e,
    0x90, 0x94,
    /* RFC 1042 SNAP header */
    0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00,
    /* ARP */
    //0x08, 0x06
    /* IP */
    0x08, 0x00
  };

  /* skip ethernet link layer header */
  len -= 14;
  buffer += 14;

  DLOG ("mac80211_tx: %p %d bytes: %x %x", buffer, len, buffer[0], buffer[1]);
  if (buffer[0] == 0 && buffer[1] == 1)
    is_arp = TRUE;              /* guess ARP */

  struct sk_buff skb = {
    .data = tx_buf,
    .len = len + sizeof (hdr)
  };
  memcpy (tx_buf, hdr, sizeof (hdr));
  memcpy (tx_buf + sizeof (hdr), buffer, len);
  if (is_arp) tx_buf[sizeof (hdr) - 1] = 6;

  return hack_local->ops->tx (hw, &skb);
}

bool
mac80211_get_hwaddr (uint8 addr[ETH_ADDR_LEN])
{
  int i;
  /* FIXME */
  ethaddr_t sa = {
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E
  };

  for (i=0; i<ETH_ADDR_LEN; i++)
    addr[i] = sa[i];
  return TRUE;
}

void
mac80211_poll (void)
{
}

u16
ieee80211_calc_duration (u32 len, s16 rate)
{
  u16 duration;
  u16 drift;
  switch (rate) {
  case 0: //1mbps
    duration = ((len+4)<<4) / 0x2;
    drift = ((len+4)<<4) % 0x2;
    if (drift == 0) break;
    duration++;
    break;

  case 1: //2mbps
    duration = ((len+4)<<4) / 0x4;
    drift = ((len+4)<<4) % 0x4;
    if (drift == 0) break;
    duration++;
    break;

  case 2: //5.5mbps
    duration = ((len+4)<<4) / 0xb;
    drift = ((len+4)<<4) % 0xb;
    if (drift == 0) break;
    duration++;
    break;

  default:
  case 3: //11mbps                         
    duration = ((len+4)<<4) / 0x16;
    drift = ((len+4)<<4) % 0x16;
    if (drift == 0) break;
    duration++;
    break;
  }

  return duration;
}



/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
