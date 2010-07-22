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

/* MAC 802.11 implements the link-layer protocol for 802.11-based
 * wireless networking. */

#include <kernel.h>
#include <drivers/net/ieee80211.h>
#include <drivers/net/mac80211.h>
#include <drivers/net/ieee80211_standard.h>
#include <util/debug.h>
#include <mem/pow2.h>
#include <sched/sched.h>
#include <string.h>

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
  bool link_up;
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
  int sz = sizeof_priv + sizeof (struct wiphy);
  struct wiphy *wiphy;
  if (pow2_alloc (sz, (u8 **)&wiphy))
    return wiphy;
  else
    return NULL;
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

static local_t *tmplocal;
static uint32 beacon_stack[1024] ALIGNED(0x1000);
static void beacon_thread (void);

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

  tmplocal = local;
  start_kernel_thread ((u32) beacon_thread, (u32) &beacon_stack[1023]);

  return TRUE;
}

/* ************************************************** */

typedef uint8 ethaddr_t[ETH_ADDR_LEN];

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
make_beacon_pkt (ethaddr_t sa, ethaddr_t bssid,
                 char *ssid,
                 u8 *rates, u8 rates_len,
                 u8 chan,
                 u8 *buf, u32 len)
{
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
  m->seq_ctrl = 0x4640;         /* FIXME */
  m->u.beacon.timestamp = 0x02c56200; /* FIXME */
  m->u.beacon.beacon_int = 0x64;      /* FIXME */
  m->u.beacon.capab_info = 0x22;      /* FIXME */
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
  ethaddr_t bssid = {
    0xb6, 0x27, 0x07, 0xb6, 0x73, 0x7E
  };
  u32 len = make_beacon_pkt (sa, bssid, "quest", rates, 8, 1, pkt, 128);
  if (len > 128) { DLOG ("make_beacon_pkt wants %d bytes", len); return; }
  struct sk_buff skb = {
    .data = pkt,
    .len = len,
  };
  local->ops->tx (local_to_hw (local), &skb);
}

static void
beacon_thread (void)
{
  local_t *local = tmplocal;
  DLOG ("beacon: hello from 0x%x", str ());
  for (;;) {
    sched_usleep (500000);
    tx_beacon (local);
  }
}

static bool
find_eid (struct ieee80211_mgmt *m, u32 len, u8 eid, u8 *output, u8 *act_len)
{
  uint8 *ptr;
  u32 hdr_len;
#define CASE(x)                                         \
  if (ieee80211_is_##x (m->frame_control)) {            \
    ptr = m->u.x.variable;                              \
    hdr_len = ((u32) &m->u.x.variable) - ((u32) &m);    \
  }
  CASE(beacon);
  CASE(probe_req);
  CASE(probe_resp);
  CASE(assoc_req);
  CASE(assoc_resp);
#undef CASE
  len -= hdr_len;
  for (;len > 0;) {
    if (ptr[0] == eid) {
      memcpy (output, &ptr[2], ptr[1]);
      *act_len = ptr[1];
      return TRUE;
    }
    len -= ptr[1];
    ptr += ptr[1];
  }
  return FALSE;
}

#define SSID "quest"

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
  local->ops->tx (local_to_hw (local), &skb);
}

extern void
ieee80211_rx (struct ieee80211_hw *hw, struct sk_buff *skb)
{
  local_t *local = hw_to_local (hw);
  struct ieee80211_hdr *hdr;
  char ssid[IEEE80211_MAX_SSID_LEN+1];
  u8 act_len;
  u8 pkt[128];
  u32 len;
  ethaddr_t sa = {
    0x00, 0x1E, 0x2A, 0x42, 0x73, 0x7E
  };

  hdr = (struct ieee80211_hdr *)skb->data;

  if (ieee80211_is_mgmt (hdr->frame_control)) {
    struct ieee80211_mgmt *m = (struct ieee80211_mgmt *)skb->data;
    if (ieee80211_is_beacon (hdr->frame_control)) {
      if (find_eid (m, skb->len, WLAN_EID_SSID, (u8 *)ssid, &act_len)) {
        ssid[act_len] = 0;
        DLOG ("beacon %s", ssid);
        len = make_probe_req_pkt (sa, m->bssid, ssid, rates, 8,
                                  pkt, sizeof (pkt));
        if (len <= sizeof (pkt)) {
          struct sk_buff skb = {
            .data = pkt,
            .len = len,
          };
          local->ops->tx (local_to_hw (local), &skb);
        }
      } else {
        DLOG ("beacon, no SSID");
      }
    } else if (ieee80211_is_probe_req (hdr->frame_control)) {
      if (find_eid (m, skb->len, WLAN_EID_SSID, (u8 *)ssid, &act_len)) {
        ssid[act_len] = 0;
        DLOG ("probe_req %s", ssid);
        if (strncmp (SSID, ssid, act_len) == 0) {
          tx_probe_resp (local);
        }
      } else {
        DLOG ("probe_req, no SSID");
      }
    } else if (ieee80211_is_probe_resp (hdr->frame_control)) {
      if (find_eid (m, skb->len, WLAN_EID_SSID, (u8 *)ssid, &act_len)) {
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
    DLOG ("data for %.02X:%.02X:%.02X:%.02X:%.02X:%.02X",
          hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
          hdr->addr1[3], hdr->addr1[4], hdr->addr1[5]);
  }
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
