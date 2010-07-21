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
#include <mem/pow2.h>

#define MAX_MACS 4

struct ieee80211_local {
  struct ieee80211_hw hw;
  const struct ieee80211_ops *ops;
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

  return TRUE;
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
