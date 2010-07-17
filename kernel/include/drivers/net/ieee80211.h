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

/* Based on Linux */

#ifndef _IEEE80211_H_
#define _IEEE80211_H_

/**
 * enum ieee80211_band - supported frequency bands
 *
 * The bands are assigned this way because the supported
 * bitrates differ in these bands.
 *
 * @IEEE80211_BAND_2GHZ: 2.4GHz ISM band
 * @IEEE80211_BAND_5GHZ: around 5GHz band (4.9-5.7)
 */
enum ieee80211_band {
  IEEE80211_BAND_2GHZ,
  IEEE80211_BAND_5GHZ,

  /* keep last */
  IEEE80211_NUM_BANDS
};

/**
 * enum ieee80211_channel_flags - channel flags
 *
 * Channel flags set by the regulatory control code.
 *
 * @IEEE80211_CHAN_DISABLED: This channel is disabled.
 * @IEEE80211_CHAN_PASSIVE_SCAN: Only passive scanning is permitted
 *      on this channel.
 * @IEEE80211_CHAN_NO_IBSS: IBSS is not allowed on this channel.
 * @IEEE80211_CHAN_RADAR: Radar detection is required on this channel.
 * @IEEE80211_CHAN_NO_HT40PLUS: extension channel above this channel
 *      is not permitted.
 * @IEEE80211_CHAN_NO_HT40MINUS: extension channel below this channel
 *      is not permitted.
 */
enum ieee80211_channel_flags {
  IEEE80211_CHAN_DISABLED               = 1<<0,
  IEEE80211_CHAN_PASSIVE_SCAN   = 1<<1,
  IEEE80211_CHAN_NO_IBSS                = 1<<2,
  IEEE80211_CHAN_RADAR          = 1<<3,
  IEEE80211_CHAN_NO_HT40PLUS    = 1<<4,
  IEEE80211_CHAN_NO_HT40MINUS   = 1<<5,
};

#define IEEE80211_CHAN_NO_HT40                                  \
  (IEEE80211_CHAN_NO_HT40PLUS | IEEE80211_CHAN_NO_HT40MINUS)

/**
 * struct ieee80211_channel - channel definition
 *
 * This structure describes a single channel for use
 * with cfg80211.
 *
 * @center_freq: center frequency in MHz
 * @hw_value: hardware-specific value for the channel
 * @flags: channel flags from &enum ieee80211_channel_flags.
 * @orig_flags: channel flags at registration time, used by regulatory
 *      code to support devices with additional restrictions
 * @band: band this channel belongs to.
 * @max_antenna_gain: maximum antenna gain in dBi
 * @max_power: maximum transmission power (in dBm)
 * @beacon_found: helper to regulatory code to indicate when a beacon
 *      has been found on this channel. Use regulatory_hint_found_beacon()
 *      to enable this, this is is useful only on 5 GHz band.
 * @orig_mag: internal use
 * @orig_mpwr: internal use
 */

struct ieee80211_channel {
  enum ieee80211_band band;
  u16 center_freq;
  u16 hw_value;
  u32 flags;
  int max_antenna_gain;
  int max_power;
  bool beacon_found;
  u32 orig_flags;
  int orig_mag, orig_mpwr;
};

/**
 * enum ieee80211_rate_flags - rate flags
 *
 * Hardware/specification flags for rates. These are structured
 * in a way that allows using the same bitrate structure for
 * different bands/PHY modes.
 *
 * @IEEE80211_RATE_SHORT_PREAMBLE: Hardware can send with short
 *      preamble on this bitrate; only relevant in 2.4GHz band and
 *      with CCK rates.
 * @IEEE80211_RATE_MANDATORY_A: This bitrate is a mandatory rate
 *      when used with 802.11a (on the 5 GHz band); filled by the
 *      core code when registering the wiphy.
 * @IEEE80211_RATE_MANDATORY_B: This bitrate is a mandatory rate
 *      when used with 802.11b (on the 2.4 GHz band); filled by the
 *      core code when registering the wiphy.
 * @IEEE80211_RATE_MANDATORY_G: This bitrate is a mandatory rate
 *      when used with 802.11g (on the 2.4 GHz band); filled by the
 *      core code when registering the wiphy.
 * @IEEE80211_RATE_ERP_G: This is an ERP rate in 802.11g mode.
 */
enum ieee80211_rate_flags {
  IEEE80211_RATE_SHORT_PREAMBLE = 1<<0,
  IEEE80211_RATE_MANDATORY_A    = 1<<1,
  IEEE80211_RATE_MANDATORY_B    = 1<<2,
  IEEE80211_RATE_MANDATORY_G    = 1<<3,
  IEEE80211_RATE_ERP_G          = 1<<4,
};

/**
 * struct ieee80211_rate - bitrate definition
 *
 * This structure describes a bitrate that an 802.11 PHY can
 * operate with. The two values @hw_value and @hw_value_short
 * are only for driver use when pointers to this structure are
 * passed around.
 *
 * @flags: rate-specific flags
 * @bitrate: bitrate in units of 100 Kbps
 * @hw_value: driver/hardware value for this rate
 * @hw_value_short: driver/hardware value for this rate when
 *      short preamble is used
 */
struct ieee80211_rate {
  u32 flags;
  u16 bitrate;
  u16 hw_value, hw_value_short;
};

/**
 * struct ieee80211_bss_conf - holds the BSS's changing parameters
 *
 * This structure keeps information about a BSS (and an association
 * to that BSS) that can change during the lifetime of the BSS.
 *
 * @assoc: association status
 * @aid: association ID number, valid only when @assoc is true
 * @use_cts_prot: use CTS protection
 * @use_short_preamble: use 802.11b short preamble;
 *	if the hardware cannot handle this it must set the
 *	IEEE80211_HW_2GHZ_SHORT_PREAMBLE_INCAPABLE hardware flag
 * @use_short_slot: use short slot time (only relevant for ERP);
 *	if the hardware cannot handle this it must set the
 *	IEEE80211_HW_2GHZ_SHORT_SLOT_INCAPABLE hardware flag
 * @dtim_period: num of beacons before the next DTIM, for PSM
 * @timestamp: beacon timestamp
 * @beacon_int: beacon interval
 * @assoc_capability: capabilities taken from assoc resp
 * @basic_rates: bitmap of basic rates, each bit stands for an
 *	index into the rate table configured by the driver in
 *	the current band.
 * @bssid: The BSSID for this BSS
 * @enable_beacon: whether beaconing should be enabled or not
 * @ht_operation_mode: HT operation mode (like in &struct ieee80211_ht_info).
 *	This field is only valid when the channel type is one of the HT types.
 */
struct ieee80211_bss_conf {
	const u8 *bssid;
	/* association related data */
	bool assoc;
	u16 aid;
	/* erp related data */
	bool use_cts_prot;
	bool use_short_preamble;
	bool use_short_slot;
	bool enable_beacon;
	u8 dtim_period;
	u16 beacon_int;
	u16 assoc_capability;
	u64 timestamp;
	u32 basic_rates;
	u16 ht_operation_mode;
};

/**
 * enum ieee80211_conf_flags - configuration flags
 *
 * Flags to define PHY configuration options
 *
 * @IEEE80211_CONF_RADIOTAP: add radiotap header at receive time (if supported)
 * @IEEE80211_CONF_PS: Enable 802.11 power save mode (managed mode only)
 * @IEEE80211_CONF_IDLE: The device is running, but idle; if the flag is set
 *	the driver should be prepared to handle configuration requests but
 *	may turn the device off as much as possible. Typically, this flag will
 *	be set when an interface is set UP but not associated or scanning, but
 *	it can also be unset in that case when monitor interfaces are active.
 */
enum ieee80211_conf_flags {
	IEEE80211_CONF_RADIOTAP		= (1<<0),
	IEEE80211_CONF_PS		= (1<<1),
	IEEE80211_CONF_IDLE		= (1<<2),
};


/**
 * enum ieee80211_conf_changed - denotes which configuration changed
 *
 * @IEEE80211_CONF_CHANGE_LISTEN_INTERVAL: the listen interval changed
 * @IEEE80211_CONF_CHANGE_RADIOTAP: the radiotap flag changed
 * @IEEE80211_CONF_CHANGE_PS: the PS flag or dynamic PS timeout changed
 * @IEEE80211_CONF_CHANGE_POWER: the TX power changed
 * @IEEE80211_CONF_CHANGE_CHANNEL: the channel/channel_type changed
 * @IEEE80211_CONF_CHANGE_RETRY_LIMITS: retry limits changed
 * @IEEE80211_CONF_CHANGE_IDLE: Idle flag changed
 */
enum ieee80211_conf_changed {
	IEEE80211_CONF_CHANGE_LISTEN_INTERVAL	= BIT(2),
	IEEE80211_CONF_CHANGE_RADIOTAP		= BIT(3),
	IEEE80211_CONF_CHANGE_PS		= BIT(4),
	IEEE80211_CONF_CHANGE_POWER		= BIT(5),
	IEEE80211_CONF_CHANGE_CHANNEL		= BIT(6),
	IEEE80211_CONF_CHANGE_RETRY_LIMITS	= BIT(7),
	IEEE80211_CONF_CHANGE_IDLE		= BIT(8),
};

enum nl80211_channel_type {
	NL80211_CHAN_NO_HT,
	NL80211_CHAN_HT20,
	NL80211_CHAN_HT40MINUS,
	NL80211_CHAN_HT40PLUS
};

/**
 * struct ieee80211_conf - configuration of the device
 *
 * This struct indicates how the driver shall configure the hardware.
 *
 * @flags: configuration flags defined above
 *
 * @listen_interval: listen interval in units of beacon interval
 * @max_sleep_period: the maximum number of beacon intervals to sleep for
 *	before checking the beacon for a TIM bit (managed mode only); this
 *	value will be only achievable between DTIM frames, the hardware
 *	needs to check for the multicast traffic bit in DTIM beacons.
 *	This variable is valid only when the CONF_PS flag is set.
 * @dynamic_ps_timeout: The dynamic powersave timeout (in ms), see the
 *	powersave documentation below. This variable is valid only when
 *	the CONF_PS flag is set.
 *
 * @power_level: requested transmit power (in dBm)
 *
 * @channel: the channel to tune to
 * @channel_type: the channel (HT) type
 *
 * @long_frame_max_tx_count: Maximum number of transmissions for a "long" frame
 *    (a frame not RTS protected), called "dot11LongRetryLimit" in 802.11,
 *    but actually means the number of transmissions not the number of retries
 * @short_frame_max_tx_count: Maximum number of transmissions for a "short"
 *    frame, called "dot11ShortRetryLimit" in 802.11, but actually means the
 *    number of transmissions not the number of retries
 */
struct ieee80211_conf {
	u32 flags;
	int power_level, dynamic_ps_timeout;
	int max_sleep_period;

	u16 listen_interval;

	u8 long_frame_max_tx_count, short_frame_max_tx_count;

	struct ieee80211_channel *channel;
	enum nl80211_channel_type channel_type;
};

/**
 * struct ieee80211_tx_queue_params - transmit queue configuration
 *
 * The information provided in this structure is required for QoS
 * transmit queue configuration. Cf. IEEE 802.11 7.3.2.29.
 *
 * @aifs: arbitration interframe space [0..255]
 * @cw_min: minimum contention window [a value of the form
 *	2^n-1 in the range 1..32767]
 * @cw_max: maximum contention window [like @cw_min]
 * @txop: maximum burst time in units of 32 usecs, 0 meaning disabled
 */
struct ieee80211_tx_queue_params {
	u16 txop;
	u16 cw_min;
	u16 cw_max;
	u8 aifs;
};

static inline int
ieee80211_frequency_to_channel(int freq)
{
  if (freq == 2484)
    return 14;

  if (freq < 2484)
    return (freq - 2407) / 5;

  /* FIXME: 802.11j 17.3.8.3.2 */
  return freq/5 - 1000;
}


/**
 * enum ieee80211_bss_change - BSS change notification flags
 *
 * These flags are used with the bss_info_changed() callback
 * to indicate which BSS parameter changed.
 *
 * @BSS_CHANGED_ASSOC: association status changed (associated/disassociated),
 *	also implies a change in the AID.
 * @BSS_CHANGED_ERP_CTS_PROT: CTS protection changed
 * @BSS_CHANGED_ERP_PREAMBLE: preamble changed
 * @BSS_CHANGED_ERP_SLOT: slot timing changed
 * @BSS_CHANGED_HT: 802.11n parameters changed
 * @BSS_CHANGED_BASIC_RATES: Basic rateset changed
 * @BSS_CHANGED_BEACON_INT: Beacon interval changed
 * @BSS_CHANGED_BSSID: BSSID changed, for whatever
 *	reason (IBSS and managed mode)
 * @BSS_CHANGED_BEACON: Beacon data changed, retrieve
 *	new beacon (beaconing modes)
 * @BSS_CHANGED_BEACON_ENABLED: Beaconing should be
 *	enabled/disabled (beaconing modes)
 */
enum ieee80211_bss_change {
	BSS_CHANGED_ASSOC		= 1<<0,
	BSS_CHANGED_ERP_CTS_PROT	= 1<<1,
	BSS_CHANGED_ERP_PREAMBLE	= 1<<2,
	BSS_CHANGED_ERP_SLOT		= 1<<3,
	BSS_CHANGED_HT                  = 1<<4,
	BSS_CHANGED_BASIC_RATES		= 1<<5,
	BSS_CHANGED_BEACON_INT		= 1<<6,
	BSS_CHANGED_BSSID		= 1<<7,
	BSS_CHANGED_BEACON		= 1<<8,
	BSS_CHANGED_BEACON_ENABLED	= 1<<9,
};

#endif

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
