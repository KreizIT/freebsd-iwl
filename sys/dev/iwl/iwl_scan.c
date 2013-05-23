/*-
 * Copyright (c) 2011 Intel Corporation
 * Copyright (c) 2007-2009
 *	Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2008
 *	Benjamin Close <benjsc@FreeBSD.org>
 * Copyright (c) 2008 Sam Leffler, Errno Consulting
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "iwl_common.h"

/*
 * Callback from net80211 to start a scan.
 */
void
iwl_scan_start(struct ieee80211com *ic)
{
	struct ifnet *ifp = ic->ic_ifp;
	struct iwl_softc *sc = ifp->if_softc;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	struct ieee80211vap *vap = ss->ss_vap;

	IWL_LOCK(sc);

	sc->uc_scan_progress = 1;
	if(sc->ctx == IWL_RXON_PAN_CTX)
		iwl_set_pan_params(sc);

	/* make the link LED blink while we're scanning */
	if (vap->iv_state  != IEEE80211_S_RUN) {
		if(sc->sc_led.led_cur_mode != IWL_LED_SLOW_BLINK) {
			sc->sc_led.led_cur_mode = IWL_LED_SLOW_BLINK;
			iwl_set_led(sc, IWL_LED_LINK, 190, 10,IWL_LED_SLOW_BLINK);
		}
	}
	IWL_UNLOCK(sc);
}

/*
 * Callback from net80211 to terminate a scan.
 */
void
iwl_scan_end(struct ieee80211com *ic)
{
	struct ifnet *ifp = ic->ic_ifp;
	struct iwl_softc *sc = ifp->if_softc;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	struct ieee80211vap *vap = ss->ss_vap;

	IWL_LOCK(sc);
	sc->uc_scan_progress = 0;
	sc->sc_scan_timer = 0;
	if(sc->ctx == IWL_RXON_PAN_CTX)
		iwl_set_pan_params(sc);

	/* Link LED always on while associated. */
	if(vap->iv_state == IEEE80211_S_RUN) {
		sc->sc_led.led_cur_mode = IWL_LED_STATIC_ON;
		iwl_set_led(sc, IWL_LED_LINK, 0, 1,IWL_LED_STATIC_ON);
	}
	IWL_UNLOCK(sc);
}

/*
 * Callback from net80211 to start scanning of the current channel.
 */
void
iwl_scan_curchan(struct ieee80211_scan_state *ss, unsigned long maxdwell)
{
	struct ieee80211vap *vap = ss->ss_vap;
	struct iwl_softc *sc = vap->iv_ic->ic_ifp->if_softc;
	int error;

	IWL_LOCK(sc);
	sc->sc_scan_timer = 0;
	error = iwl_scan(sc);
	IWL_UNLOCK(sc);
	if (error != 0) {
		sc->uc_scan_progress = 0;
		sc->sc_scan_timer = 0;
		ieee80211_cancel_scan(vap);
	}
}

int
iwl_scan(struct iwl_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	struct ieee80211_node *ni = ss->ss_vap->iv_bss;
	struct iwl_scan_hdr *hdr;
	struct iwl_cmd_data *tx;
	struct iwl_scan_essid *essid;
	struct iwl_scan_chan *chan;
	struct ieee80211_frame *wh;
	struct ieee80211_rateset *rs;
	struct ieee80211_channel *c;
	uint8_t *buf, *frm;
	uint16_t rxchain;
	uint8_t txant;
	int buflen, error;
	uint8_t new_scan_threshold;
	struct iwl_eeprom_chan *channel;

	struct ieee80211vap *vap = ni->ni_vap;
	struct iwl_vap *ivp = IWL_VAP(vap);

	if(ivp->ctx == IWL_RXON_BSS_CTX)
		sc->rxon = &sc->rx_on[IWL_RXON_BSS_CTX];
	else if(ivp->ctx == IWL_RXON_PAN_CTX)
		sc->rxon = &sc->rx_on[IWL_RXON_PAN_CTX];

	buf = malloc(IWL_SCAN_MAXSZ, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (buf == NULL) {
		device_printf(sc->sc_dev,
		    "%s: could not allocate buffer for scan command\n",
		    __func__);
		return ENOMEM;
	}
	hdr = (struct iwl_scan_hdr *)buf;
	/*
	 * Move to the next channel if no frames are received within 10ms
	 * after sending the probe request.
	 */
	hdr->quiet_time = htole16(10);		/* timeout in milliseconds */
	hdr->quiet_threshold = htole16(1);	/* min # of packets */

	/* Select antennas for scanning. */
	rxchain =
	    IWL_RXCHAIN_VALID(sc->rxchainmask) |
	    IWL_RXCHAIN_FORCE_MIMO_SEL(sc->rxchainmask) |
	    IWL_RXCHAIN_DRIVER_FORCE;
	if (IEEE80211_IS_CHAN_A(ic->ic_curchan) &&
	    sc->hw_type == IWL_HW_REV_TYPE_4965) {
		/* Ant A must be avoided in 5GHz because of an HW bug. */
		rxchain |= IWL_RXCHAIN_FORCE_SEL(IWL_ANT_B);
	} else	/* Use all available RX antennas. */
		rxchain |= IWL_RXCHAIN_FORCE_SEL(sc->rxchainmask);
	hdr->rxchain = htole16(rxchain);
	hdr->filter = htole32(IWL_FILTER_MULTICAST | IWL_FILTER_BEACON);

	tx = (struct iwl_cmd_data *)(hdr + 1);
	tx->flags = htole32(IWL_TX_AUTO_SEQ);
	if(ivp->ctx == IWL_RXON_PAN_CTX)
		tx->id = IWL_PAN_BCAST_ID;
	else
		tx->id = IWL_BROADCAST_ID;
	tx->lifetime = htole32(IWL_LIFETIME_INFINITE);

	if (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan)) {
		/* Send probe requests at 6Mbps. */
		tx->rate = htole32(0xd);
		rs = &ic->ic_sup_rates[IEEE80211_MODE_11A];
	} else {
		hdr->flags = htole32(IWL_RXON_24GHZ | IWL_RXON_AUTO);
		if (sc->hw_type == IWL_HW_REV_TYPE_4965 &&
		    sc->rxon->associd && sc->rxon->chan > 14)
			tx->rate = htole32(0xd);
		else {
			/* Send probe requests at 1Mbps. */
			tx->rate = htole32(10 | IWL_RFLAG_CCK);
		}
		rs = &ic->ic_sup_rates[IEEE80211_MODE_11G];
	}
	/* Use the first valid TX antenna. */
	txant = IWL_LSB(sc->txchainmask);
	tx->rate |= htole32(IWL_RFLAG_ANT(txant));

	essid = (struct iwl_scan_essid *)(tx + 1);
	if (ss->ss_ssid[0].len != 0) {
		essid[0].id = IEEE80211_ELEMID_SSID;
		essid[0].len = ss->ss_ssid[0].len;
		memcpy(essid[0].data, ss->ss_ssid[0].ssid, ss->ss_ssid[0].len);
	}
	/*
	 * Build a probe request frame.  Most of the following code is a
	 * copy & paste of what is done in net80211.
	 */
	wh = (struct ieee80211_frame *)(essid + 20);
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_PROBE_REQ;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	IEEE80211_ADDR_COPY(wh->i_addr1, ifp->if_broadcastaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, ivp->macaddr);

	IEEE80211_ADDR_COPY(wh->i_addr3, ifp->if_broadcastaddr);
	*(uint16_t *)&wh->i_dur[0] = 0;	/* filled by HW */
	*(uint16_t *)&wh->i_seq[0] = 0;	/* filled by HW */

	frm = (uint8_t *)(wh + 1);
	frm = ieee80211_add_ssid(frm, NULL, 0);
	frm = ieee80211_add_rates(frm, rs);
	if (rs->rs_nrates > IEEE80211_RATE_SIZE)
		frm = ieee80211_add_xrates(frm, rs);
	if (ic->ic_htcaps & IEEE80211_HTC_HT)
		frm = ieee80211_add_htcap(frm, ni);

	/* Set length of probe request. */
	tx->len = htole16(frm - (uint8_t *)wh);

	c = ic->ic_curchan;
	chan = (struct iwl_scan_chan *)frm;
	chan->chan = htole16(ieee80211_chan2ieee(ic, c));
	chan->flags = 0;

	if (ss->ss_nssid > 0)
		chan->flags |= htole32(IWL_CHAN_NPBREQS(1));

	/*
	 * If active scanning is requested but a certain channel is
	 * marked passive, we can do active scanning if we detect
	 * transmissions.
	 *
	 * There is an issue with some firmware versions that triggers
	 * a sysassert on a "good CRC threshold" of zero (== disabled),
	 * on a radar channel even though this means that we should NOT
	 * send probes.
	 *
	 * The "good CRC threshold" is the number of frames that we
	 * need to receive during our dwell time on a channel before
	 * sending out probes -- setting this to a huge value will
	 * mean we never reach it, but at the same time work around
	 * the aforementioned issue. Thus use IWL_SCAN_CRC_TH_NEVER
	 * here instead of IWL_SCAN_CRC_TH_DISABLED.
	 *
	 * This was fixed in later versions along with some other
	 * scan changes, and the threshold behaves as a flag in those
	 * versions.
	 */

	channel = iwl_find_eeprom_channel(sc, c);
	if (channel == NULL) {
		if_printf(ic->ic_ifp,
		    "%s: invalid channel %u freq %u/0x%x\n",
		    __func__, c->ic_ieee, c->ic_freq, c->ic_flags);
		return EINVAL;
	}

	new_scan_threshold = ((sc->tlv_feature_flags &
	    (1<<IWL_FW_TLV_FLAGS_NEW_SCAN_BITPOS)) >> IWL_FW_TLV_FLAGS_NEW_SCAN_BITPOS);

	/* Selection criteria for Active/Passive scanning */
	if ((ss->ss_nssid == 0) || ((channel->flags & IWL_EEPROM_CHAN_ACTIVE) == 0) ||
	    (c->ic_flags & IEEE80211_CHAN_PASSIVE)) {
		chan->flags |= htole32(IWL_CHAN_PASSIVE);
		if (new_scan_threshold == 1)
			hdr->crc_threshold = IWL_SCAN_CRC_TH_DISABLED;
		else
			hdr->crc_threshold = IWL_SCAN_CRC_TH_NEVER;
	} else {
		chan->flags |= htole32(IWL_CHAN_ACTIVE);
		hdr->crc_threshold = IWL_SCAN_CRC_TH_DEFAULT;
	}

	chan->dsp_gain = 0x6e;
	if (IEEE80211_IS_CHAN_5GHZ(c))
		chan->rf_gain = 0x3b;
	else
		chan->rf_gain = 0x28;

	chan->active  = htole16(iwl_get_active_dwell(sc, c));
	chan->passive = htole16(iwl_get_passive_dwell(sc, c));

	DPRINTF(sc, IWL_DEBUG_STATE,
	    "%s: chan %u flags 0x%x rf_gain 0x%x "
	    "dsp_gain 0x%x active 0x%x passive 0x%x\n", __func__,
	    chan->chan, chan->flags, chan->rf_gain, chan->dsp_gain,
	    chan->active, chan->passive);

	hdr->nchan++;
	chan++;
	buflen = (uint8_t *)chan - buf;
	hdr->len = htole16(buflen);

	DPRINTF(sc, IWL_DEBUG_STATE, "sending scan command nchan=%d\n",
	    hdr->nchan);

	sc->sc_scan_timer = IWL_SCAN_CHAN_TIMEOUT;
	error = iwl_cmd(sc, IWL_CMD_SCAN, buf, buflen, 0);

	free(buf, M_DEVBUF);

	return error;
}

/*
 * Add an ssid element to a frame.
 */
uint8_t *
ieee80211_add_ssid(uint8_t *frm, const uint8_t *ssid, u_int len)
{
	*frm++ = IEEE80211_ELEMID_SSID;
	*frm++ = len;
	memcpy(frm, ssid, len);
	return frm + len;
}

/*
 * Callback from net80211 to handle the minimum dwell time being met.
 * The intent is to terminate the scan but we just let the firmware
 * notify us when it's finished as we have no safe way to abort it.
 */
void
iwl_scan_mindwell(struct ieee80211_scan_state *ss)
{
	/* Nothing as of now. */
	/* NB: don't try to abort scan; wait for firmware to finish */
}

uint16_t
iwl_get_active_dwell(struct iwl_softc *sc, struct ieee80211_channel *c)
{
	int n_probes = 1;

	if (IEEE80211_IS_CHAN_5GHZ(c))
		return IWL_ACTIVE_DWELL_TIME_52 +
			IWL_ACTIVE_DWELL_FACTOR_52 * (n_probes + 1);
	else
		return IWL_ACTIVE_DWELL_TIME_24 +
			IWL_ACTIVE_DWELL_FACTOR_24 * (n_probes + 1);
}

uint16_t
iwl_get_passive_dwell(struct iwl_softc *sc, struct ieee80211_channel *c)
{
	uint8_t ctx_id = 0;
	uint16_t beacon_int;
	struct ieee80211vap *vap;
	struct iwl_vap *ivp;
	uint16_t dwell_time;

	dwell_time = (IEEE80211_IS_CHAN_2GHZ(c)) ?
			    IWL_PASSIVE_DWELL_BASE + IWL_PASSIVE_DWELL_TIME_24 :
			    IWL_PASSIVE_DWELL_BASE + IWL_PASSIVE_DWELL_TIME_52;

	for (ctx_id = 0; ctx_id < IWL_NUM_RXON_CTX; ctx_id++) {
		vap = sc->ivap[ctx_id];
		ivp = IWL_VAP(vap);
		sc->rxon = &sc->rx_on[ctx_id];
		if (!sc->rxon->associd)
			continue;
		beacon_int = ivp->beacon_int;
		if (!beacon_int)
			beacon_int = IWL_PASSIVE_DWELL_BASE;
		beacon_int = (beacon_int * 98) / 100 - IWL_CHANNEL_TUNE_TIME * 2;
		dwell_time = min(beacon_int, dwell_time);
		if(sc->ctx != IWL_RXON_PAN_CTX) break;
	}

	return dwell_time;
}
