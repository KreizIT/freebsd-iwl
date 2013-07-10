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

/*
 * Driver for Intel WiFi Link 6000 Series 802.11 network adapters.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "iwl_common.h"

struct iwl_ident {
	uint16_t	vendor;
	uint16_t	device;
	const char	*name;
};

static int	iwl5000_attach(struct iwl_softc *, uint16_t);
static void	iwl_radiotap_attach(struct iwl_softc *);
static void	iwl_sysctlattach(struct iwl_softc *);
static struct	ieee80211vap *iwl_vap_create(struct ieee80211com *,
		    const char name[IFNAMSIZ], int unit, enum ieee80211_opmode opmode,
		    int flags, const uint8_t bssid[IEEE80211_ADDR_LEN],
		    const uint8_t mac[IEEE80211_ADDR_LEN]);
static void	iwl_vap_delete(struct ieee80211vap *);

static int	iwl_eeprom_lock(struct iwl_softc *);
static int	iwl_init_otprom(struct iwl_softc *);
static int	iwl_read_prom_data(struct iwl_softc *, uint32_t, void *, int);
static int	iwl_alloc_ict(struct iwl_softc *);
static void	iwl_free_ict(struct iwl_softc *);
static int	iwl_alloc_fwmem(struct iwl_softc *);
static void	iwl_free_fwmem(struct iwl_softc *);
static int	iwl_alloc_rx_ring(struct iwl_softc *, struct iwl_rx_ring *);
static void	iwl_reset_rx_ring(struct iwl_softc *, struct iwl_rx_ring *);
static void	iwl_free_rx_ring(struct iwl_softc *, struct iwl_rx_ring *);
static void	iwl5000_ict_reset(struct iwl_softc *);
static int	iwl_read_eeprom(struct iwl_softc *,
		    uint8_t macaddr[IEEE80211_ADDR_LEN]);
static void	iwl5000_read_eeprom(struct iwl_softc *);
static uint32_t	iwl_eeprom_channel_flags(struct iwl_eeprom_chan *);
static void	iwl_read_eeprom_band(struct iwl_softc *, int);
static void	iwl_read_eeprom_ht40(struct iwl_softc *, int);
static void	iwl_read_eeprom_channels(struct iwl_softc *, int, uint32_t);
static int	iwl_setregdomain(struct ieee80211com *,
		    struct ieee80211_regdomain *, int,
		    struct ieee80211_channel[]);
static void	iwl_read_eeprom_enhinfo(struct iwl_softc *);
static struct	ieee80211_node *iwl_node_alloc(struct ieee80211vap *,
		    const uint8_t mac[IEEE80211_ADDR_LEN]);
static void	iwl_newassoc(struct ieee80211_node *, int);
static int	iwl_media_change(struct ifnet *);
static int	iwl_newstate(struct ieee80211vap *, enum ieee80211_state, int);
static int	iwl_newstate1(struct ieee80211vap *, enum ieee80211_state, int);
void		iwl_calib_timeout(void *);
static void	iwl_rx_phy(struct iwl_softc *, struct iwl_rx_desc *,
		    struct iwl_rx_data *);
static void	iwl_rx_done(struct iwl_softc *, struct iwl_rx_desc *,
		    struct iwl_rx_data *);
static void	iwl_rx_compressed_ba(struct iwl_softc *, struct iwl_rx_desc *,
		    struct iwl_rx_data *);
static void	iwl5000_rx_calib_results(struct iwl_softc *,
		    struct iwl_rx_desc *, struct iwl_rx_data *);
static void	iwl_rx_statistics(struct iwl_softc *, struct iwl_rx_desc *,
		    struct iwl_rx_data *);
static void	iwl5000_tx_done(struct iwl_softc *, struct iwl_rx_desc *,
		    struct iwl_rx_data *);
static void	iwl_tx_done(struct iwl_softc *, struct iwl_rx_desc *,
			int, uint8_t);
static void	iwl_ampdu_tx_done(struct iwl_softc *, int, int, int, void *);
static void	iwl_cmd_done(struct iwl_softc *, struct iwl_rx_desc *);
static void	iwl_notif_intr(struct iwl_softc *);
static void	iwl_wakeup_intr(struct iwl_softc *);
static void	iwl_rftoggle_intr(struct iwl_softc *);
static void	iwl_fatal_intr(struct iwl_softc *);
static void	iwl_start(struct ifnet *);
static void	iwl_start_locked(struct ifnet *);
static void	iwl_watchdog(void *);
static int	iwl_ioctl(struct ifnet *, u_long, caddr_t);
static int	iwl5000_add_node(struct iwl_softc *, struct iwl_node_info *,
		    int);
static int	iwl_add_broadcast_node(struct iwl_softc *, int);
static int	iwl_add_broadcast_node1(struct iwl_softc *, int);
static int	iwl_updateedca(struct ieee80211com *);
static int	iwl_updateedca1(struct ieee80211com *);
static void	iwl_update_mcast(struct ifnet *);
static int	iwl_set_critical_temp(struct iwl_softc *);
static int	iwl_set_timing(struct iwl_softc *, struct ieee80211_node *);
static int	iwl_set_timing1(struct iwl_softc *);
static int	iwl5000_set_txpower(struct iwl_softc *,
		    struct ieee80211_channel *, int);
static int	iwl5000_get_rssi(struct iwl_softc *, struct iwl_rx_stat *);
static int	iwl_get_noise(const struct iwl_rx_general_stats *);
static void	iwl5000_get_temperature(struct iwl_softc *);
int		iwl_init_sensitivity(struct iwl_softc *);
static void	iwl_collect_noise(struct iwl_softc *,
		    const struct iwl_rx_general_stats *);
static int	iwl5000_init_gains(struct iwl_softc *);
static int	iwl5000_set_gains(struct iwl_softc *);
static void	iwl_tune_sensitivity(struct iwl_softc *,
		    const struct iwl_rx_stats *);
static int	iwl_send_sensitivity(struct iwl_softc *);
static int	iwl_send_btcoex(struct iwl_softc *);
static int	iwl_send_advanced_btcoex(struct iwl_softc *);
static int	iwl_config(struct iwl_softc *);
static int	iwl_config1(struct iwl_softc *);
static int	iwl_auth(struct iwl_softc *, struct ieee80211vap *vap);
static int	iwl_auth1(struct iwl_softc *, struct ieee80211vap *vap);
static int	iwl_run(struct iwl_softc *, struct ieee80211vap *vap);
static int	iwl_run1(struct iwl_softc *, struct ieee80211vap *vap);
static int	iwl_ampdu_rx_start(struct ieee80211_node *,
		    struct ieee80211_rx_ampdu *, int, int, int);
static void	iwl_ampdu_rx_stop(struct ieee80211_node *,
		    struct ieee80211_rx_ampdu *);
static int	iwl_addba_request(struct ieee80211_node *,
		    struct ieee80211_tx_ampdu *, int, int, int);
static int	iwl_addba_response(struct ieee80211_node *,
		    struct ieee80211_tx_ampdu *, int, int, int);
static int	iwl_ampdu_tx_start(struct ieee80211com *,
		    struct ieee80211_node *, uint8_t);
static void	iwl_ampdu_tx_stop(struct ieee80211_node *,
		    struct ieee80211_tx_ampdu *);
static void	iwl5000_ampdu_tx_start(struct iwl_softc *,
		    struct ieee80211_node *, int, uint8_t, uint16_t);
static void	iwl5000_ampdu_tx_stop(struct iwl_softc *, int,
		    uint8_t, uint16_t);
static int	iwl5000_query_calibration(struct iwl_softc *);
static int	iwl5000_send_calibration(struct iwl_softc *);
static int	iwl5000_send_wimax_coex(struct iwl_softc *);
static int	iwl5000_crystal_calib(struct iwl_softc *);
static int	iwl5000_temp_offset_calib(struct iwl_softc *);
static int	iwl5000_post_alive(struct iwl_softc *);
static int	iwl5000_load_firmware_section(struct iwl_softc *, uint32_t,
		    const uint8_t *, int);
static int	iwl5000_load_firmware(struct iwl_softc *);
static int	iwl_read_firmware_leg(struct iwl_softc *,
		    struct iwl_fw_info *);
static int	iwl_read_firmware_tlv(struct iwl_softc *,
		    struct iwl_fw_info *, uint16_t);
static int	iwl_read_firmware(struct iwl_softc *);

static void	iwl_apm_stop_master(struct iwl_softc *);

static int	iwl5000_nic_config(struct iwl_softc *);
static int	iwl_hw_prepare(struct iwl_softc *);
static int	iwl_hw_init(struct iwl_softc *);
static void	iwl_hw_stop(struct iwl_softc *);
static void	iwl_radio_on(void *, int);
static void	iwl_radio_off(void *, int);
static void	iwl_init_locked(struct iwl_softc *);
static void	iwl_init(void *);
static void	iwl_stop_locked(struct iwl_softc *);
static void	iwl_stop(struct iwl_softc *);
static void	iwl_set_channel(struct ieee80211com *);
static void	iwl_hw_reset(void *, int);

#ifdef IWL_DEBUG

const char *
iwl_intr_str(uint8_t cmd)
{
	switch (cmd) {
	/* Notifications */
	case IWL_UC_READY:		return "UC_READY";
	case IWL_ADD_NODE_DONE:		return "ADD_NODE_DONE";
	case IWL_TX_DONE:		return "TX_DONE";
	case IWL_START_SCAN:		return "START_SCAN";
	case IWL_STOP_SCAN:		return "STOP_SCAN";
	case IWL_RX_STATISTICS:		return "RX_STATS";
	case IWL_BEACON_STATISTICS:	return "BEACON_STATS";
	case IWL_STATE_CHANGED:		return "STATE_CHANGED";
	case IWL_BEACON_MISSED:		return "BEACON_MISSED";
	case IWL_RX_PHY:		return "RX_PHY";
	case IWL_MPDU_RX_DONE:		return "MPDU_RX_DONE";
	case IWL_RX_DONE:		return "RX_DONE";
	case IWL_TEMP_NOTIFICATION: return "IWL_TEMP_NOTIFICATION";

	/* Command Notifications */
	case IWL_CMD_RXON:		return "IWL_CMD_RXON";
	case IWL_CMD_RXON_ASSOC:	return "IWL_CMD_RXON_ASSOC";
	case IWL_CMD_EDCA_PARAMS:	return "IWL_CMD_EDCA_PARAMS";
	case IWL_CMD_TIMING:		return "IWL_CMD_TIMING";
	case IWL_CMD_LINK_QUALITY:	return "IWL_CMD_LINK_QUALITY";
	case IWL_CMD_SET_LED:		return "IWL_CMD_SET_LED";
	case IWL5000_CMD_WIMAX_COEX:	return "IWL5000_CMD_WIMAX_COEX";
	case IWL5000_CMD_CALIB_CONFIG:	return "IWL5000_CMD_CALIB_CONFIG";
	case IWL5000_CMD_CALIB_RESULT:	return "IWL5000_CMD_CALIB_RESULT";
	case IWL5000_CMD_CALIB_COMPLETE: return "IWL5000_CMD_CALIB_COMPLETE";
	case IWL_CMD_SET_POWER_MODE:	return "IWL_CMD_SET_POWER_MODE";
	case IWL_CMD_SCAN:		return "IWL_CMD_SCAN";
	case IWL_CMD_SCAN_RESULTS:	return "IWL_CMD_SCAN_RESULTS";
	case IWL_CMD_TXPOWER:		return "IWL_CMD_TXPOWER";
	case IWL_CMD_TXPOWER_DBM:	return "IWL_CMD_TXPOWER_DBM";
	case IWL5000_CMD_TX_ANT_CONFIG:	return "IWL5000_CMD_TX_ANT_CONFIG";
	case IWL_CMD_BT_COEX:		return "IWL_CMD_BT_COEX";
	case IWL_CMD_SET_CRITICAL_TEMP:	return "IWL_CMD_SET_CRITICAL_TEMP";
	case IWL_CMD_SET_SENSITIVITY:	return "IWL_CMD_SET_SENSITIVITY";
	case IWL_CMD_PHY_CALIB:		return "IWL_CMD_PHY_CALIB";
	}
	return "UNKNOWN INTR NOTIF/CMD";
}

#endif

int
iwl_attach(device_t dev)
{
	struct iwl_softc *sc = (struct iwl_softc *)device_get_softc(dev);
	struct ieee80211com *ic;
	struct ifnet *ifp;
	int i, error;
	uint8_t macaddr[IEEE80211_ADDR_LEN];

	sc->desired_pwrsave_level = IWL_POWERSAVE_LVL_DEFAULT;
	sc->current_pwrsave_level = -1;  /* signifies uninitialized */

	sc->sc_dev = dev;

	IWL_LOCK_INIT(sc);

	/* Read hardware revision and attach. */
	sc->hw_type = (IWL_READ(sc, IWL_HW_REV) >> 4) & 0xff;
	error = iwl5000_attach(sc, pci_get_device(dev));
	if (error != 0) {
		device_printf(dev, "could not attach device, error %d\n",
		    error);
		goto fail;
	}

	if ((error = iwl_hw_prepare(sc)) != 0) {
		device_printf(dev, "hardware not ready, error %d\n", error);
		goto fail;
	}

	/* Allocate DMA memory for firmware transfers. */
	if ((error = iwl_alloc_fwmem(sc)) != 0) {
		device_printf(dev,
		    "could not allocate memory for firmware, error %d\n",
		    error);
		goto fail;
	}

	/* Allocate "Keep Warm" page. */
	if ((error = iwl_alloc_kw(sc)) != 0) {
		device_printf(dev,
		    "could not allocate keep warm page, error %d\n", error);
		goto fail;
	}

	/* Allocate ICT table for 5000 Series. */
	if (sc->hw_type != IWL_HW_REV_TYPE_4965 &&
	    (error = iwl_alloc_ict(sc)) != 0) {
		device_printf(dev, "could not allocate ICT table, error %d\n",
		    error);
		goto fail;
	}

	/* Allocate TX scheduler "rings". */
	if ((error = iwl_alloc_sched(sc)) != 0) {
		device_printf(dev,
		    "could not allocate TX scheduler rings, error %d\n", error);
		goto fail;
	}

	/* Allocate TX rings (16 on 4965AGN, 20 on >=5000). */
	for (i = 0; i < sc->ntxqs; i++) {
		if ((error = iwl_alloc_tx_ring(sc, &sc->txq[i], i)) != 0) {
			device_printf(dev,
			    "could not allocate TX ring %d, error %d\n", i,
			    error);
			goto fail;
		}
	}

	/* Allocate RX ring. */
	if ((error = iwl_alloc_rx_ring(sc, &sc->rxq)) != 0) {
		device_printf(dev, "could not allocate RX ring, error %d\n",
		    error);
		goto fail;
	}

	/* Clear pending interrupts. */
	IWL_WRITE(sc, IWL_INT, 0xffffffff);

	ifp = sc->sc_ifp = if_alloc(IFT_IEEE80211);
	if (ifp == NULL) {
		device_printf(dev, "can not allocate ifnet structure\n");
		goto fail;
	}

	ic = ifp->if_l2com;
	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */

	/* Set device capabilities. */
	ic->ic_caps =
		  IEEE80211_C_STA		/* station mode supported */
		| IEEE80211_C_MONITOR		/* monitor mode supported */
		| IEEE80211_C_BGSCAN		/* background scanning */
		| IEEE80211_C_TXPMGT		/* tx power management */
		| IEEE80211_C_SHSLOT		/* short slot time supported */
		| IEEE80211_C_WPA
		| IEEE80211_C_SHPREAMBLE	/* short preamble supported */
#if 0
		| IEEE80211_C_IBSS		/* ibss/adhoc mode */
#endif
		| IEEE80211_C_WME		/* WME */
		| IEEE80211_C_PMGT		/* power management */
		;
		if (sc->hw_type == IWL_HW_REV_TYPE_1000 || sc->hw_type == IWL_HW_REV_TYPE_6000) {
			ic->ic_caps &= ~IEEE80211_C_HOSTAP ;/* HOSTAP mode not supported  */
		}
		else
		{
			ic->ic_caps |=IEEE80211_C_HOSTAP ; /* HOSTAP mode supported */
		}
	/* Read MAC address, channels, etc from EEPROM. */
	if ((error = iwl_read_eeprom(sc, macaddr)) != 0) {
		device_printf(dev, "could not read EEPROM, error %d\n",
		    error);
		goto fail;
	}
	/* XXX: initialize thermal throttling */
	/* Count the number of available chains. */
	sc->ntxchains =
	    ((sc->txchainmask >> 2) & 1) +
	    ((sc->txchainmask >> 1) & 1) +
	    ((sc->txchainmask >> 0) & 1);
	sc->nrxchains =
	    ((sc->rxchainmask >> 2) & 1) +
	    ((sc->rxchainmask >> 1) & 1) +
	    ((sc->rxchainmask >> 0) & 1);
	if (bootverbose) {
		device_printf(dev, "MIMO %dT%dR, %.4s, address %6D\n",
		    sc->ntxchains, sc->nrxchains, sc->eeprom_domain,
		    macaddr, ":");
	}

	if (sc->sc_flags & IWL_FLAG_HAS_11N) {
		ic->ic_rxstream = sc->nrxchains;
		ic->ic_txstream = sc->ntxchains;
		ic->ic_htcaps =
			  IEEE80211_HTCAP_SMPS_OFF	/* SMPS mode disabled */
			| IEEE80211_HTCAP_SHORTGI20	/* short GI in 20MHz */
			| IEEE80211_HTCAP_CHWIDTH40	/* 40MHz channel width*/
			| IEEE80211_HTCAP_SHORTGI40	/* short GI in 40MHz */
#ifdef notyet
			| IEEE80211_HTCAP_GREENFIELD
#if IWL_RBUF_SIZE == 8192
			| IEEE80211_HTCAP_MAXAMSDU_7935	/* max A-MSDU length */
#else
			| IEEE80211_HTCAP_MAXAMSDU_3839	/* max A-MSDU length */
#endif
#endif
			/* s/w capabilities */
			| IEEE80211_HTC_HT		/* HT operation */
			| IEEE80211_HTC_AMPDU		/* tx A-MPDU */
#ifdef notyet
			| IEEE80211_HTC_AMSDU		/* tx A-MSDU */
#endif
			;
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_init = iwl_init;
	ifp->if_ioctl = iwl_ioctl;
	ifp->if_start = iwl_start;
	IFQ_SET_MAXLEN(&ifp->if_snd, ifqmaxlen);
	ifp->if_snd.ifq_drv_maxlen = ifqmaxlen;
	IFQ_SET_READY(&ifp->if_snd);

	ieee80211_ifattach(ic, macaddr);
	ic->ic_vap_create = iwl_vap_create;
	ic->ic_vap_delete = iwl_vap_delete;
	ic->ic_raw_xmit = iwl_raw_xmit;
	ic->ic_node_alloc = iwl_node_alloc;
	sc->sc_ampdu_rx_start = ic->ic_ampdu_rx_start;
	ic->ic_ampdu_rx_start = iwl_ampdu_rx_start;
	sc->sc_ampdu_rx_stop = ic->ic_ampdu_rx_stop;
	ic->ic_ampdu_rx_stop = iwl_ampdu_rx_stop;
	sc->sc_addba_request = ic->ic_addba_request;
	ic->ic_addba_request = iwl_addba_request;
	sc->sc_addba_response = ic->ic_addba_response;
	ic->ic_addba_response = iwl_addba_response;
	sc->sc_addba_stop = ic->ic_addba_stop;
	ic->ic_addba_stop = iwl_ampdu_tx_stop;
	ic->ic_newassoc = iwl_newassoc;
	ic->ic_wme.wme_update = iwl_updateedca;
	ic->ic_update_mcast = iwl_update_mcast;
	ic->ic_scan_start = iwl_scan_start;
	ic->ic_scan_end = iwl_scan_end;
	ic->ic_set_channel = iwl_set_channel;
	ic->ic_scan_curchan = iwl_scan_curchan;
	ic->ic_scan_mindwell = iwl_scan_mindwell;
	ic->ic_setregdomain = iwl_setregdomain;

	sc->sc_led.led_cur_mode = IWL_LED_STATIC_OFF;
	sc->sc_led.led_cur_tpt = 0;
	sc->sc_led.led_last_tpt = 0;
	sc->sc_led.led_cur_time = 0;
	sc->sc_led.led_last_time = 0;

	iwl_radiotap_attach(sc);

	callout_init_mtx(&sc->calib_to, &sc->sc_mtx, 0);
	callout_init_mtx(&sc->watchdog_to, &sc->sc_mtx, 0);
	callout_init_mtx(&sc->ct_kill_exit_to, &sc->sc_mtx, 0);
	TASK_INIT(&sc->sc_reinit_task, 0, iwl_hw_reset, sc);
	TASK_INIT(&sc->sc_radioon_task, 0, iwl_radio_on, sc);
	TASK_INIT(&sc->sc_radiooff_task, 0, iwl_radio_off, sc);

	iwl_sysctlattach(sc);

	if (bootverbose)
		ieee80211_announce(ic);

	/* update ic->ic_flags to the default power save mode */
	if (IWL_POWERSAVE_LVL_DEFAULT != IWL_POWERSAVE_LVL_NONE)
		ic->ic_flags |= IEEE80211_F_PMGTON;
	else
		ic->ic_flags &= ~IEEE80211_F_PMGTON;

	return 0;
fail:
	iwl_detach(dev);
	return error;
}

static int
iwl5000_attach(struct iwl_softc *sc, uint16_t pid)
{
	struct iwl_ops *ops = &sc->ops;

	ops->load_firmware = iwl5000_load_firmware;
	ops->read_eeprom = iwl5000_read_eeprom;
	ops->post_alive = iwl5000_post_alive;
	ops->nic_config = iwl5000_nic_config;
	ops->update_sched = iwl5000_update_sched;
	ops->get_temperature = iwl5000_get_temperature;
	ops->get_rssi = iwl5000_get_rssi;
	ops->set_txpower = iwl5000_set_txpower;
	ops->init_gains = iwl5000_init_gains;
	ops->set_gains = iwl5000_set_gains;
	ops->add_node = iwl5000_add_node;
	ops->tx_done = iwl5000_tx_done;
	ops->ampdu_tx_start = iwl5000_ampdu_tx_start;
	ops->ampdu_tx_stop = iwl5000_ampdu_tx_stop;
	sc->ntxqs = IWL5000_NTXQUEUES;
	sc->firstaggqueue = IWL5000_FIRSTAGGQUEUE;
	sc->ndmachnls = IWL5000_NDMACHNLS;
	sc->broadcast_id = IWL5000_ID_BROADCAST;
	sc->rxonsz = IWL5000_RXONSZ;
	sc->schedsz = IWL5000_SCHEDSZ;
	sc->fw_text_maxsz = IWL5000_FW_TEXT_MAXSZ;
	sc->fw_data_maxsz = IWL5000_FW_DATA_MAXSZ;
	sc->fwsz = IWL5000_FWSZ;
	sc->sched_txfact_addr = IWL5000_SCHED_TXFACT;
	sc->reset_noise_gain = IWL5000_PHY_CALIB_RESET_NOISE_GAIN;
	sc->noise_gain = IWL5000_PHY_CALIB_NOISE_GAIN;

	switch (sc->hw_type) {
	case IWL_HW_REV_TYPE_6005:
		sc->limits = &iwl6000_sensitivity_limits;
		if (pid != 0x0082 && pid != 0x0085) {
			sc->fwname = "iwl6000g2bfw";
			sc->sc_flags |= IWL_FLAG_ADV_BTCOEX;
		} else
			sc->fwname = "iwl6000g2afw";
		break;
	case IWL_HW_REV_TYPE_1000:

		sc->limits = &iwl1000_sensitivity_limits;
		if (pid == 0x08AE || pid == 0x08AF) {
		 sc->fwname = "iwl100fw";
		}
		break;
	case IWL_HW_REV_TYPE_6000:
		sc->limits = &iwl6000_sensitivity_limits;
		sc->fwname = "iwl6000fw";
		if (pid == 0x422c || pid == 0x4239) {
			sc->sc_flags |= IWL_FLAG_INTERNAL_PA;
			/* Override chains masks, ROM is known to be broken. */
			sc->txchainmask = IWL_ANT_BC;
			sc->rxchainmask = IWL_ANT_BC;
		}
		break;
	case IWL_HW_REV_TYPE_2000:
		sc->limits = &iwl1000_sensitivity_limits;
		sc->fwname = "iwl2000fw";
		break;
	case IWL_HW_REV_TYPE_2030:
		sc->limits = &iwl2030_sensitivity_limits;
		sc->fwname = "iwl2030fw";
		sc->sc_flags |= IWL_FLAG_ADV_BTCOEX;
		break;
	default:
		device_printf(sc->sc_dev, "adapter type %d not supported\n",
		    sc->hw_type);
		return ENOTSUP;
	}
	return 0;
}

/*
 * Attach the interface to 802.11 radiotap.
 */
static void
iwl_radiotap_attach(struct iwl_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;

	ieee80211_radiotap_attach(ic,
	    &sc->sc_txtap.wt_ihdr, sizeof(sc->sc_txtap),
		IWL_TX_RADIOTAP_PRESENT,
	    &sc->sc_rxtap.wr_ihdr, sizeof(sc->sc_rxtap),
		IWL_RX_RADIOTAP_PRESENT);
}

static void
iwl_sysctlattach(struct iwl_softc *sc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->sc_dev);
	struct sysctl_oid *tree = device_get_sysctl_tree(sc->sc_dev);

#ifdef IWL_DEBUG
	sc->sc_debug = 0;
	SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(tree), OID_AUTO,
	    "debug", CTLFLAG_RW, &sc->sc_debug, 0, "control debugging printfs");
#endif
}

/*
 * Handles ioctl change requests for power management state etc.
 */
static int iwl_iv_reset(struct ieee80211vap *vap, u_long cmd)
{
	int error = 0;
	switch (cmd) {
	case IEEE80211_IOC_POWERSAVE:{
		struct iwl_softc * sc = (struct iwl_softc *)
				(vap->iv_ic->ic_ifp->if_softc);
		IWL_LOCK(sc);

		/* XXX to discuss */
		int dtim = IWL_POWERSAVE_DTIM_VOIP_COMPATIBLE;

		int async = 0;

		/*
		 * At present the powersave level is set to 0 (receiver
		 * continuously on) if power save is disabled, and set to
		 * IWL_POWERSAVE_LVL_VOIP_COMPATIBLE otherwise.
		 * Later the power levels 0-5 could be exposed to the user.
		 */
		if (vap->iv_flags & IEEE80211_F_PMGTON) {
			/*
			 * In ieee80211_vap_setup() we have:
			 *  vap->iv_flags = ic->ic_flags;
			 * i.e. any vap's iv_flags initialize to ic->ic_flags.
			 * Setting the flags on ic will allow a vap to get
			 * the correct powersave level once it is created.
			 */
			vap->iv_ic->ic_flags |= IEEE80211_F_PMGTON;
			sc->desired_pwrsave_level =
					IWL_POWERSAVE_LVL_VOIP_COMPATIBLE;
		} else {
			vap->iv_ic->ic_flags &= ~IEEE80211_F_PMGTON;
			sc->desired_pwrsave_level = IWL_POWERSAVE_LVL_NONE;
		}
		error = iwl_set_pslevel(sc, dtim, sc->desired_pwrsave_level,
				async);
		IWL_UNLOCK(sc);
		}
		break;
	default:
		/*
		 * For unhandled ioctls, ENETRESET has to be passed back to
		 * net80211 layer, which will in turn call the function to
		 * reinitialize the entire device.
		 */
		error = ENETRESET;
		break;
	}
	return error;
}

static struct ieee80211vap *
iwl_vap_create(struct ieee80211com *ic,
    const char name[IFNAMSIZ], int unit, enum ieee80211_opmode opmode, int flags,
    const uint8_t bssid[IEEE80211_ADDR_LEN],
    const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct iwl_vap *ivp;
	struct ieee80211vap *vap;
	uint8_t mac1[IEEE80211_ADDR_LEN];
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	
	if (sc->hw_type == IWL_HW_REV_TYPE_1000 || sc->hw_type == IWL_HW_REV_TYPE_6000  ) {
	if (!TAILQ_EMPTY(&ic->ic_vaps))		/* only one at a time */
		return NULL;
	}
	
	IEEE80211_ADDR_COPY(mac1, mac);

	if(unit == 1) {
		if(sc->uc_pan_support != IWL_UC_PAN_PRESENT)
			return NULL;
		mac1[5] += 1;
		sc->ctx	= IWL_RXON_PAN_CTX;
	}

	ivp = (struct iwl_vap *) malloc(sizeof(struct iwl_vap),
	    M_80211_VAP, M_NOWAIT | M_ZERO);
	if (ivp == NULL)
		return NULL;
	vap = &ivp->iv_vap;

	ieee80211_vap_setup(ic, vap, name, unit, opmode, flags, bssid, mac1);

	if(unit == 1) {
		ivp->ctx = IWL_RXON_PAN_CTX;
		ivp->iv_newstate = vap->iv_newstate;
		vap->iv_newstate = iwl_newstate1;
		IEEE80211_ADDR_COPY(ivp->macaddr, mac1);
		memset(&sc->rx_on[IWL_RXON_PAN_CTX], 0, sizeof (struct iwl_rxon));
		memcpy(&sc->rx_on[IWL_RXON_PAN_CTX], &sc->rx_on[IWL_RXON_BSS_CTX], sc->rxonsz);
		IEEE80211_ADDR_COPY(sc->rx_on[IWL_RXON_PAN_CTX].myaddr, mac1);
		sc->rx_on[IWL_RXON_PAN_CTX].mode = IWL_MODE_2STA;
		sc->ivap[IWL_RXON_PAN_CTX] = vap;
	}
	else {
		ivp->ctx = IWL_RXON_BSS_CTX;
		IEEE80211_ADDR_COPY(ivp->macaddr, mac1);
		ivp->iv_newstate = vap->iv_newstate;
		vap->iv_newstate = iwl_newstate;
		sc->ivap[IWL_RXON_BSS_CTX] = vap;
	}

	vap->iv_bmissthreshold = 10;		/* override default */
	/* Override with driver methods. */

	/* handler for setting change (partial 're'set) requested via ioctl */
	vap->iv_reset = iwl_iv_reset;

	/* Complete setup. */
	ieee80211_vap_attach(vap, iwl_media_change, ieee80211_media_status);
	ic->ic_opmode = opmode;
	return vap;
}

static void
iwl_vap_delete(struct ieee80211vap *vap)
{
	struct iwl_vap *ivp = IWL_VAP(vap);
	struct iwl_softc *sc = vap->iv_ic->ic_ifp->if_softc;

	if(ivp->ctx == IWL_RXON_PAN_CTX)
		sc->ctx = 0;

	ieee80211_ratectl_deinit(vap);
	ieee80211_vap_detach(vap);
	free(ivp, M_80211_VAP);
}

int
iwl_detach(device_t dev)
{
	struct iwl_softc *sc = device_get_softc(dev);
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic;
	int qid;

	if (ifp != NULL) {
		ic = ifp->if_l2com;

		ieee80211_draintask(ic, &sc->sc_reinit_task);
		ieee80211_draintask(ic, &sc->sc_radioon_task);
		ieee80211_draintask(ic, &sc->sc_radiooff_task);

		iwl_stop(sc);
		callout_drain(&sc->watchdog_to);
		callout_drain(&sc->ct_kill_exit_to);
		callout_drain(&sc->calib_to);
		ieee80211_ifdetach(ic);
	}
	/* XXX: free thermal throttling resources */
	/* Free DMA resources. */
	iwl_free_rx_ring(sc, &sc->rxq);
	for (qid = 0; qid < sc->ntxqs; qid++)
		iwl_free_tx_ring(sc, &sc->txq[qid]);
	iwl_free_sched(sc);
	iwl_free_kw(sc);
	if (sc->ict != NULL)
		iwl_free_ict(sc);
	iwl_free_fwmem(sc);

	if (ifp != NULL)
		if_free(ifp);

	IWL_LOCK_DESTROY(sc);
	return 0;
}

int
iwl_shutdown(device_t dev)
{
	struct iwl_softc *sc = device_get_softc(dev);

	iwl_stop(sc);
	return 0;
}

int
iwl_suspend(device_t dev)
{
	struct iwl_softc *sc = device_get_softc(dev);
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	iwl_stop(sc);
	if (vap != NULL)
		ieee80211_stop(vap);
	return 0;
}

int
iwl_resume(device_t dev)
{
	struct iwl_softc *sc = device_get_softc(dev);
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	/* Clear device-specific "PCI retry timeout" register (41h). */
	pci_write_config(dev, 0x41, 0, 1);

	if (ifp->if_flags & IFF_UP) {
		iwl_init(sc);
		if (vap != NULL)
			ieee80211_init(vap);
		if (ifp->if_drv_flags & IFF_DRV_RUNNING)
			iwl_start(ifp);
	}
	return 0;
}

int
iwl_nic_lock(struct iwl_softc *sc)
{
	int ntries;

	/* Request exclusive access to NIC. */
	IWL_SETBITS(sc, IWL_GP_CNTRL, IWL_GP_CNTRL_MAC_ACCESS_REQ);

	/* Spin until we actually get the lock. */
	for (ntries = 0; ntries < 1000; ntries++) {
		if ((IWL_READ(sc, IWL_GP_CNTRL) &
		     (IWL_GP_CNTRL_MAC_ACCESS_ENA | IWL_GP_CNTRL_SLEEP)) ==
		    IWL_GP_CNTRL_MAC_ACCESS_ENA)
			return 0;
		DELAY(10);
	}
	return ETIMEDOUT;
}

void
iwl_nic_unlock(struct iwl_softc *sc)
{
	IWL_CLRBITS(sc, IWL_GP_CNTRL, IWL_GP_CNTRL_MAC_ACCESS_REQ);
}

uint32_t
iwl_prph_read(struct iwl_softc *sc, uint32_t addr)
{
	IWL_WRITE(sc, IWL_PRPH_RADDR, IWL_PRPH_DWORD | addr);
	IWL_BARRIER_READ_WRITE(sc);
	return IWL_READ(sc, IWL_PRPH_RDATA);
}

void
iwl_prph_write(struct iwl_softc *sc, uint32_t addr, uint32_t data)
{
	IWL_WRITE(sc, IWL_PRPH_WADDR, IWL_PRPH_DWORD | addr);
	IWL_BARRIER_WRITE(sc);
	IWL_WRITE(sc, IWL_PRPH_WDATA, data);
}

void
iwl_prph_setbits(struct iwl_softc *sc, uint32_t addr, uint32_t mask)
{
	iwl_prph_write(sc, addr, iwl_prph_read(sc, addr) | mask);
}

void
iwl_prph_clrbits(struct iwl_softc *sc, uint32_t addr, uint32_t mask)
{
	iwl_prph_write(sc, addr, iwl_prph_read(sc, addr) & ~mask);
}

static __inline void
iwl_prph_write_region_4(struct iwl_softc *sc, uint32_t addr,
    const uint32_t *data, int count)
{
	for (; count > 0; count--, data++, addr += 4)
		iwl_prph_write(sc, addr, *data);
}

static __inline uint32_t
iwl_mem_read(struct iwl_softc *sc, uint32_t addr)
{
	IWL_WRITE(sc, IWL_MEM_RADDR, addr);
	IWL_BARRIER_READ_WRITE(sc);
	return IWL_READ(sc, IWL_MEM_RDATA);
}

static __inline void
iwl_mem_write(struct iwl_softc *sc, uint32_t addr, uint32_t data)
{
	IWL_WRITE(sc, IWL_MEM_WADDR, addr);
	IWL_BARRIER_WRITE(sc);
	IWL_WRITE(sc, IWL_MEM_WDATA, data);
}

static __inline void
iwl_mem_write_2(struct iwl_softc *sc, uint32_t addr, uint16_t data)
{
	uint32_t tmp;

	tmp = iwl_mem_read(sc, addr & ~3);
	if (addr & 3)
		tmp = (tmp & 0x0000ffff) | data << 16;
	else
		tmp = (tmp & 0xffff0000) | data;
	iwl_mem_write(sc, addr & ~3, tmp);
}

static __inline void
iwl_mem_read_region_4(struct iwl_softc *sc, uint32_t addr, uint32_t *data,
    int count)
{
	for (; count > 0; count--, addr += 4)
		*data++ = iwl_mem_read(sc, addr);
}

static __inline void
iwl_mem_set_region_4(struct iwl_softc *sc, uint32_t addr, uint32_t val,
    int count)
{
	for (; count > 0; count--, addr += 4)
		iwl_mem_write(sc, addr, val);
}

static int
iwl_eeprom_lock(struct iwl_softc *sc)
{
	int i, ntries;

	for (i = 0; i < 100; i++) {
		/* Request exclusive access to EEPROM. */
		IWL_SETBITS(sc, IWL_HW_IF_CONFIG,
		    IWL_HW_IF_CONFIG_EEPROM_LOCKED);

		/* Spin until we actually get the lock. */
		for (ntries = 0; ntries < 100; ntries++) {
			if (IWL_READ(sc, IWL_HW_IF_CONFIG) &
			    IWL_HW_IF_CONFIG_EEPROM_LOCKED)
				return 0;
			DELAY(10);
		}
	}
	return ETIMEDOUT;
}

static __inline void
iwl_eeprom_unlock(struct iwl_softc *sc)
{
	IWL_CLRBITS(sc, IWL_HW_IF_CONFIG, IWL_HW_IF_CONFIG_EEPROM_LOCKED);
}

/*
 * Initialize access by host to One Time Programmable ROM.
 * NB: This kind of ROM can be found on 1000 or 6000 Series only.
 */
static int
iwl_init_otprom(struct iwl_softc *sc)
{
	uint16_t prev, base, next;
	int count, error;

	/* Wait for clock stabilization before accessing prph. */
	if ((error = iwl_clock_wait(sc)) != 0)
		return error;

	if ((error = iwl_nic_lock(sc)) != 0)
		return error;
	iwl_prph_setbits(sc, IWL_APMG_PS, IWL_APMG_PS_RESET_REQ);
	DELAY(5);
	iwl_prph_clrbits(sc, IWL_APMG_PS, IWL_APMG_PS_RESET_REQ);
	iwl_nic_unlock(sc);

	/* Set auto clock gate disable bit for HW with OTP shadow RAM. */
	if (sc->hw_type != IWL_HW_REV_TYPE_1000) {
		IWL_SETBITS(sc, IWL_DBG_LINK_PWR_MGMT,
		    IWL_RESET_LINK_PWR_MGMT_DIS);
	}
	IWL_CLRBITS(sc, IWL_EEPROM_GP, IWL_EEPROM_GP_IF_OWNER);
	/* Clear ECC status. */
	IWL_SETBITS(sc, IWL_OTP_GP,
	    IWL_OTP_GP_ECC_CORR_STTS | IWL_OTP_GP_ECC_UNCORR_STTS);

	/*
	 * Find the block before last block (contains the EEPROM image)
	 * for HW without OTP shadow RAM.
	 */
	if (sc->hw_type == IWL_HW_REV_TYPE_1000) {
		/* Switch to absolute addressing mode. */
		IWL_CLRBITS(sc, IWL_OTP_GP, IWL_OTP_GP_RELATIVE_ACCESS);
		base = prev = 0;
		for (count = 0; count < IWL1000_OTP_NBLOCKS; count++) {
			error = iwl_read_prom_data(sc, base, &next, 2);
			if (error != 0)
				return error;
			if (next == 0)	/* End of linked-list. */
				break;
			prev = base;
			base = le16toh(next);
		}
		if (count == 0 || count == IWL1000_OTP_NBLOCKS)
			return EIO;
		/* Skip "next" word. */
		sc->prom_base = prev + 1;
	}
	return 0;
}

static int
iwl_read_prom_data(struct iwl_softc *sc, uint32_t addr, void *data, int count)
{
	uint8_t *out = data;
	uint32_t val, tmp;
	int ntries;

	addr += sc->prom_base;
	for (; count > 0; count -= 2, addr++) {
		IWL_WRITE(sc, IWL_EEPROM, addr << 2);
		for (ntries = 0; ntries < 10; ntries++) {
			val = IWL_READ(sc, IWL_EEPROM);
			if (val & IWL_EEPROM_READ_VALID)
				break;
			DELAY(5);
		}
		if (ntries == 10) {
			device_printf(sc->sc_dev,
			    "timeout reading ROM at 0x%x\n", addr);
			return ETIMEDOUT;
		}
		if (sc->sc_flags & IWL_FLAG_HAS_OTPROM) {
			/* OTPROM, check for ECC errors. */
			tmp = IWL_READ(sc, IWL_OTP_GP);
			if (tmp & IWL_OTP_GP_ECC_UNCORR_STTS) {
				device_printf(sc->sc_dev,
				    "OTPROM ECC error at 0x%x\n", addr);
				return EIO;
			}
			if (tmp & IWL_OTP_GP_ECC_CORR_STTS) {
				/* Correctable ECC error, clear bit. */
				IWL_SETBITS(sc, IWL_OTP_GP,
				    IWL_OTP_GP_ECC_CORR_STTS);
			}
		}
		*out++ = val >> 16;
		if (count > 1)
			*out++ = val >> 24;
	}
	return 0;
}

void
iwl_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	if (error != 0)
		return;
	KASSERT(nsegs == 1, ("too many DMA segments, %d should be 1", nsegs));
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

int
iwl_dma_contig_alloc(struct iwl_softc *sc, struct iwl_dma_info *dma,
    void **kvap, bus_size_t size, bus_size_t alignment)
{
	int error;

	dma->tag = NULL;
	dma->size = size;

	error = bus_dma_tag_create(bus_get_dma_tag(sc->sc_dev), alignment,
	    0, BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, size,
	    1, size, BUS_DMA_NOWAIT, NULL, NULL, &dma->tag);
	if (error != 0)
		goto fail;

	error = bus_dmamem_alloc(dma->tag, (void **)&dma->vaddr,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &dma->map);
	if (error != 0)
		goto fail;

	error = bus_dmamap_load(dma->tag, dma->map, dma->vaddr, size,
	    iwl_dma_map_addr, &dma->paddr, BUS_DMA_NOWAIT);
	if (error != 0)
		goto fail;

	bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);

	if (kvap != NULL)
		*kvap = dma->vaddr;

	return 0;

fail:	iwl_dma_contig_free(dma);
	return error;
}

void
iwl_dma_contig_free(struct iwl_dma_info *dma)
{
	if (dma->map != NULL) {
		if (dma->vaddr != NULL) {
			bus_dmamap_sync(dma->tag, dma->map,
			    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(dma->tag, dma->map);
			bus_dmamem_free(dma->tag, &dma->vaddr, dma->map);
			dma->vaddr = NULL;
		}
		bus_dmamap_destroy(dma->tag, dma->map);
		dma->map = NULL;
	}
	if (dma->tag != NULL) {
		bus_dma_tag_destroy(dma->tag);
		dma->tag = NULL;
	}
}

static int
iwl_alloc_ict(struct iwl_softc *sc)
{
	/* ICT table must be aligned on a 4KB boundary. */
	return iwl_dma_contig_alloc(sc, &sc->ict_dma, (void **)&sc->ict,
	    IWL_ICT_SIZE, 4096);
}

static void
iwl_free_ict(struct iwl_softc *sc)
{
	iwl_dma_contig_free(&sc->ict_dma);
}

static int
iwl_alloc_fwmem(struct iwl_softc *sc)
{
	/* Must be aligned on a 16-byte boundary. */
	return iwl_dma_contig_alloc(sc, &sc->fw_dma, NULL, sc->fwsz, 16);
}

static void
iwl_free_fwmem(struct iwl_softc *sc)
{
	iwl_dma_contig_free(&sc->fw_dma);
}

static int
iwl_alloc_rx_ring(struct iwl_softc *sc, struct iwl_rx_ring *ring)
{
	bus_size_t size;
	int i, error;

	ring->cur = 0;

	/* Allocate RX descriptors (256-byte aligned). */
	size = IWL_RX_RING_COUNT * sizeof (uint32_t);
	error = iwl_dma_contig_alloc(sc, &ring->desc_dma, (void **)&ring->desc,
	    size, 256);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not allocate RX ring DMA memory, error %d\n",
		    __func__, error);
		goto fail;
	}

	/* Allocate RX status area (16-byte aligned). */
	error = iwl_dma_contig_alloc(sc, &ring->stat_dma, (void **)&ring->stat,
	    sizeof (struct iwl_rx_status), 16);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not allocate RX status DMA memory, error %d\n",
		    __func__, error);
		goto fail;
	}

	/* Create RX buffer DMA tag. */
	error = bus_dma_tag_create(bus_get_dma_tag(sc->sc_dev), 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    IWL_RBUF_SIZE, 1, IWL_RBUF_SIZE, BUS_DMA_NOWAIT, NULL, NULL,
	    &ring->data_dmat);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not create RX buf DMA tag, error %d\n",
		    __func__, error);
		goto fail;
	}

	/*
	 * Allocate and map RX buffers.
	 */
	for (i = 0; i < IWL_RX_RING_COUNT; i++) {
		struct iwl_rx_data *data = &ring->data[i];
		bus_addr_t paddr;

		error = bus_dmamap_create(ring->data_dmat, 0, &data->map);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not create RX buf DMA map, error %d\n",
			    __func__, error);
			goto fail;
		}

		data->m = m_getjcl(M_DONTWAIT, MT_DATA, M_PKTHDR,
		    IWL_RBUF_SIZE);
		if (data->m == NULL) {
			device_printf(sc->sc_dev,
			    "%s: could not allocate RX mbuf\n", __func__);
			error = ENOBUFS;
			goto fail;
		}

		error = bus_dmamap_load(ring->data_dmat, data->map,
		    mtod(data->m, void *), IWL_RBUF_SIZE, iwl_dma_map_addr,
		    &paddr, BUS_DMA_NOWAIT);
		if (error != 0 && error != EFBIG) {
			device_printf(sc->sc_dev,
			    "%s: can't not map mbuf, error %d\n", __func__,
			    error);
			goto fail;
		}

		/* Set physical address of RX buffer (256-byte aligned). */
		ring->desc[i] = htole32(paddr >> 8);
	}

	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

	return 0;

fail:	iwl_free_rx_ring(sc, ring);
	return error;
}

static void
iwl_reset_rx_ring(struct iwl_softc *sc, struct iwl_rx_ring *ring)
{
	int ntries;

	if (iwl_nic_lock(sc) == 0) {
		IWL_WRITE(sc, IWL_FH_RX_CONFIG, 0);
		for (ntries = 0; ntries < 1000; ntries++) {
			if (IWL_READ(sc, IWL_FH_RX_STATUS) &
			    IWL_FH_RX_STATUS_IDLE)
				break;
			DELAY(10);
		}
		iwl_nic_unlock(sc);
	}
	ring->cur = 0;
	sc->last_rx_valid = 0;
}

static void
iwl_free_rx_ring(struct iwl_softc *sc, struct iwl_rx_ring *ring)
{
	int i;

	iwl_dma_contig_free(&ring->desc_dma);
	iwl_dma_contig_free(&ring->stat_dma);

	for (i = 0; i < IWL_RX_RING_COUNT; i++) {
		struct iwl_rx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(ring->data_dmat, data->map);
			m_freem(data->m);
			data->m = NULL;
		}
		if (data->map != NULL)
			bus_dmamap_destroy(ring->data_dmat, data->map);
	}
	if (ring->data_dmat != NULL) {
		bus_dma_tag_destroy(ring->data_dmat);
		ring->data_dmat = NULL;
	}
}

static void
iwl5000_ict_reset(struct iwl_softc *sc)
{
	/* Disable interrupts. */
	IWL_WRITE(sc, IWL_INT_MASK, 0);

	/* Reset ICT table. */
	memset(sc->ict, 0, IWL_ICT_SIZE);
	sc->ict_cur = 0;

	/* Set physical address of ICT table (4KB aligned). */
	DPRINTF(sc, IWL_DEBUG_RESET, "%s: enabling ICT\n", __func__);
	IWL_WRITE(sc, IWL_DRAM_INT_TBL, IWL_DRAM_INT_TBL_ENABLE |
	    IWL_DRAM_INT_TBL_WRAP_CHECK | sc->ict_dma.paddr >> 12);

	/* Enable periodic RX interrupt. */
	sc->int_mask |= IWL_INT_RX_PERIODIC;
	/* Switch to ICT interrupt mode in driver. */
	sc->sc_flags |= IWL_FLAG_USE_ICT;

	/* Re-enable interrupts. */
	IWL_WRITE(sc, IWL_INT, 0xffffffff);
	IWL_WRITE(sc, IWL_INT_MASK, sc->int_mask);
}

static int
iwl_read_eeprom(struct iwl_softc *sc, uint8_t macaddr[IEEE80211_ADDR_LEN])
{
	struct iwl_ops *ops = &sc->ops;
	uint16_t val;
	int error;

	/* Check whether adapter has an EEPROM or an OTPROM. */
	if (sc->hw_type >= IWL_HW_REV_TYPE_1000 &&
	    (IWL_READ(sc, IWL_OTP_GP) & IWL_OTP_GP_DEV_SEL_OTP))
		sc->sc_flags |= IWL_FLAG_HAS_OTPROM;
	DPRINTF(sc, IWL_DEBUG_RESET, "%s found\n",
	    (sc->sc_flags & IWL_FLAG_HAS_OTPROM) ? "OTPROM" : "EEPROM");

	/* Adapter has to be powered on for EEPROM access to work. */
	if ((error = iwl_apm_init(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not power ON adapter, error %d\n", __func__,
		    error);
		return error;
	}

	if ((IWL_READ(sc, IWL_EEPROM_GP) & 0x7) == 0) {
		device_printf(sc->sc_dev, "%s: bad ROM signature\n", __func__);
		return EIO;
	}
	if ((error = iwl_eeprom_lock(sc)) != 0) {
		device_printf(sc->sc_dev, "%s: could not lock ROM, error %d\n",
		    __func__, error);
		return error;
	}
	if (sc->sc_flags & IWL_FLAG_HAS_OTPROM) {
		if ((error = iwl_init_otprom(sc)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not initialize OTPROM, error %d\n",
			    __func__, error);
			return error;
		}
	}

	iwl_read_prom_data(sc, IWL_EEPROM_SKU_CAP, &val, 2);
	DPRINTF(sc, IWL_DEBUG_RESET, "SKU capabilities=0x%04x\n", le16toh(val));
	/* Check if HT support is bonded out. */
	if (val & htole16(IWL_EEPROM_SKU_CAP_11N))
		sc->sc_flags |= IWL_FLAG_HAS_11N;

	iwl_read_prom_data(sc, IWL_EEPROM_RFCFG, &val, 2);
	sc->rfcfg = le16toh(val);
	DPRINTF(sc, IWL_DEBUG_RESET, "radio config=0x%04x\n", sc->rfcfg);
	/* Read Tx/Rx chains from ROM unless it's known to be broken. */
	if (sc->txchainmask == 0)
		sc->txchainmask = IWL_RFCFG_TXANTMSK(sc->rfcfg);
	if (sc->rxchainmask == 0)
		sc->rxchainmask = IWL_RFCFG_RXANTMSK(sc->rfcfg);

	/* Read MAC address. */
	iwl_read_prom_data(sc, IWL_EEPROM_MAC, macaddr, 6);

	/* Read adapter-specific information from EEPROM. */
	ops->read_eeprom(sc);

	iwl_apm_stop(sc);	/* Power OFF adapter. */

	iwl_eeprom_unlock(sc);
	return 0;
}

static void
iwl5000_read_eeprom(struct iwl_softc *sc)
{
	struct iwl5000_eeprom_calib_hdr hdr;
	int32_t volt;
	uint32_t base, addr;
	uint16_t val;
	int i;

	/* Read regulatory domain (4 ASCII characters). */
	iwl_read_prom_data(sc, IWL5000_EEPROM_REG, &val, 2);
	base = le16toh(val);
	iwl_read_prom_data(sc, base + IWL5000_EEPROM_DOMAIN,
	    sc->eeprom_domain, 4);

	/* Read the list of authorized channels (20MHz ones only). */
	for (i = 0; i < 7; i++) {
		if (sc->hw_type >= IWL_HW_REV_TYPE_6000)
			addr = base + iwl6000_regulatory_bands[i];
		else
			addr = base + iwl5000_regulatory_bands[i];
		iwl_read_eeprom_channels(sc, i, addr);
	}

	/* Read enhanced TX power information for 6000 Series. */
	if (sc->hw_type >= IWL_HW_REV_TYPE_6000)
		iwl_read_eeprom_enhinfo(sc);

	iwl_read_prom_data(sc, IWL5000_EEPROM_CAL, &val, 2);
	base = le16toh(val);
	iwl_read_prom_data(sc, base, &hdr, sizeof hdr);
	DPRINTF(sc, IWL_DEBUG_CALIBRATE,
	    "%s: calib version=%u pa type=%u voltage=%u\n", __func__,
	    hdr.version, hdr.pa_type, le16toh(hdr.volt));
	sc->calib_ver = hdr.version;

	if (sc->hw_type == IWL_HW_REV_TYPE_5150) {
		/* Compute temperature offset. */
		iwl_read_prom_data(sc, base + IWL5000_EEPROM_TEMP, &val, 2);
		sc->eeprom_temp = le16toh(val);
		iwl_read_prom_data(sc, base + IWL5000_EEPROM_VOLT, &val, 2);
		volt = le16toh(val);
		sc->temp_off = sc->eeprom_temp - (volt / -5);
		DPRINTF(sc, IWL_DEBUG_CALIBRATE, "temp=%d volt=%d offset=%dK\n",
		    sc->eeprom_temp, volt, sc->temp_off);
	} else {
		/* Read crystal calibration. */
		iwl_read_prom_data(sc, base + IWL5000_EEPROM_CRYSTAL,
		    &sc->eeprom_crystal, sizeof (uint32_t));
		DPRINTF(sc, IWL_DEBUG_CALIBRATE, "crystal calibration 0x%08x\n",
		    le32toh(sc->eeprom_crystal));
	}
}

/*
 * Translate EEPROM flags to net80211.
 */
static uint32_t
iwl_eeprom_channel_flags(struct iwl_eeprom_chan *channel)
{
	uint32_t nflags;

	nflags = 0;
	if ((channel->flags & IWL_EEPROM_CHAN_ACTIVE) == 0)
		nflags |= IEEE80211_CHAN_PASSIVE;
	if ((channel->flags & IWL_EEPROM_CHAN_IBSS) == 0)
		nflags |= IEEE80211_CHAN_NOADHOC;
	if (channel->flags & IWL_EEPROM_CHAN_RADAR) {
		nflags |= IEEE80211_CHAN_DFS;
		/* XXX apparently IBSS may still be marked */
		nflags |= IEEE80211_CHAN_NOADHOC;
	}

	return nflags;
}

static void
iwl_read_eeprom_band(struct iwl_softc *sc, int n)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct iwl_eeprom_chan *channels = sc->eeprom_channels[n];
	const struct iwl_chan_band *band = &iwl_bands[n];
	struct ieee80211_channel *c;
	uint8_t chan;
	int i, nflags;

	for (i = 0; i < band->nchan; i++) {
		if (!(channels[i].flags & IWL_EEPROM_CHAN_VALID)) {
			DPRINTF(sc, IWL_DEBUG_RESET,
			    "skip chan %d flags 0x%x maxpwr %d\n",
			    band->chan[i], channels[i].flags,
			    channels[i].maxpwr);
			continue;
		}
		chan = band->chan[i];
		nflags = iwl_eeprom_channel_flags(&channels[i]);

		c = &ic->ic_channels[ic->ic_nchans++];
		c->ic_ieee = chan;
		c->ic_maxregpower = channels[i].maxpwr;
		c->ic_maxpower = 2*c->ic_maxregpower;

		if (n == 0) {	/* 2GHz band */
			c->ic_freq = ieee80211_ieee2mhz(chan, IEEE80211_CHAN_G);
			/* G =>'s B is supported */
			c->ic_flags = IEEE80211_CHAN_B | nflags;
			c = &ic->ic_channels[ic->ic_nchans++];
			c[0] = c[-1];
			c->ic_flags = IEEE80211_CHAN_G | nflags;
		} else {	/* 5GHz band */
			c->ic_freq = ieee80211_ieee2mhz(chan, IEEE80211_CHAN_A);
			c->ic_flags = IEEE80211_CHAN_A | nflags;
		}

		/* Save maximum allowed TX power for this channel. */
		sc->maxpwr[chan] = channels[i].maxpwr;

		DPRINTF(sc, IWL_DEBUG_RESET,
		    "add chan %d flags 0x%x maxpwr %d\n", chan,
		    channels[i].flags, channels[i].maxpwr);

		if (sc->sc_flags & IWL_FLAG_HAS_11N) {
			/* add HT20, HT40 added separately */
			c = &ic->ic_channels[ic->ic_nchans++];
			c[0] = c[-1];
			c->ic_flags |= IEEE80211_CHAN_HT20;
		}
	}
}

static void
iwl_read_eeprom_ht40(struct iwl_softc *sc, int n)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct iwl_eeprom_chan *channels = sc->eeprom_channels[n];
	const struct iwl_chan_band *band = &iwl_bands[n];
	struct ieee80211_channel *c, *cent, *extc;
	uint8_t chan;
	int i, nflags;

	if (!(sc->sc_flags & IWL_FLAG_HAS_11N))
		return;

	for (i = 0; i < band->nchan; i++) {
		if (!(channels[i].flags & IWL_EEPROM_CHAN_VALID)) {
			DPRINTF(sc, IWL_DEBUG_RESET,
			    "skip chan %d flags 0x%x maxpwr %d\n",
			    band->chan[i], channels[i].flags,
			    channels[i].maxpwr);
			continue;
		}
		chan = band->chan[i];
		nflags = iwl_eeprom_channel_flags(&channels[i]);

		/*
		 * Each entry defines an HT40 channel pair; find the
		 * center channel, then the extension channel above.
		 */
		cent = ieee80211_find_channel_byieee(ic, chan,
		    (n == 5 ? IEEE80211_CHAN_G : IEEE80211_CHAN_A));
		if (cent == NULL) {	/* XXX shouldn't happen */
			device_printf(sc->sc_dev,
			    "%s: no entry for channel %d\n", __func__, chan);
			continue;
		}
		extc = ieee80211_find_channel(ic, cent->ic_freq+20,
		    (n == 5 ? IEEE80211_CHAN_G : IEEE80211_CHAN_A));
		if (extc == NULL) {
			DPRINTF(sc, IWL_DEBUG_RESET,
			    "%s: skip chan %d, extension channel not found\n",
			    __func__, chan);
			continue;
		}

		DPRINTF(sc, IWL_DEBUG_RESET,
		    "add ht40 chan %d flags 0x%x maxpwr %d\n",
		    chan, channels[i].flags, channels[i].maxpwr);

		c = &ic->ic_channels[ic->ic_nchans++];
		c[0] = cent[0];
		c->ic_extieee = extc->ic_ieee;
		c->ic_flags &= ~IEEE80211_CHAN_HT;
		c->ic_flags |= IEEE80211_CHAN_HT40U | nflags;
		c = &ic->ic_channels[ic->ic_nchans++];
		c[0] = extc[0];
		c->ic_extieee = cent->ic_ieee;
		c->ic_flags &= ~IEEE80211_CHAN_HT;
		c->ic_flags |= IEEE80211_CHAN_HT40D | nflags;
	}
}

static void
iwl_read_eeprom_channels(struct iwl_softc *sc, int n, uint32_t addr)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;

	iwl_read_prom_data(sc, addr, &sc->eeprom_channels[n],
	    iwl_bands[n].nchan * sizeof (struct iwl_eeprom_chan));

	if (n < 5)
		iwl_read_eeprom_band(sc, n);
	else
		iwl_read_eeprom_ht40(sc, n);
	ieee80211_sort_channels(ic->ic_channels, ic->ic_nchans);
}

struct iwl_eeprom_chan *
iwl_find_eeprom_channel(struct iwl_softc *sc, struct ieee80211_channel *c)
{
	int band, chan, i, j;

	if (IEEE80211_IS_CHAN_HT40(c)) {
		band = IEEE80211_IS_CHAN_5GHZ(c) ? 6 : 5;
		if (IEEE80211_IS_CHAN_HT40D(c))
			chan = c->ic_extieee;
		else
			chan = c->ic_ieee;
		for (i = 0; i < iwl_bands[band].nchan; i++) {
			if (iwl_bands[band].chan[i] == chan)
				return &sc->eeprom_channels[band][i];
		}
	} else {
		for (j = 0; j < 5; j++) {
			for (i = 0; i < iwl_bands[j].nchan; i++) {
				if (iwl_bands[j].chan[i] == c->ic_ieee)
					return &sc->eeprom_channels[j][i];
			}
		}
	}
	return NULL;
}

/*
 * Enforce flags read from EEPROM.
 */
static int
iwl_setregdomain(struct ieee80211com *ic, struct ieee80211_regdomain *rd,
    int nchan, struct ieee80211_channel chans[])
{
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	int i;

	for (i = 0; i < nchan; i++) {
		struct ieee80211_channel *c = &chans[i];
		struct iwl_eeprom_chan *channel;

		channel = iwl_find_eeprom_channel(sc, c);
		if (channel == NULL) {
			if_printf(ic->ic_ifp,
			    "%s: invalid channel %u freq %u/0x%x\n",
			    __func__, c->ic_ieee, c->ic_freq, c->ic_flags);
			return EINVAL;
		}
		c->ic_flags |= iwl_eeprom_channel_flags(channel);
	}

	return 0;
}

static void
iwl_read_eeprom_enhinfo(struct iwl_softc *sc)
{
	struct iwl_eeprom_enhinfo enhinfo[35];
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_channel *c;
	uint16_t val, base;
	int8_t maxpwr;
	uint8_t flags;
	int i, j;

	iwl_read_prom_data(sc, IWL5000_EEPROM_REG, &val, 2);
	base = le16toh(val);
	iwl_read_prom_data(sc, base + IWL6000_EEPROM_ENHINFO,
	    enhinfo, sizeof enhinfo);

	for (i = 0; i < nitems(enhinfo); i++) {
		flags = enhinfo[i].flags;
		if (!(flags & IWL_ENHINFO_VALID))
			continue;	/* Skip invalid entries. */

		maxpwr = 0;
		if (sc->txchainmask & IWL_ANT_A)
			maxpwr = MAX(maxpwr, enhinfo[i].chain[0]);
		if (sc->txchainmask & IWL_ANT_B)
			maxpwr = MAX(maxpwr, enhinfo[i].chain[1]);
		if (sc->txchainmask & IWL_ANT_C)
			maxpwr = MAX(maxpwr, enhinfo[i].chain[2]);
		if (sc->ntxchains == 2)
			maxpwr = MAX(maxpwr, enhinfo[i].mimo2);
		else if (sc->ntxchains == 3)
			maxpwr = MAX(maxpwr, enhinfo[i].mimo3);

		for (j = 0; j < ic->ic_nchans; j++) {
			c = &ic->ic_channels[j];
			if ((flags & IWL_ENHINFO_5GHZ)) {
				if (!IEEE80211_IS_CHAN_A(c))
					continue;
			} else if ((flags & IWL_ENHINFO_OFDM)) {
				if (!IEEE80211_IS_CHAN_G(c))
					continue;
			} else if (!IEEE80211_IS_CHAN_B(c))
				continue;
			if ((flags & IWL_ENHINFO_HT40)) {
				if (!IEEE80211_IS_CHAN_HT40(c))
					continue;
			} else {
				if (IEEE80211_IS_CHAN_HT40(c))
					continue;
			}
			if (enhinfo[i].chan != 0 &&
			    enhinfo[i].chan != c->ic_ieee)
				continue;

			DPRINTF(sc, IWL_DEBUG_RESET,
			    "channel %d(%x), maxpwr %d\n", c->ic_ieee,
			    c->ic_flags, maxpwr / 2);
			c->ic_maxregpower = maxpwr / 2;
			c->ic_maxpower = maxpwr;
		}
	}
}

static struct ieee80211_node *
iwl_node_alloc(struct ieee80211vap *vap, const uint8_t mac[IEEE80211_ADDR_LEN])
{
	return malloc(sizeof (struct iwl_node), M_80211_NODE,M_NOWAIT | M_ZERO);
}

int
rate2plcp(int rate)
{
	switch (rate & 0xff) {
	case 12:	return 0xd;
	case 18:	return 0xf;
	case 24:	return 0x5;
	case 36:	return 0x7;
	case 48:	return 0x9;
	case 72:	return 0xb;
	case 96:	return 0x1;
	case 108:	return 0x3;
	case 2:		return 10;
	case 4:		return 20;
	case 11:	return 55;
	case 22:	return 110;
	}
	return 0;
}

static void
iwl_newassoc(struct ieee80211_node *ni, int isnew)
{
#define	RV(v)	((v) & IEEE80211_RATE_VAL)
	struct ieee80211com *ic = ni->ni_ic;
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	struct iwl_node *wn = (void *)ni;
	uint8_t txant1, txant2;
	int i, plcp, rate, ridx;

	/* Use the first valid TX antenna. */
	txant1 = IWL_LSB(sc->txchainmask);
	txant2 = IWL_LSB(sc->txchainmask & ~txant1);

	if (IEEE80211_IS_CHAN_HT(ni->ni_chan)) {
		ridx = ni->ni_rates.rs_nrates - 1;
		for (i = ni->ni_htrates.rs_nrates - 1; i >= 0; i--) {
			plcp = RV(ni->ni_htrates.rs_rates[i]) | IWL_RFLAG_MCS;
			if (IEEE80211_IS_CHAN_HT40(ni->ni_chan)) {
				plcp |= IWL_RFLAG_HT40;
				if (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI40)
					plcp |= IWL_RFLAG_SGI;
			} else if (ni->ni_htcap & IEEE80211_HTCAP_SHORTGI20)
				plcp |= IWL_RFLAG_SGI;
			if (i > 7)
				plcp |= IWL_RFLAG_ANT(txant1 | txant2);
			else
				plcp |= IWL_RFLAG_ANT(txant1);
			if (ridx >= 0) {
				rate = RV(ni->ni_rates.rs_rates[ridx]);
				wn->ridx[rate] = plcp;
			}
			wn->ridx[IEEE80211_RATE_MCS | i] = plcp;
			ridx--;
		}
	} else {
		static const uint32_t iwl_rates_plcp[] = {
			/* PLCP/hw value for CCK */
			10, 20, 55, 110,
			/* PLCP/hw value for OFDM */
			13, 15, 5, 7, 9, 11, 1, 3,
			/* PLCP/hw value for HT 60 MBps -- remove? */
			3
		};

		for (i = 0; i < ni->ni_rates.rs_nrates; i++) {
			rate = RV(ni->ni_rates.rs_rates[i]);
			plcp = iwl_rates_plcp[i];
			ridx = ic->ic_rt->rateCodeToIndex[rate];
			if (ridx < IWL_RIDX_OFDM6 &&
			    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
				plcp |= IWL_RFLAG_CCK;
			plcp |= IWL_RFLAG_ANT(txant1);
			wn->ridx[rate] = htole32(plcp);
		}
	}
#undef	RV
}

static int
iwl_media_change(struct ifnet *ifp)
{
	int error;

	error = ieee80211_media_change(ifp);
	/* NB: only the fixed rate can change and that doesn't need a reset */
	return (error == ENETRESET ? 0 : error);
}

static int
iwl_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct iwl_vap *ivp = IWL_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	int error = 0;

	DPRINTF(sc, IWL_DEBUG_STATE, "%s: %s -> %s\n", __func__,
	    ieee80211_state_name[vap->iv_state], ieee80211_state_name[nstate]);

	IEEE80211_UNLOCK(ic);
	IWL_LOCK(sc);
	callout_stop(&sc->calib_to);

	sc->rxon = &sc->rx_on[IWL_RXON_BSS_CTX];

	switch (nstate) {
	case IEEE80211_S_ASSOC:
		if (vap->iv_state != IEEE80211_S_RUN)
			break;
		/* FALLTHROUGH */
	case IEEE80211_S_AUTH:
		if (vap->iv_state == IEEE80211_S_AUTH)
			break;

		/*
		 * !AUTH -> AUTH transition requires state reset to handle
		 * reassociations correctly.
		 */
		sc->rxon->associd = 0;
		sc->rxon->filter &= ~htole32(IWL_FILTER_BSS);
		sc->calib.state = IWL_CALIB_STATE_INIT;

		if ((error = iwl_auth(sc, vap)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not move to auth state\n", __func__);
		}
		break;

	case IEEE80211_S_RUN:
		/*
		 * RUN -> RUN transition; Just restart the timers.
		 */
		if (vap->iv_state == IEEE80211_S_RUN) {
			sc->calib_cnt = 0;
			break;
		}

		/*
		 * !RUN -> RUN requires setting the association id
		 * which is done with a firmware cmd.  We also defer
		 * starting the timers until that work is done.
		 */
		if ((error = iwl_run(sc, vap)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not move to run state\n", __func__);
		}
		break;

	case IEEE80211_S_INIT:
		sc->calib.state = IWL_CALIB_STATE_INIT;
		break;

	default:
		break;
	}
	IWL_UNLOCK(sc);
	IEEE80211_LOCK(ic);
	if (error != 0)
		return error;
	return ivp->iv_newstate(vap, nstate, arg);
}

static int
iwl_newstate1(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct iwl_vap *ivp = IWL_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct iwl_softc *sc = ic->ic_ifp->if_softc;

	int error = 0;

	DPRINTF(sc, IWL_DEBUG_STATE, "%s: %s -> %s\n", __func__,
	    ieee80211_state_name[vap->iv_state], ieee80211_state_name[nstate]);

	IEEE80211_UNLOCK(ic);
	IWL_LOCK(sc);
	callout_stop(&sc->calib_to);

	sc->rxon = &sc->rx_on[IWL_RXON_PAN_CTX];

	switch (nstate) {
	case IEEE80211_S_ASSOC:
		if (vap->iv_state != IEEE80211_S_RUN)
			break;
		/* FALLTHROUGH */
	case IEEE80211_S_AUTH:
		if (vap->iv_state == IEEE80211_S_AUTH)
			break;

		/*
		 * !AUTH -> AUTH transition requires state reset to handle
		 * reassociations correctly.
		 */
		sc->rxon->associd = 0;
		sc->rxon->filter &= ~htole32(IWL_FILTER_BSS);
		sc->calib.state = IWL_CALIB_STATE_INIT;

		if ((error = iwl_auth1(sc, vap)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not move to auth state\n", __func__);
		}
		break;

	case IEEE80211_S_SCAN:


		if ((error = iwl_set_timing1(sc)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: iwl_set_timing1 error %d\n", __func__, error);
			return error;
		}

		break;

	case IEEE80211_S_RUN:

		/*
		 * RUN -> RUN transition; Just restart the timers.
		 */
		if (vap->iv_state == IEEE80211_S_RUN) {
			sc->calib_cnt = 0;
			break;
		}

		/*
		 * !RUN -> RUN requires setting the association id
		 * which is done with a firmware cmd.  We also defer
		 * starting the timers until that work is done.
		 */
		if ((error = iwl_run1(sc, vap)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not move to run state\n", __func__);
		}
		break;

	case IEEE80211_S_INIT:
		sc->calib.state = IWL_CALIB_STATE_INIT;
		break;

	default:
		break;
	}
	IWL_UNLOCK(sc);
	IEEE80211_LOCK(ic);
	if (error != 0)
		return error;
	return ivp->iv_newstate(vap, nstate, arg);
}

void
iwl_calib_timeout(void *arg)
{
	struct iwl_softc *sc = arg;

	IWL_LOCK_ASSERT(sc);

	/* Force automatic TX power calibration every 60 secs. */
	if (++sc->calib_cnt >= 120) {
		uint32_t flags = 0;

		DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s\n",
		    "sending request for statistics");
		(void)iwl_cmd(sc, IWL_CMD_GET_STATISTICS, &flags,
		    sizeof flags, 1);
		sc->calib_cnt = 0;
	}
	callout_reset(&sc->calib_to, msecs_to_ticks(500), iwl_calib_timeout,
	    sc);
}

/*
 * Process an RX_PHY firmware notification.  This is usually immediately
 * followed by an MPDU_RX_DONE notification.
 */
static void
iwl_rx_phy(struct iwl_softc *sc, struct iwl_rx_desc *desc,
    struct iwl_rx_data *data)
{
	struct iwl_rx_stat *stat = (struct iwl_rx_stat *)(desc + 1);

	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s: received PHY stats\n", __func__);
	bus_dmamap_sync(sc->rxq.data_dmat, data->map, BUS_DMASYNC_POSTREAD);

	/* Save RX statistics, they will be used on MPDU_RX_DONE. */
	memcpy(&sc->last_rx_stat, stat, sizeof (*stat));
	sc->last_rx_valid = 1;
}

/*
 * Process an RX_DONE (4965AGN only) or MPDU_RX_DONE firmware notification.
 * Each MPDU_RX_DONE notification must be preceded by an RX_PHY one.
 */
static void
iwl_rx_done(struct iwl_softc *sc, struct iwl_rx_desc *desc,
    struct iwl_rx_data *data)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct iwl_rx_ring *ring = &sc->rxq;
	struct ieee80211_frame *wh;
	uint8_t type;
	struct ieee80211_node *ni;
	struct mbuf *m, *m1;
	struct iwl_rx_stat *stat;
	caddr_t head;
	bus_addr_t paddr;
	uint32_t flags;
	int error, len, rssi, nf;

	if (desc->type == IWL_MPDU_RX_DONE) {
		/* Check for prior RX_PHY notification. */
		if (!sc->last_rx_valid) {
			DPRINTF(sc, IWL_DEBUG_ANY,
			    "%s: missing RX_PHY\n", __func__);
			return;
		}
		stat = &sc->last_rx_stat;
	} else
		stat = (struct iwl_rx_stat *)(desc + 1);

	bus_dmamap_sync(ring->data_dmat, data->map, BUS_DMASYNC_POSTREAD);

	if (stat->cfg_phy_len > IWL_STAT_MAXLEN) {
		device_printf(sc->sc_dev,
		    "%s: invalid RX statistic header, len %d\n", __func__,
		    stat->cfg_phy_len);
		return;
	}
	if (desc->type == IWL_MPDU_RX_DONE) {
		struct iwl_rx_mpdu *mpdu = (struct iwl_rx_mpdu *)(desc + 1);
		head = (caddr_t)(mpdu + 1);
		len = le16toh(mpdu->len);
	} else {
		head = (caddr_t)(stat + 1) + stat->cfg_phy_len;
		len = le16toh(stat->len);
	}

	flags = le32toh(*(uint32_t *)(head + len));

	/* Discard frames with a bad FCS early. */
	if ((flags & IWL_RX_NOERROR) != IWL_RX_NOERROR) {
		DPRINTF(sc, IWL_DEBUG_RECV, "%s: RX flags error %x\n",
		    __func__, flags);
		ifp->if_ierrors++;
		return;
	}
	/* Discard frames that are too short. */
	if (len < sizeof (*wh)) {
		DPRINTF(sc, IWL_DEBUG_RECV, "%s: frame too short: %d\n",
		    __func__, len);
		ifp->if_ierrors++;
		return;
	}

	m1 = m_getjcl(M_DONTWAIT, MT_DATA, M_PKTHDR, IWL_RBUF_SIZE);
	if (m1 == NULL) {
		DPRINTF(sc, IWL_DEBUG_ANY, "%s: no mbuf to restock ring\n",
		    __func__);
		ifp->if_ierrors++;
		return;
	}
	bus_dmamap_unload(ring->data_dmat, data->map);

	error = bus_dmamap_load(ring->data_dmat, data->map, mtod(m1, void *),
	    IWL_RBUF_SIZE, iwl_dma_map_addr, &paddr, BUS_DMA_NOWAIT);
	if (error != 0 && error != EFBIG) {
		device_printf(sc->sc_dev,
		    "%s: bus_dmamap_load failed, error %d\n", __func__, error);
		m_freem(m1);

		/* Try to reload the old mbuf. */
		error = bus_dmamap_load(ring->data_dmat, data->map,
		    mtod(data->m, void *), IWL_RBUF_SIZE, iwl_dma_map_addr,
		    &paddr, BUS_DMA_NOWAIT);
		if (error != 0 && error != EFBIG) {
			panic("%s: could not load old RX mbuf", __func__);
		}
		/* Physical address may have changed. */
		ring->desc[ring->cur] = htole32(paddr >> 8);
		bus_dmamap_sync(ring->data_dmat, ring->desc_dma.map,
		    BUS_DMASYNC_PREWRITE);
		ifp->if_ierrors++;
		return;
	}

	m = data->m;
	data->m = m1;
	/* Update RX descriptor. */
	ring->desc[ring->cur] = htole32(paddr >> 8);
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

	/* Finalize mbuf. */
	m->m_pkthdr.rcvif = ifp;
	m->m_data = head;
	m->m_pkthdr.len = m->m_len = len;

	/* Grab a reference to the source node. */
	wh = mtod(m, struct ieee80211_frame *);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)wh);
	nf = (ni != NULL && ni->ni_vap->iv_state == IEEE80211_S_RUN &&
	    (ic->ic_flags & IEEE80211_F_SCAN) == 0) ? sc->noise : -95;

	rssi = ops->get_rssi(sc, stat);

	if (ieee80211_radiotap_active(ic)) {
		struct iwl_rx_radiotap_header *tap = &sc->sc_rxtap;

		tap->wr_flags = 0;
		if (stat->flags & htole16(IWL_STAT_FLAG_SHPREAMBLE))
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
		tap->wr_dbm_antsignal = (int8_t)rssi;
		tap->wr_dbm_antnoise = (int8_t)nf;
		tap->wr_tsft = stat->tstamp;
		switch (stat->rate) {
		/* CCK rates. */
		case  10: tap->wr_rate =   2; break;
		case  20: tap->wr_rate =   4; break;
		case  55: tap->wr_rate =  11; break;
		case 110: tap->wr_rate =  22; break;
		/* OFDM rates. */
		case 0xd: tap->wr_rate =  12; break;
		case 0xf: tap->wr_rate =  18; break;
		case 0x5: tap->wr_rate =  24; break;
		case 0x7: tap->wr_rate =  36; break;
		case 0x9: tap->wr_rate =  48; break;
		case 0xb: tap->wr_rate =  72; break;
		case 0x1: tap->wr_rate =  96; break;
		case 0x3: tap->wr_rate = 108; break;
		/* Unknown rate: should not happen. */
		default:  tap->wr_rate =   0;
		}
	}

	IWL_UNLOCK(sc);

	/* Send the frame to the 802.11 layer. */
	if (ni != NULL) {
		if (ni->ni_flags & IEEE80211_NODE_HT)
			m->m_flags |= M_AMPDU;
		(void)ieee80211_input(ni, m, rssi - nf, nf);
		/* Node is no longer needed. */
		ieee80211_free_node(ni);
	} else
		(void)ieee80211_input_all(ic, m, rssi - nf, nf);

	IWL_LOCK(sc);
}

/* Process an incoming Compressed BlockAck. */
static void
iwl_rx_compressed_ba(struct iwl_softc *sc, struct iwl_rx_desc *desc,
    struct iwl_rx_data *data)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct iwl_node *wn;
	struct ieee80211_node *ni;
	struct iwl_compressed_ba *ba = (struct iwl_compressed_ba *)(desc + 1);
	struct iwl_tx_ring *txq;
	struct ieee80211_tx_ampdu *tap;
	uint64_t bitmap;
	uint8_t tid;
	int ack = 0;
	int successes = 0;
	int i, shift;

	bus_dmamap_sync(sc->rxq.data_dmat, data->map, BUS_DMASYNC_POSTREAD);

	txq = &sc->txq[le16toh(ba->qid)];
	tap = sc->qid2tap[le16toh(ba->qid)];
	tid = tap->txa_tid;
	ni = tap->txa_ni;
	wn = (void *)ni;

	if (wn->agg[tid].bitmap == 0)
		return;

	shift = wn->agg[tid].startidx - ((le16toh(ba->seq) >> 4) & 0xff);
	if (shift < 0)
		shift += 0x100;

	if (wn->agg[tid].nframes > (64 - shift))
		return;

	bitmap = (le64toh(ba->bitmap) >> shift) & wn->agg[tid].bitmap;

	for (i = 0; bitmap; i++) {
		/* new line */
		ack = bitmap & 1;
		successes += ack;

		if ((bitmap & 1) == 0) {
			ifp->if_oerrors++;
			/* XXX: call rate scaling */
		} else {
			ifp->if_opackets++;
			/* XXX: call rate scaling */
		}
		bitmap >>= 1;
	}
}

/*
 * Process a CALIBRATION_RESULT notification sent by the initialization
 * firmware on response to a CMD_CALIB_CONFIG command (5000 only).
 */
static void
iwl5000_rx_calib_results(struct iwl_softc *sc, struct iwl_rx_desc *desc,
    struct iwl_rx_data *data)
{
	struct iwl_phy_calib *calib = (struct iwl_phy_calib *)(desc + 1);
	int len, idx = -1;

	/* Runtime firmware should not send such a notification. */
	if (sc->sc_flags & IWL_FLAG_CALIB_DONE)
		return;

	len = (le32toh(desc->len) & 0x3fff) - 4;
	bus_dmamap_sync(sc->rxq.data_dmat, data->map, BUS_DMASYNC_POSTREAD);

	switch (calib->code) {
	case IWL5000_PHY_CALIB_DC:
		if ((sc->sc_flags & IWL_FLAG_INTERNAL_PA) == 0 &&
		    (sc->hw_type == IWL_HW_REV_TYPE_5150 ||
		     sc->hw_type >= IWL_HW_REV_TYPE_6000))
			idx = 0;
		break;
	case IWL5000_PHY_CALIB_LO:
		idx = 1;
		break;
	case IWL5000_PHY_CALIB_TX_IQ:
		idx = 2;
		break;
	case IWL5000_PHY_CALIB_TX_IQ_PERIODIC:
		if (sc->hw_type < IWL_HW_REV_TYPE_6000 &&
		    sc->hw_type != IWL_HW_REV_TYPE_5150)
			idx = 3;
		break;
	case IWL5000_PHY_CALIB_BASE_BAND:
		idx = 4;
		break;
	}
	if (idx == -1)	/* Ignore other results. */
		return;

	/* Save calibration result. */
	if (sc->calibcmd[idx].buf != NULL)
		free(sc->calibcmd[idx].buf, M_DEVBUF);
	sc->calibcmd[idx].buf = malloc(len, M_DEVBUF, M_NOWAIT);
	if (sc->calibcmd[idx].buf == NULL) {
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "not enough memory for calibration result %d\n",
		    calib->code);
		return;
	}
	DPRINTF(sc, IWL_DEBUG_CALIBRATE,
	    "saving calibration result code=%d len=%d\n", calib->code, len);
	sc->calibcmd[idx].len = len;
	memcpy(sc->calibcmd[idx].buf, calib, len);
}

/*
 * Process an RX_STATISTICS or BEACON_STATISTICS firmware notification.
 * The latter is sent by the firmware after each received beacon.
 */
static void
iwl_rx_statistics(struct iwl_softc *sc, struct iwl_rx_desc *desc,
    struct iwl_rx_data *data)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct iwl_calib_state *calib = &sc->calib;
	struct iwl_stats *stats = (struct iwl_stats *)(desc + 1);
	struct ieee80211_node *ni = vap->iv_bss;
	struct ieee80211_nodestats ns = ni->ni_stats;
	struct bintime bt;

	struct ieee80211vap *vap1;

	/* Ignore statistics received during a scan. */
	if (vap->iv_state != IEEE80211_S_RUN ||
	    (ic->ic_flags & IEEE80211_F_SCAN))
		return;

	if(sc->ctx == IWL_RXON_PAN_CTX) {
		vap1 = sc->ivap[IWL_RXON_PAN_CTX];
		/* Ignore statistics received during a scan. */
		if (vap1->iv_state != IEEE80211_S_RUN ||
		    (ic->ic_flags & IEEE80211_F_SCAN))
			return;
	}

	getbinuptime(&bt);
	sc->sc_led.led_cur_time = bt.sec;
	int time_diff = (sc->sc_led.led_cur_time - sc->sc_led.led_last_time);

	if(time_diff >= 4) {
		if(vap->iv_state == IEEE80211_S_RUN) {
			sc->sc_led.led_cur_bt = (ns.ns_rx_bytes + ns.ns_tx_bytes);
			if(sc->sc_led.led_cur_bt < sc->sc_led.led_last_bt) {
				sc->sc_led.led_cur_bt = 0;
				sc->sc_led.led_last_bt = 0;
			}
			sc->sc_led.led_bt_diff = (sc->sc_led.led_cur_bt - sc->sc_led.led_last_bt);
			sc->sc_led.led_last_bt = sc->sc_led.led_cur_bt;
			sc->sc_led.led_cur_tpt = (sc->sc_led.led_bt_diff / time_diff);
			sc->sc_led.led_last_time = sc->sc_led.led_cur_time;

			if(sc->sc_led.led_cur_tpt > 0) {
				sc->sc_led.led_cur_mode = IWL_LED_INT_BLINK;
				iwl_led_pattern(sc);
			} else {
				sc->sc_led.led_cur_mode = IWL_LED_STATIC_ON;
				iwl_set_led(sc, IWL_LED_LINK, 0, 1, IWL_LED_STATIC_ON);
			}

			sc->sc_led.led_last_tpt = sc->sc_led.led_cur_tpt;
		}
	}

	bus_dmamap_sync(sc->rxq.data_dmat, data->map, BUS_DMASYNC_POSTREAD);

	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s: received statistics, cmd %d\n",
	    __func__, desc->type);
	sc->calib_cnt = 0;	/* Reset TX power calibration timeout. */

	/* Test if temperature has changed. */
	if (stats->general.temp != sc->rawtemp) {
		/* Convert "raw" temperature to degC. */
		sc->rawtemp = stats->general.temp;
		ops->get_temperature(sc);
		
	
	}

	if (desc->type != IWL_BEACON_STATISTICS)
		return;	/* Reply to a statistics request. */

	sc->noise = iwl_get_noise(&stats->rx.general);
	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s: noise %d\n", __func__, sc->noise);

	/* Test that RSSI and noise are present in stats report. */
	if (le32toh(stats->rx.general.flags) != 1) {
		DPRINTF(sc, IWL_DEBUG_ANY, "%s\n",
		    "received statistics without RSSI");
		return;
	}

	if (calib->state == IWL_CALIB_STATE_ASSOC)
		iwl_collect_noise(sc, &stats->rx.general);
	else if (calib->state == IWL_CALIB_STATE_RUN)
		iwl_tune_sensitivity(sc, &stats->rx);
}

/*
 * Process a TX_DONE firmware notification.  Unfortunately, the 4965AGN
 * and 5000 adapters have different incompatible TX status formats.
 */

static void
iwl5000_tx_done(struct iwl_softc *sc, struct iwl_rx_desc *desc,
    struct iwl_rx_data *data)
{
	struct iwl5000_tx_stat *stat = (struct iwl5000_tx_stat *)(desc + 1);
	struct iwl_tx_ring *ring;
	int qid;

	qid = desc->qid & 0xf;
	ring = &sc->txq[qid];
        
	DPRINTF(sc, IWL_DEBUG_XMIT, "%s: "
	    "qid %d idx %d retries %d nkill %d rate %x duration %d status %x\n",
	    __func__, desc->qid, desc->idx, stat->ackfailcnt,
	    stat->btkillcnt, stat->rate, le16toh(stat->duration),
	    le32toh(stat->status));

#ifdef notyet
	/* Reset TX scheduler slot. */
	iwl5000_reset_sched(sc, desc->qid & 0xf, desc->idx);
#endif

	bus_dmamap_sync(ring->data_dmat, data->map, BUS_DMASYNC_POSTREAD);
	if (qid >= sc->firstaggqueue) {
		iwl_ampdu_tx_done(sc, qid, desc->idx, stat->nframes,
		    &stat->status);
	} else {
		iwl_tx_done(sc, desc, stat->ackfailcnt,
		    le16toh(stat->status) & 0xff);
	}
}

/*
 * Adapter-independent backend for TX_DONE firmware notifications.
 */
static void
iwl_tx_done(struct iwl_softc *sc, struct iwl_rx_desc *desc,
  	    int ackfailcnt, uint8_t status)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct iwl_tx_ring *ring = &sc->txq[desc->qid & 0xf];
	struct iwl_tx_data *data = &ring->data[desc->idx];

	struct iwl_tx_cmd *cmd;
	struct iwl_cmd_data *tx;

	struct mbuf *m;
	struct ieee80211_node *ni;
	struct ieee80211vap *vap;
	struct ieee80211_frame *wh;

	KASSERT(data->ni != NULL, ("no node"));

	/* Unmap and free mbuf. */
	uint8_t ridx;
	bus_dmamap_sync(ring->data_dmat, data->map, BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(ring->data_dmat, data->map);
	m = data->m, data->m = NULL;
	ni = data->ni, data->ni = NULL;
	uint8_t type;
	vap = ni->ni_vap;

	cmd = &ring->cmd[desc->idx];
	tx = (struct iwl_cmd_data *)cmd->data;
	wh = (struct ieee80211_frame *)(tx + 1);

	struct ieee80211com *ic = ni->ni_ic;

	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;

	if (m->m_flags & M_TXCB) {
		/*
		 * Channels marked for "radar" require traffic to be received
		 * to unlock before we can transmit.  Until traffic is seen
		 * any attempt to transmit is returned immediately with status
		 * set to IWL_TX_FAIL_TX_LOCKED.  Unfortunately this can easily
		 * happen on first authenticate after scanning.  To workaround
		 * this we ignore a failure of this sort in AUTH state so the
		 * 802.11 layer will fall back to using a timeout to wait for
		 * the AUTH reply.  This allows the firmware time to see
		 * traffic so a subsequent retry of AUTH succeeds.  It's
		 * unclear why the firmware does not maintain state for
		 * channels recently visited as this would allow immediate
		 * use of the channel after a scan (where we see traffic).
		 */
		if (status == IWL_TX_FAIL_TX_LOCKED &&
		    ni->ni_vap->iv_state == IEEE80211_S_AUTH)
			ieee80211_process_callback(ni, m, 0);
		else
			ieee80211_process_callback(ni, m,
			    (status & IWL_TX_FAIL) != 0);
	}

	ridx = ic->ic_rt->rateCodeToIndex[ni->ni_txrate];

	/*
	 * Update rate control statistics for the node.
	 */
	if (status & IWL_TX_FAIL) {
		ifp->if_oerrors++;
		/* XXX: call rate scaling */
	} else {
		ifp->if_opackets++;
		/* XXX: call rate scaling */
	}

	m_freem(m);
	ieee80211_free_node(ni);

	sc->sc_tx_timer = 0;
	if (--ring->queued < IWL_TX_RING_LOMARK) {
		sc->qfullmsk &= ~(1 << ring->qid);
		if (sc->qfullmsk == 0 &&
		    (ifp->if_drv_flags & IFF_DRV_OACTIVE)) {
			ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
			iwl_start_locked(ifp);
		}
	}
}

/*
 * Process a "command done" firmware notification.  This is where we wakeup
 * processes waiting for a synchronous command completion.
 */
static void
iwl_cmd_done(struct iwl_softc *sc, struct iwl_rx_desc *desc)
{
	struct iwl_tx_ring *ring;
	struct iwl_tx_data *data;
	int cmd_queue_num;

	if(sc->uc_pan_support == IWL_UC_PAN_PRESENT)
		cmd_queue_num = IWL_PAN_CMD_QUEUE;
	else
		cmd_queue_num = IWL_CMD_QUEUE_NUM;

	ring = &sc->txq[cmd_queue_num];

	if ((desc->qid & 0xf) != cmd_queue_num)
		return;	/* Not a command ack. */

	data = &ring->data[desc->idx];

	/* If the command was mapped in an mbuf, free it. */
	if (data->m != NULL) {
		bus_dmamap_sync(ring->data_dmat, data->map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(ring->data_dmat, data->map);
		m_freem(data->m);
		data->m = NULL;
	}
	wakeup(&ring->desc[desc->idx]);
}

static void
iwl_ampdu_tx_done(struct iwl_softc *sc, int qid, int idx, int nframes,
    void *stat)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct iwl_tx_ring *ring = &sc->txq[qid];
	struct iwl_tx_data *data;
	struct mbuf *m;
	struct iwl_node *wn;
	struct ieee80211_node *ni;
	struct ieee80211vap *vap;
	struct ieee80211_tx_ampdu *tap;
	uint64_t bitmap;
	uint32_t *status = stat;
	uint16_t *aggstatus = stat;
	uint8_t tid;
	int bit, i, lastidx, seqno, shift, start;

#ifdef NOT_YET
	if (nframes == 1) {
		if ((*status & 0xff) != 1 && (*status & 0xff) != 2)
			printf("ieee80211_send_bar()\n");
	}
#endif

	bitmap = 0;
	start = idx;
	for (i = 0; i < nframes; i++) {
		if (le16toh(aggstatus[i * 2]) & 0xc)
			continue;

		idx = le16toh(aggstatus[2*i + 1]) & 0xff;
		bit = idx - start;
		shift = 0;
		if (bit >= 64) {
			shift = 0x100 - idx + start;
			bit = 0;
			start = idx;
		} else if (bit <= -64)
			bit = 0x100 - start + idx;
		else if (bit < 0) {
			shift = start - idx;
			start = idx;
			bit = 0;
		}
		bitmap = bitmap << shift;
		bitmap |= 1ULL << bit;
	}
	tap = sc->qid2tap[qid];
	tid = tap->txa_tid;
	wn = (void *)tap->txa_ni;
	wn->agg[tid].bitmap = bitmap;
	wn->agg[tid].startidx = start;
	wn->agg[tid].nframes = nframes;

	seqno = le32toh(*(status + nframes)) & 0xfff;
	for (lastidx = (seqno & 0xff); ring->read != lastidx;) {
		data = &ring->data[ring->read];

		KASSERT(data->ni != NULL, ("no node"));

		/* Unmap and free mbuf. */
		bus_dmamap_sync(ring->data_dmat, data->map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(ring->data_dmat, data->map);
		m = data->m, data->m = NULL;
		ni = data->ni, data->ni = NULL;
		vap = ni->ni_vap;

		if (m->m_flags & M_TXCB)
			ieee80211_process_callback(ni, m, 1);

		m_freem(m);
		ieee80211_free_node(ni);

		ring->queued--;
		ring->read = (ring->read + 1) % IWL_TX_RING_COUNT;
	}

	sc->sc_tx_timer = 0;
	if (ring->queued < IWL_TX_RING_LOMARK) {
		sc->qfullmsk &= ~(1 << ring->qid);
		if (sc->qfullmsk == 0 &&
		    (ifp->if_drv_flags & IFF_DRV_OACTIVE)) {
			ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
			iwl_start_locked(ifp);
		}
	}
}

/* CARD_STATE_NOTIFICATION */ 
#define HW_CARD_DISABLED               0x01
#define SW_CARD_DISABLED               0x02
#define CT_CARD_DISABLED               0x04
#define RXON_CARD_DISABLED             0x10

/*
 * Process an INT_FH_RX or INT_SW_RX interrupt.
 */
static void
iwl_notif_intr(struct iwl_softc *sc)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	uint16_t hw;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	struct ieee80211vap *vapscan = ss->ss_vap;

	bus_dmamap_sync(sc->rxq.stat_dma.tag, sc->rxq.stat_dma.map,
	    BUS_DMASYNC_POSTREAD);

	hw = le16toh(sc->rxq.stat->closed_count) & 0xfff;

	while (sc->rxq.cur != hw) {
		struct iwl_rx_data *data = &sc->rxq.data[sc->rxq.cur];
		struct iwl_rx_desc *desc;
		bus_dmamap_sync(sc->rxq.data_dmat, data->map,
		    BUS_DMASYNC_POSTREAD);
		desc = mtod(data->m, struct iwl_rx_desc *);

		DPRINTF(sc, IWL_DEBUG_RECV,
		    "%s: qid %x idx %d flags %x type %d(%s) len %d\n",
		    __func__, desc->qid & 0xf, desc->idx, desc->flags,
		    desc->type, iwl_intr_str(desc->type),
		    le16toh(desc->len));

		if (!(desc->qid & 0x80))	/* Reply to a command. */
			iwl_cmd_done(sc, desc);

		switch (desc->type) {
		case IWL_RX_PHY:
			iwl_rx_phy(sc, desc, data);
			break;

		case IWL_RX_DONE:		/* 4965AGN only. */
		case IWL_MPDU_RX_DONE:
			/* An 802.11 frame has been received. */
			iwl_rx_done(sc, desc, data);
			break;

		case IWL_RX_COMPRESSED_BA:
			/* A Compressed BlockAck has been received. */
			iwl_rx_compressed_ba(sc, desc, data);
			break;

		case IWL_TX_DONE:
			/* An 802.11 frame has been transmitted. */
			ops->tx_done(sc, desc, data);
			break;

		case IWL_RX_STATISTICS:
		case IWL_BEACON_STATISTICS:
			iwl_rx_statistics(sc, desc, data);
			break;

		case IWL_BEACON_MISSED:
		{
			struct iwl_beacon_missed *miss =
			    (struct iwl_beacon_missed *)(desc + 1);
			int misses;
			int iv_bmissthreshold;
			int flag = 0;
			struct ieee80211vap *vap0 = sc->ivap[IWL_RXON_BSS_CTX];
			struct ieee80211vap *vap1 = sc->ivap[IWL_RXON_PAN_CTX];

			bus_dmamap_sync(sc->rxq.data_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			misses = le32toh(miss->consecutive);

			DPRINTF(sc, IWL_DEBUG_STATE,
			    "%s: beacons missed %d/%d\n", __func__,
			    misses, le32toh(miss->total));

			iv_bmissthreshold = vap0->iv_bmissthreshold;

			if(sc->ctx == IWL_RXON_PAN_CTX) {
				iv_bmissthreshold = vap1->iv_bmissthreshold;
				if (vap0->iv_state == IEEE80211_S_RUN &&
				    vap1->iv_state == IEEE80211_S_RUN &&
				    (ic->ic_flags & IEEE80211_F_SCAN) == 0)
					flag = 1;
			}
			else if (vap0->iv_state == IEEE80211_S_RUN &&
				 (ic->ic_flags & IEEE80211_F_SCAN) == 0)
					flag = 1;

			/*
			 * If more than 5 consecutive beacons are missed,
			 * reinitialize the sensitivity state machine.
			 */
			if (flag == 1) {
				if (misses > 5)
					(void)iwl_init_sensitivity(sc);
				if (misses >= iv_bmissthreshold) {
					IWL_UNLOCK(sc);
					ieee80211_beacon_miss(ic);
					IWL_LOCK(sc);
				}
			}

			break;
		}
		case IWL_UC_READY:
		{
			struct iwl_ucode_info *uc =
			    (struct iwl_ucode_info *)(desc + 1);

			/* The microcontroller is ready. */
			bus_dmamap_sync(sc->rxq.data_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			DPRINTF(sc, IWL_DEBUG_RESET,
			    "microcode alive notification version=%d.%d "
			    "subtype=%x alive=%x\n", uc->major, uc->minor,
			    uc->subtype, le32toh(uc->valid));

			if (le32toh(uc->valid) != 1) {
				device_printf(sc->sc_dev,
				    "microcontroller initialization failed");
				break;
			}
			if (uc->subtype == IWL_UCODE_INIT) {
				/* Save microcontroller report. */
				memcpy(&sc->ucode_info, uc, sizeof (*uc));
			}
			/* Save the address of the error log in SRAM. */
			sc->errptr = le32toh(uc->errptr);
			break;
		}
		case IWL_STATE_CHANGED:
		{
		
			uint32_t *status = (uint32_t *)(desc + 1);
			
			bus_dmamap_sync(sc->rxq.data_dmat, data->map,BUS_DMASYNC_POSTREAD);
		if (*status & (HW_CARD_DISABLED | CT_CARD_DISABLED |SW_CARD_DISABLED))
                {
				IWL_WRITE(sc, IWL_UCODE_GP1_SET, IWL_UCODE_GP1_CMD_BLOCKED);
				IWL_WRITE(sc,IWL_TARG_MBX_C, 0x00000004);
			
				if (!(*status & RXON_CARD_DISABLED)) {
				IWL_WRITE(sc, IWL_UCODE_GP1_CLR, IWL_UCODE_GP1_CMD_BLOCKED);
				IWL_WRITE(sc, IWL_TARG_MBX_C, 0x00000004);
				
				   }
				   
				if(*status & CT_CARD_DISABLED) {
				device_printf(sc->sc_dev,"critical temp. reached\n");
				/* XXX: enter CT kill */
				}
                }  

				if(!(*status & CT_CARD_DISABLED)){
				callout_stop(&sc->ct_kill_exit_to);
				/* XXX: exit CT kill */
			}
			/*
			 * State change allows hardware switch change to be
			 * noted. However, we handle this in iwl_intr as we
			 * get both the enable/disble intr.
			 */
		
			
			device_printf(sc->sc_dev,"state changed to %x\n",
			    le32toh(*status));
			break;
		}
		case IWL_START_SCAN:
		{
			struct iwl_start_scan *scan =
			    (struct iwl_start_scan *)(desc + 1);

			bus_dmamap_sync(sc->rxq.data_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			DPRINTF(sc, IWL_DEBUG_ANY,
			    "%s: scanning channel %d status %x\n",
			    __func__, scan->chan, le32toh(scan->status));
			break;
		}
		case IWL_STOP_SCAN:
		{
			struct iwl_stop_scan *scan =
			    (struct iwl_stop_scan *)(desc + 1);

			bus_dmamap_sync(sc->rxq.data_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			DPRINTF(sc, IWL_DEBUG_STATE,
			    "scan finished nchan=%d status=%d chan=%d\n",
			    scan->nchan, scan->status, scan->chan);

			IWL_UNLOCK(sc);
			ieee80211_scan_next(vapscan);
			IWL_LOCK(sc);
			break;
		}
		case IWL5000_CALIBRATION_RESULT:
			iwl5000_rx_calib_results(sc, desc, data);
			break;

		case IWL5000_CALIBRATION_DONE:
			sc->sc_flags |= IWL_FLAG_CALIB_DONE;
			wakeup(sc);
			break;
		}

		sc->rxq.cur = (sc->rxq.cur + 1) % IWL_RX_RING_COUNT;
	}

	/* Tell the firmware what we have processed. */
	hw = (hw == 0) ? IWL_RX_RING_COUNT - 1 : hw - 1;
	IWL_WRITE(sc, IWL_FH_RX_WPTR, hw & ~7);
}

/*
 * Process an INT_WAKEUP interrupt raised when the microcontroller wakes up
 * from power-down sleep mode.
 */
static void
iwl_wakeup_intr(struct iwl_softc *sc)
{
	int qid;

	DPRINTF(sc, IWL_DEBUG_RESET, "%s: ucode wakeup from power-down sleep\n",
	    __func__);

	/* Wakeup RX and TX rings. */
	IWL_WRITE(sc, IWL_FH_RX_WPTR, sc->rxq.cur & ~7);
	for (qid = 0; qid < sc->ntxqs; qid++) {
		struct iwl_tx_ring *ring = &sc->txq[qid];
		IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, qid << 8 | ring->cur);
	}
}

static void
iwl_rftoggle_intr(struct iwl_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	uint32_t tmp = IWL_READ(sc, IWL_GP_CNTRL);

	IWL_LOCK_ASSERT(sc);

	device_printf(sc->sc_dev, "RF switch: radio %s\n",
	    (tmp & IWL_GP_CNTRL_RFKILL) ? "enabled" : "disabled");
	if (tmp & IWL_GP_CNTRL_RFKILL)
		ieee80211_runtask(ic, &sc->sc_radioon_task);
	else
		ieee80211_runtask(ic, &sc->sc_radiooff_task);
}

/*
 * Dump the error log of the firmware when a firmware panic occurs.  Although
 * we can't debug the firmware because it is neither open source nor free, it
 * can help us to identify certain classes of problems.
 */
static void
iwl_fatal_intr(struct iwl_softc *sc)
{
	struct iwl_fw_dump dump;
	int i;

	IWL_LOCK_ASSERT(sc);

	/* Force a complete recalibration on next init. */
	sc->sc_flags &= ~IWL_FLAG_CALIB_DONE;

	/* Check that the error log address is valid. */
	if (sc->errptr < IWL_FW_DATA_BASE ||
	    sc->errptr + sizeof (dump) >
	    IWL_FW_DATA_BASE + sc->fw_data_maxsz) {
		printf("%s: bad firmware error log address 0x%08x\n", __func__,
		    sc->errptr);
		return;
	}
	if (iwl_nic_lock(sc) != 0) {
		printf("%s: could not read firmware error log\n", __func__);
		return;
	}
	/* Read firmware error log from SRAM. */
	iwl_mem_read_region_4(sc, sc->errptr, (uint32_t *)&dump,
	    sizeof (dump) / sizeof (uint32_t));
	iwl_nic_unlock(sc);

	if (dump.valid == 0) {
		printf("%s: firmware error log is empty\n", __func__);
		return;
	}
	printf("firmware error log:\n");
	printf("  error type      = \"%s\" (0x%08X)\n",
	    (dump.id < nitems(iwl_fw_errmsg)) ?
		iwl_fw_errmsg[dump.id] : "UNKNOWN",
	    dump.id);
	printf("  program counter = 0x%08X\n", dump.pc);
	printf("  source line     = 0x%08X\n", dump.src_line);
	printf("  error data      = 0x%08X%08X\n",
	    dump.error_data[0], dump.error_data[1]);
	printf("  branch link     = 0x%08X%08X\n",
	    dump.branch_link[0], dump.branch_link[1]);
	printf("  interrupt link  = 0x%08X%08X\n",
	    dump.interrupt_link[0], dump.interrupt_link[1]);
	printf("  time            = %u\n", dump.time[0]);

	/* Dump driver status (TX and RX rings) while we're here. */
	printf("driver status:\n");
	for (i = 0; i < sc->ntxqs; i++) {
		struct iwl_tx_ring *ring = &sc->txq[i];
		printf("  tx ring %2d: qid=%-2d cur=%-3d queued=%-3d\n",
		    i, ring->qid, ring->cur, ring->queued);
	}
	printf("  rx ring: cur=%d\n", sc->rxq.cur);
}

void
iwl_intr(void *arg)
{
	struct iwl_softc *sc = arg;
	struct ifnet *ifp = sc->sc_ifp;
	uint32_t r1, r2, tmp;

	IWL_LOCK(sc);

	/* Disable interrupts. */
	IWL_WRITE(sc, IWL_INT_MASK, 0);

	/* Read interrupts from ICT (fast) or from registers (slow). */
	if (sc->sc_flags & IWL_FLAG_USE_ICT) {
		tmp = 0;
		while (sc->ict[sc->ict_cur] != 0) {
			tmp |= sc->ict[sc->ict_cur];
			sc->ict[sc->ict_cur] = 0;	/* Acknowledge. */
			sc->ict_cur = (sc->ict_cur + 1) % IWL_ICT_COUNT;
		}
		tmp = le32toh(tmp);
		if (tmp == 0xffffffff)	/* Shouldn't happen. */
			tmp = 0;
		else if (tmp & 0xc0000)	/* Workaround a HW bug. */
			tmp |= 0x8000;
		r1 = (tmp & 0xff00) << 16 | (tmp & 0xff);
		r2 = 0;	/* Unused. */
	} else {
		r1 = IWL_READ(sc, IWL_INT);
		if (r1 == 0xffffffff || (r1 & 0xfffffff0) == 0xa5a5a5a0)
			return;	/* Hardware gone! */
		r2 = IWL_READ(sc, IWL_FH_INT);
	}

	DPRINTF(sc, IWL_DEBUG_INTR, "interrupt reg1=%x reg2=%x\n", r1, r2);

	if (r1 == 0 && r2 == 0)
		goto done;	/* Interrupt not for us. */

	/* Acknowledge interrupts. */
	IWL_WRITE(sc, IWL_INT, r1);
	if (!(sc->sc_flags & IWL_FLAG_USE_ICT))
		IWL_WRITE(sc, IWL_FH_INT, r2);

	if (r1 & IWL_INT_RF_TOGGLED) {
		iwl_rftoggle_intr(sc);
		goto done;
	}
	if (r1 & IWL_INT_CT_REACHED) {
		device_printf(sc->sc_dev, "%s: critical temperature reached!\n",
		    __func__);
	}
	if (r1 & (IWL_INT_SW_ERR | IWL_INT_HW_ERR)) {
		device_printf(sc->sc_dev, "%s: fatal firmware error\n",
		    __func__);
		/* Dump firmware error log and stop. */
		iwl_fatal_intr(sc);
		ifp->if_flags &= ~IFF_UP;
		iwl_stop_locked(sc);
		goto done;
	}
	if ((r1 & (IWL_INT_FH_RX | IWL_INT_SW_RX | IWL_INT_RX_PERIODIC)) ||
	    (r2 & IWL_FH_INT_RX)) {
		if (sc->sc_flags & IWL_FLAG_USE_ICT) {
			if (r1 & (IWL_INT_FH_RX | IWL_INT_SW_RX))
				IWL_WRITE(sc, IWL_FH_INT, IWL_FH_INT_RX);
			IWL_WRITE_1(sc, IWL_INT_PERIODIC,
			    IWL_INT_PERIODIC_DIS);
			iwl_notif_intr(sc);
			if (r1 & (IWL_INT_FH_RX | IWL_INT_SW_RX)) {
				IWL_WRITE_1(sc, IWL_INT_PERIODIC,
				    IWL_INT_PERIODIC_ENA);
			}
		} else
			iwl_notif_intr(sc);
	}

	if ((r1 & IWL_INT_FH_TX) || (r2 & IWL_FH_INT_TX)) {
		if (sc->sc_flags & IWL_FLAG_USE_ICT)
			IWL_WRITE(sc, IWL_FH_INT, IWL_FH_INT_TX);
		wakeup(sc);	/* FH DMA transfer completed. */
	}

	if (r1 & IWL_INT_ALIVE)
		wakeup(sc);	/* Firmware is alive. */

	if (r1 & IWL_INT_WAKEUP)
		iwl_wakeup_intr(sc);

done:
	/* Re-enable interrupts. */
	if (ifp->if_flags & IFF_UP)
		IWL_WRITE(sc, IWL_INT_MASK, sc->int_mask);

	IWL_UNLOCK(sc);
}

static void
iwl_start(struct ifnet *ifp)
{
	struct iwl_softc *sc = ifp->if_softc;

	IWL_LOCK(sc);
	iwl_start_locked(ifp);
	IWL_UNLOCK(sc);
}

static void
iwl_start_locked(struct ifnet *ifp)
{
	struct iwl_softc *sc = ifp->if_softc;
	struct ieee80211_node *ni;
	struct mbuf *m;

	IWL_LOCK_ASSERT(sc);

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0 ||
	    (ifp->if_drv_flags & IFF_DRV_OACTIVE))
		return;

	for (;;) {
		if (sc->qfullmsk != 0) {
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}
		IFQ_DRV_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;
		ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
		if (iwl_tx_data(sc, m, ni) != 0) {
			ieee80211_free_node(ni);
			ifp->if_oerrors++;
			continue;
		}
		sc->sc_tx_timer = 5;
	}
}

static void
iwl_watchdog(void *arg)
{
	struct iwl_softc *sc = arg;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	struct ieee80211vap *vapscan = ss->ss_vap;

	IWL_LOCK_ASSERT(sc);

	KASSERT(ifp->if_drv_flags & IFF_DRV_RUNNING, ("not running"));

	if (sc->sc_tx_timer > 0) {
		if (--sc->sc_tx_timer == 0) {
			if_printf(ifp, "device timeout\n");
			ieee80211_runtask(ic, &sc->sc_reinit_task);
			return;
		}
	}

	if (sc->sc_scan_timer > 0) {
		if (--sc->sc_scan_timer == 0)
			ieee80211_scan_next(vapscan);
	}

	callout_reset(&sc->watchdog_to, hz, iwl_watchdog, sc);
}

static int
iwl_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct iwl_softc *sc = ifp->if_softc;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct ifreq *ifr = (struct ifreq *) data;
	int error = 0, startall = 0, stop = 0;

	switch (cmd) {
	case SIOCGIFADDR:
		error = ether_ioctl(ifp, cmd, data);
		break;
	case SIOCSIFFLAGS:
		IWL_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (!(ifp->if_drv_flags & IFF_DRV_RUNNING)) {
				iwl_init_locked(sc);
				if (IWL_READ(sc, IWL_GP_CNTRL) & IWL_GP_CNTRL_RFKILL)
					startall = 1;
				else
					stop = 1;
			}
		} else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				iwl_stop_locked(sc);
		}
		IWL_UNLOCK(sc);
		if (startall)
			ieee80211_start_all(ic);
		else if (vap != NULL && stop)
			ieee80211_stop(vap);
		break;
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &ic->ic_media, cmd);
		break;
	default:
		error = EINVAL;
		break;
	}
	return error;
}

static int
iwl5000_add_node(struct iwl_softc *sc, struct iwl_node_info *node, int async)
{
	/* Direct mapping. */
	return iwl_cmd(sc, IWL_CMD_ADD_NODE, node, sizeof (*node), async);
}

/*
 * Broadcast node is used to send group-addressed and management frames.
 */
static int
iwl_add_broadcast_node(struct iwl_softc *sc, int async)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct iwl_node_info node;
	struct iwl_cmd_link_quality linkq;
	uint8_t txant;
	int i, error;

	sc->rxon = &sc->rx_on[IWL_RXON_BSS_CTX];

	memset(&node, 0, sizeof node);
	IEEE80211_ADDR_COPY(node.macaddr, ifp->if_broadcastaddr);
	node.id = IWL_BROADCAST_ID;
	DPRINTF(sc, IWL_DEBUG_RESET, "%s: adding broadcast node\n", __func__);
	if ((error = ops->add_node(sc, &node, async)) != 0)
		return error;

	/* Use the first valid TX antenna. */
	txant = IWL_LSB(sc->txchainmask);

	memset(&linkq, 0, sizeof linkq);
	linkq.id = IWL_BROADCAST_ID;
	linkq.antmsk_1stream = txant;
	linkq.antmsk_2stream = IWL_ANT_AB;
	linkq.ampdu_max = 64;
	linkq.ampdu_threshold = 3;
	linkq.ampdu_limit = htole16(4000);	/* 4ms */

	/* Use lowest mandatory bit-rate. */
	if (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan))
		linkq.retry[0] = htole32(0xd);
	else
		linkq.retry[0] = htole32(10 | IWL_RFLAG_CCK);
	linkq.retry[0] |= htole32(IWL_RFLAG_ANT(txant));
	/* Use same bit-rate for all TX retries. */
	for (i = 1; i < IWL_MAX_TX_RETRIES; i++) {
		linkq.retry[i] = linkq.retry[0];
	}
	return iwl_cmd(sc, IWL_CMD_LINK_QUALITY, &linkq, sizeof linkq, async);
}

/*
 * Broadcast node is used to send group-addressed and management frames.
 */
static int
iwl_add_broadcast_node1(struct iwl_softc *sc, int async)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct iwl_node_info node;
	struct iwl_cmd_link_quality linkq;
	uint8_t txant;
	int i, error;

	sc->rxon = &sc->rx_on[IWL_RXON_PAN_CTX];

	memset(&node, 0, sizeof node);
	IEEE80211_ADDR_COPY(node.macaddr, ifp->if_broadcastaddr);

	node.id = IWL_PAN_BCAST_ID;
	node.htflags |= htole32(STA_FLAG_PAN_STATION);
	DPRINTF(sc, IWL_DEBUG_RESET, "%s: adding broadcast node1\n", __func__);
	if ((error = ops->add_node(sc, &node, async)) != 0)
		return error;

	/* Use the first valid TX antenna. */
	txant = IWL_LSB(sc->txchainmask);

	memset(&linkq, 0, sizeof linkq);
	linkq.id = IWL_PAN_BCAST_ID;
	linkq.antmsk_1stream = txant;
	linkq.antmsk_2stream = IWL_ANT_AB;
	linkq.ampdu_max = 64;
	linkq.ampdu_threshold = 3;
	linkq.ampdu_limit = htole16(4000);	/* 4ms */

	/* Use lowest mandatory bit-rate. */
	if (IEEE80211_IS_CHAN_5GHZ(ic->ic_curchan))
		linkq.retry[0] = htole32(0xd);
	else
		linkq.retry[0] = htole32(10 | IWL_RFLAG_CCK);
	linkq.retry[0] |= htole32(IWL_RFLAG_ANT(txant));
	/* Use same bit-rate for all TX retries. */
	for (i = 1; i < IWL_MAX_TX_RETRIES; i++) {
		linkq.retry[i] = linkq.retry[0];
	}
	return iwl_cmd(sc, IWL_CMD_LINK_QUALITY, &linkq, sizeof linkq, async);
}

static int
iwl_updateedca(struct ieee80211com *ic)
{
#define IWL_EXP2(x)	((1 << (x)) - 1)	/* CWmin = 2^ECWmin - 1 */
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	struct iwl_edca_params cmd;
	int aci;

	memset(&cmd, 0, sizeof cmd);
	cmd.flags = htole32(IWL_EDCA_UPDATE);
	for (aci = 0; aci < WME_NUM_AC; aci++) {
		const struct wmeParams *ac =
		    &ic->ic_wme.wme_chanParams.cap_wmeParams[aci];
		cmd.ac[aci].aifsn = ac->wmep_aifsn;
		cmd.ac[aci].cwmin = htole16(IWL_EXP2(ac->wmep_logcwmin));
		cmd.ac[aci].cwmax = htole16(IWL_EXP2(ac->wmep_logcwmax));
		cmd.ac[aci].txoplimit =
		    htole16(IEEE80211_TXOP_TO_US(ac->wmep_txopLimit));
	}
	IEEE80211_UNLOCK(ic);
	IWL_LOCK(sc);
	(void)iwl_cmd(sc, IWL_CMD_EDCA_PARAMS, &cmd, sizeof cmd, 1);
	IWL_UNLOCK(sc);
	IEEE80211_LOCK(ic);
	return 0;
#undef IWL_EXP2
}

static int
iwl_updateedca1(struct ieee80211com *ic)
{
#define IWL_EXP2(x)	((1 << (x)) - 1)	/* CWmin = 2^ECWmin - 1 */
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	struct iwl_edca_params cmd;
	int aci;

	memset(&cmd, 0, sizeof cmd);
	cmd.flags = htole32(IWL_EDCA_UPDATE);
	for (aci = 0; aci < WME_NUM_AC; aci++) {
		const struct wmeParams *ac =
		    &ic->ic_wme.wme_chanParams.cap_wmeParams[aci];
		cmd.ac[aci].aifsn = ac->wmep_aifsn;
		cmd.ac[aci].cwmin = htole16(IWL_EXP2(ac->wmep_logcwmin));
		cmd.ac[aci].cwmax = htole16(IWL_EXP2(ac->wmep_logcwmax));
		cmd.ac[aci].txoplimit =
		    htole16(IEEE80211_TXOP_TO_US(ac->wmep_txopLimit));
	}
	return iwl_cmd(sc, IWL_CMD_WIPAN_QOS_PARAM, &cmd, sizeof cmd, 1);
#undef IWL_EXP2
}

static void
iwl_update_mcast(struct ifnet *ifp)
{
	/* Ignore */
}

#define CT_KILL_THRESHOLD              114             /* in Celsius */
#define CT_KILL_EXIT_THRESHOLD         95              /* in Celsius */

/*
 * Set the critical temperature at which the firmware will stop the radio
 * and notify us.
 */
static int
iwl_set_critical_temp(struct iwl_softc *sc)
{
	struct iwl_critical_temp crit;
	int32_t ct_enter,ct_exit;
	IWL_WRITE(sc, IWL_UCODE_GP1_CLR, IWL_UCODE_GP1_CT_KILL_EXIT);	

	ct_enter = CT_KILL_THRESHOLD;
	ct_exit = CT_KILL_EXIT_THRESHOLD;
	memset(&crit, 0, sizeof crit);
	crit.tempR = htole32(ct_enter);
	crit.tempM = htole32(ct_exit);
	DPRINTF(sc, IWL_DEBUG_RESET, "setting critical temp to %d\n", ct_enter);
	return iwl_cmd(sc, IWL_CMD_SET_CRITICAL_TEMP, &crit, sizeof crit, 0);
}

static int
iwl_set_timing(struct iwl_softc *sc, struct ieee80211_node *ni)
{
	struct iwl_cmd_timing cmd;
	uint64_t val, mod;

	memset(&cmd, 0, sizeof cmd);
	memcpy(&cmd.tstamp, ni->ni_tstamp.data, sizeof (uint64_t));
	cmd.bintval = htole16(ni->ni_intval);
	cmd.lintval = htole16(10);

	/* Compute remaining time until next beacon. */
	val = (uint64_t)ni->ni_intval * IEEE80211_DUR_TU;
	mod = le64toh(cmd.tstamp) % val;
	cmd.binitval = htole32((uint32_t)(val - mod));

	DPRINTF(sc, IWL_DEBUG_RESET, "timing bintval=%u tstamp=%ju, init=%u\n",
	    ni->ni_intval, le64toh(cmd.tstamp), (uint32_t)(val - mod));

	return iwl_cmd(sc, IWL_CMD_TIMING, &cmd, sizeof cmd, 0);
}

static int
iwl_set_timing1(struct iwl_softc *sc)
{
	struct iwl_cmd_timing cmd;
	int error = 0;
	struct ieee80211vap *vap;
	struct iwl_vap *ivp;

	vap = sc->ivap[IWL_RXON_PAN_CTX];
	ivp = IWL_VAP(vap);

	if ((error = iwl_config1(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: iwl_config1 error %d\n", __func__, error);
		return error;
	}

	if ((error = iwl_set_pan_params(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: iwl_set_pan_params error %d\n", __func__, error);
		return error;
	}

	memset(&cmd, 0, sizeof cmd);
	cmd.lintval = htole16(10);
	cmd.bintval = htole16(BEACON_INTERVAL_DEFAULT);
	ivp->beacon_int = cmd.bintval;
	cmd.binitval = htole32(0x032000);
	cmd.dtim_period = 1;

	return iwl_cmd(sc, IWL_CMD_WIPAN_RXON_TIMING, &cmd, sizeof cmd, 0);
}

/*
 * Set TX power for current channel (each rate has its own power settings).
 * This function takes into account the regulatory information from EEPROM,
 * the current temperature and the current voltage.
 */

static int
iwl5000_set_txpower(struct iwl_softc *sc, struct ieee80211_channel *ch,
    int async)
{
	struct iwl5000_cmd_txpower cmd;

	/*
	 * TX power calibration is handled automatically by the firmware
	 * for 5000 Series.
	 */
	memset(&cmd, 0, sizeof cmd);
	cmd.global_limit = 2 * IWL5000_TXPOWER_MAX_DBM;	/* 16 dBm */
	cmd.flags = IWL5000_TXPOWER_NO_CLOSED;
	cmd.srv_limit = IWL5000_TXPOWER_AUTO;
	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s: setting TX power\n", __func__);
	return iwl_cmd(sc, IWL_CMD_TXPOWER_DBM, &cmd, sizeof cmd, async);
}

/*
 * Retrieve the maximum RSSI (in dBm) among receivers.
 */

static int
iwl5000_get_rssi(struct iwl_softc *sc, struct iwl_rx_stat *stat)
{
	struct iwl5000_rx_phystat *phy = (void *)stat->phybuf;
	uint8_t agc;
	int rssi;

	agc = (le32toh(phy->agc) >> 9) & 0x7f;

	rssi = MAX(le16toh(phy->rssi[0]) & 0xff,
		   le16toh(phy->rssi[1]) & 0xff);
	rssi = MAX(le16toh(phy->rssi[2]) & 0xff, rssi);

	DPRINTF(sc, IWL_DEBUG_RECV,
	    "%s: agc %d rssi %d %d %d result %d\n", __func__, agc,
	    phy->rssi[0], phy->rssi[1], phy->rssi[2],
	    rssi - agc - IWL_RSSI_TO_DBM);
	return rssi - agc - IWL_RSSI_TO_DBM;
}

/*
 * Retrieve the average noise (in dBm) among receivers.
 */
static int
iwl_get_noise(const struct iwl_rx_general_stats *stats)
{
	int i, total, nbant, noise;

	total = nbant = 0;
	for (i = 0; i < 3; i++) {
		if ((noise = le32toh(stats->noise[i]) & 0xff) == 0)
			continue;
		total += noise;
		nbant++;
	}
	/* There should be at least one antenna but check anyway. */
	return (nbant == 0) ? -127 : (total / nbant) - 107;
}

/*
 * Compute temperature (in degC) from last received statistics.
 */

static void
iwl5000_get_temperature(struct iwl_softc *sc)
{
	int32_t temp;

	/*
	 * Temperature is not used by the driver for 5000 Series because
	 * TX power calibration is handled by firmware.
	 */
	temp = le32toh(sc->rawtemp);
	if (sc->hw_type == IWL_HW_REV_TYPE_5150) {
		temp = (temp / -5) + sc->temp_off;
		temp = IWL_KTOC(temp);
	}
	//XXX: handle thermal throttling state machine
}

/*
 * Initialize sensitivity calibration state machine.
 */
int
iwl_init_sensitivity(struct iwl_softc *sc)
{
	struct iwl_ops *ops = &sc->ops;
	struct iwl_calib_state *calib = &sc->calib;
	uint32_t flags;
	int error;

	/* Reset calibration state machine. */
	memset(calib, 0, sizeof (*calib));
	calib->state = IWL_CALIB_STATE_INIT;
	calib->cck_state = IWL_CCK_STATE_HIFA;
	/* Set initial correlation values. */
	calib->ofdm_x1     = sc->limits->min_ofdm_x1;
	calib->ofdm_mrc_x1 = sc->limits->min_ofdm_mrc_x1;
	calib->ofdm_x4     = sc->limits->min_ofdm_x4;
	calib->ofdm_mrc_x4 = sc->limits->min_ofdm_mrc_x4;
	calib->cck_x4      = 125;
	calib->cck_mrc_x4  = sc->limits->min_cck_mrc_x4;
	calib->energy_cck  = sc->limits->energy_cck;

	/* Write initial sensitivity. */
	if ((error = iwl_send_sensitivity(sc)) != 0)
		return error;

	/* Write initial gains. */
	if ((error = ops->init_gains(sc)) != 0)
		return error;

	/* Request statistics at each beacon interval. */
	flags = 0;
	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s: sending request for statistics\n",
	    __func__);
	return iwl_cmd(sc, IWL_CMD_GET_STATISTICS, &flags, sizeof flags, 1);
}

/*
 * Collect noise and RSSI statistics for the first 20 beacons received
 * after association and use them to determine connected antennas and
 * to set differential gains.
 */
static void
iwl_collect_noise(struct iwl_softc *sc,
    const struct iwl_rx_general_stats *stats)
{
	struct iwl_ops *ops = &sc->ops;
	struct iwl_calib_state *calib = &sc->calib;
	uint32_t val;
	int i;

	/* Accumulate RSSI and noise for all 3 antennas. */
	for (i = 0; i < 3; i++) {
		calib->rssi[i] += le32toh(stats->rssi[i]) & 0xff;
		calib->noise[i] += le32toh(stats->noise[i]) & 0xff;
	}
	/* NB: We update differential gains only once after 20 beacons. */
	if (++calib->nbeacons < 20)
		return;

	/* Determine highest average RSSI. */
	val = MAX(calib->rssi[0], calib->rssi[1]);
	val = MAX(calib->rssi[2], val);

	/* Determine which antennas are connected. */
	sc->chainmask = sc->rxchainmask;
	for (i = 0; i < 3; i++)
		if (val - calib->rssi[i] > 15 * 20)
			sc->chainmask &= ~(1 << i);
	DPRINTF(sc, IWL_DEBUG_CALIBRATE,
	    "%s: RX chains mask: theoretical=0x%x, actual=0x%x\n",
	    __func__, sc->rxchainmask, sc->chainmask);

	/* If none of the TX antennas are connected, keep at least one. */
	if ((sc->chainmask & sc->txchainmask) == 0)
		sc->chainmask |= IWL_LSB(sc->txchainmask);

	(void)ops->set_gains(sc);
	calib->state = IWL_CALIB_STATE_RUN;

#ifdef notyet
	/* XXX Disable RX chains with no antennas connected. */
	sc->rxon->rxchain = htole16(IWL_RXCHAIN_SEL(sc->chainmask));
	(void)iwl_cmd(sc, IWL_CMD_RXON, sc->rxon, sc->rxonsz, 0);
#endif

#if 0
	/* XXX: not yet */
	/* Enable power-saving mode if requested by user. */
	if (sc->sc_ic.ic_flags & IEEE80211_F_PMGTON)
		(void)iwl_set_pslevel(sc, 0, 3, 1);
#endif
}

static int
iwl5000_init_gains(struct iwl_softc *sc)
{
	struct iwl_phy_calib cmd;

	memset(&cmd, 0, sizeof cmd);
	cmd.code = sc->reset_noise_gain;
	cmd.ngroups = 1;
	cmd.isvalid = 1;
	DPRINTF(sc, IWL_DEBUG_CALIBRATE,
	    "%s: setting initial differential gains\n", __func__);
	return iwl_cmd(sc, IWL_CMD_PHY_CALIB, &cmd, sizeof cmd, 1);
}

static int
iwl5000_set_gains(struct iwl_softc *sc)
{
	struct iwl_calib_state *calib = &sc->calib;
	struct iwl_phy_calib_gain cmd;
	int i, ant, div, delta;

	/* We collected 20 beacons and !=6050 need a 1.5 factor. */
	div = (sc->hw_type == IWL_HW_REV_TYPE_6050) ? 20 : 30;

	memset(&cmd, 0, sizeof cmd);
	cmd.code = sc->noise_gain;
	cmd.ngroups = 1;
	cmd.isvalid = 1;
	/* Get first available RX antenna as referential. */
	ant = IWL_LSB(sc->rxchainmask);
	/* Set differential gains for other antennas. */
	for (i = ant + 1; i < 3; i++) {
		if (sc->chainmask & (1 << i)) {
			/* The delta is relative to antenna "ant". */
			delta = ((int32_t)calib->noise[ant] -
			    (int32_t)calib->noise[i]) / div;
			/* Limit to [-4.5dB,+4.5dB]. */
			cmd.gain[i - 1] = MIN(abs(delta), 3);
			if (delta < 0)
				cmd.gain[i - 1] |= 1 << 2;	/* sign bit */
		}
	}
	DPRINTF(sc, IWL_DEBUG_CALIBRATE,
	    "setting differential gains Ant B/C: %x/%x (%x)\n",
	    cmd.gain[0], cmd.gain[1], sc->chainmask);
	return iwl_cmd(sc, IWL_CMD_PHY_CALIB, &cmd, sizeof cmd, 1);
}

/*
 * Tune RF RX sensitivity based on the number of false alarms detected
 * during the last beacon period.
 */
static void
iwl_tune_sensitivity(struct iwl_softc *sc, const struct iwl_rx_stats *stats)
{
#define inc(val, inc, max)			\
	if ((val) < (max)) {			\
		if ((val) < (max) - (inc))	\
			(val) += (inc);		\
		else				\
			(val) = (max);		\
		needs_update = 1;		\
	}
#define dec(val, dec, min)			\
	if ((val) > (min)) {			\
		if ((val) > (min) + (dec))	\
			(val) -= (dec);		\
		else				\
			(val) = (min);		\
		needs_update = 1;		\
	}

	const struct iwl_sensitivity_limits *limits = sc->limits;
	struct iwl_calib_state *calib = &sc->calib;
	uint32_t val, rxena, fa;
	uint32_t energy[3], energy_min;
	uint8_t noise[3], noise_ref;
	int i, needs_update = 0;

	/* Check that we've been enabled long enough. */
	if ((rxena = le32toh(stats->general.load)) == 0)
		return;

	/* Compute number of false alarms since last call for OFDM. */
	fa  = le32toh(stats->ofdm.bad_plcp) - calib->bad_plcp_ofdm;
	fa += le32toh(stats->ofdm.fa) - calib->fa_ofdm;
	fa *= 200 * IEEE80211_DUR_TU;	/* 200TU */

	/* Save counters values for next call. */
	calib->bad_plcp_ofdm = le32toh(stats->ofdm.bad_plcp);
	calib->fa_ofdm = le32toh(stats->ofdm.fa);

	if (fa > 50 * rxena) {
		/* High false alarm count, decrease sensitivity. */
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "%s: OFDM high false alarm count: %u\n", __func__, fa);
		inc(calib->ofdm_x1,     1, limits->max_ofdm_x1);
		inc(calib->ofdm_mrc_x1, 1, limits->max_ofdm_mrc_x1);
		inc(calib->ofdm_x4,     1, limits->max_ofdm_x4);
		inc(calib->ofdm_mrc_x4, 1, limits->max_ofdm_mrc_x4);

	} else if (fa < 5 * rxena) {
		/* Low false alarm count, increase sensitivity. */
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "%s: OFDM low false alarm count: %u\n", __func__, fa);
		dec(calib->ofdm_x1,     1, limits->min_ofdm_x1);
		dec(calib->ofdm_mrc_x1, 1, limits->min_ofdm_mrc_x1);
		dec(calib->ofdm_x4,     1, limits->min_ofdm_x4);
		dec(calib->ofdm_mrc_x4, 1, limits->min_ofdm_mrc_x4);
	}

	/* Compute maximum noise among 3 receivers. */
	for (i = 0; i < 3; i++)
		noise[i] = (le32toh(stats->general.noise[i]) >> 8) & 0xff;
	val = MAX(noise[0], noise[1]);
	val = MAX(noise[2], val);
	/* Insert it into our samples table. */
	calib->noise_samples[calib->cur_noise_sample] = val;
	calib->cur_noise_sample = (calib->cur_noise_sample + 1) % 20;

	/* Compute maximum noise among last 20 samples. */
	noise_ref = calib->noise_samples[0];
	for (i = 1; i < 20; i++)
		noise_ref = MAX(noise_ref, calib->noise_samples[i]);

	/* Compute maximum energy among 3 receivers. */
	for (i = 0; i < 3; i++)
		energy[i] = le32toh(stats->general.energy[i]);
	val = MIN(energy[0], energy[1]);
	val = MIN(energy[2], val);
	/* Insert it into our samples table. */
	calib->energy_samples[calib->cur_energy_sample] = val;
	calib->cur_energy_sample = (calib->cur_energy_sample + 1) % 10;

	/* Compute minimum energy among last 10 samples. */
	energy_min = calib->energy_samples[0];
	for (i = 1; i < 10; i++)
		energy_min = MAX(energy_min, calib->energy_samples[i]);
	energy_min += 6;

	/* Compute number of false alarms since last call for CCK. */
	fa  = le32toh(stats->cck.bad_plcp) - calib->bad_plcp_cck;
	fa += le32toh(stats->cck.fa) - calib->fa_cck;
	fa *= 200 * IEEE80211_DUR_TU;	/* 200TU */

	/* Save counters values for next call. */
	calib->bad_plcp_cck = le32toh(stats->cck.bad_plcp);
	calib->fa_cck = le32toh(stats->cck.fa);

	if (fa > 50 * rxena) {
		/* High false alarm count, decrease sensitivity. */
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "%s: CCK high false alarm count: %u\n", __func__, fa);
		calib->cck_state = IWL_CCK_STATE_HIFA;
		calib->low_fa = 0;

		if (calib->cck_x4 > 160) {
			calib->noise_ref = noise_ref;
			if (calib->energy_cck > 2)
				dec(calib->energy_cck, 2, energy_min);
		}
		if (calib->cck_x4 < 160) {
			calib->cck_x4 = 161;
			needs_update = 1;
		} else
			inc(calib->cck_x4, 3, limits->max_cck_x4);

		inc(calib->cck_mrc_x4, 3, limits->max_cck_mrc_x4);

	} else if (fa < 5 * rxena) {
		/* Low false alarm count, increase sensitivity. */
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "%s: CCK low false alarm count: %u\n", __func__, fa);
		calib->cck_state = IWL_CCK_STATE_LOFA;
		calib->low_fa++;

		if (calib->cck_state != IWL_CCK_STATE_INIT &&
		    (((int32_t)calib->noise_ref - (int32_t)noise_ref) > 2 ||
		     calib->low_fa > 100)) {
			inc(calib->energy_cck, 2, limits->min_energy_cck);
			dec(calib->cck_x4,     3, limits->min_cck_x4);
			dec(calib->cck_mrc_x4, 3, limits->min_cck_mrc_x4);
		}
	} else {
		/* Not worth to increase or decrease sensitivity. */
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "%s: CCK normal false alarm count: %u\n", __func__, fa);
		calib->low_fa = 0;
		calib->noise_ref = noise_ref;

		if (calib->cck_state == IWL_CCK_STATE_HIFA) {
			/* Previous interval had many false alarms. */
			dec(calib->energy_cck, 8, energy_min);
		}
		calib->cck_state = IWL_CCK_STATE_INIT;
	}

	if (needs_update)
		(void)iwl_send_sensitivity(sc);
#undef dec
#undef inc
}

static int
iwl_send_sensitivity(struct iwl_softc *sc)
{
	struct iwl_calib_state *calib = &sc->calib;
	struct iwl_enhanced_sensitivity_cmd cmd;
	int len;

	memset(&cmd, 0, sizeof cmd);
	len = sizeof (struct iwl_sensitivity_cmd);
	cmd.which = IWL_SENSITIVITY_WORKTBL;
	/* OFDM modulation. */
	cmd.corr_ofdm_x1       = htole16(calib->ofdm_x1);
	cmd.corr_ofdm_mrc_x1   = htole16(calib->ofdm_mrc_x1);
	cmd.corr_ofdm_x4       = htole16(calib->ofdm_x4);
	cmd.corr_ofdm_mrc_x4   = htole16(calib->ofdm_mrc_x4);
	cmd.energy_ofdm        = htole16(sc->limits->energy_ofdm);
	cmd.energy_ofdm_th     = htole16(62);
	/* CCK modulation. */
	cmd.corr_cck_x4        = htole16(calib->cck_x4);
	cmd.corr_cck_mrc_x4    = htole16(calib->cck_mrc_x4);
	cmd.energy_cck         = htole16(calib->energy_cck);
	/* Barker modulation: use default values. */
	cmd.corr_barker        = htole16(190);
	cmd.corr_barker_mrc    = htole16(390);

	DPRINTF(sc, IWL_DEBUG_CALIBRATE,
	    "%s: set sensitivity %d/%d/%d/%d/%d/%d/%d\n", __func__,
	    calib->ofdm_x1, calib->ofdm_mrc_x1, calib->ofdm_x4,
	    calib->ofdm_mrc_x4, calib->cck_x4,
	    calib->cck_mrc_x4, calib->energy_cck);

	if (!(sc->sc_flags & IWL_FLAG_ENH_SENS))
		goto send;
	/* Enhanced sensitivity settings. */
	len = sizeof (struct iwl_enhanced_sensitivity_cmd);
	cmd.ofdm_det_slope_mrc = htole16(668);
	cmd.ofdm_det_icept_mrc = htole16(4);
	cmd.ofdm_det_slope     = htole16(486);
	cmd.ofdm_det_icept     = htole16(37);
	cmd.cck_det_slope_mrc  = htole16(853);
	cmd.cck_det_icept_mrc  = htole16(4);
	cmd.cck_det_slope      = htole16(476);
	cmd.cck_det_icept      = htole16(99);
send:
	return iwl_cmd(sc, IWL_CMD_SET_SENSITIVITY, &cmd, len, 1);
}

/*
 * Set STA mode power saving level (between 0 and 5).
 * Level 0 is CAM (Continuously Aware Mode), 5 is for maximum power saving.
 * DTIM value passed in will if so configured, be ignored and taken from
 * the value set at the AP.
 */
int
iwl_set_pslevel(struct iwl_softc *sc, int dtim, int level, int async)
{
	struct iwl_pmgt_cmd cmd;
	const struct iwl_pmgt *pmgt;
	uint32_t max, skip_dtim;
	uint32_t reg;
	int i, retval;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;

	if(!TAILQ_EMPTY(&ic->ic_vaps)) {
		/* At present the driver supports only a single vap. */
		struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

		/* XXX confirm PS level setting validity for various modes */
		if(vap->iv_opmode == IEEE80211_M_HOSTAP ||
		    vap->iv_opmode == IEEE80211_M_MONITOR)
			return 0;

		#ifdef IWL_DTIM_INDICATES_UNICAST_PENDING_AT_AP
			if (vap->iv_state == IEEE80211_S_RUN)
				dtim = vap->iv_bss->ni_dtim_period;
		#endif
	}

	/* Select which PS parameters to use. */
	if (dtim <= 2)
		pmgt = &iwl_pmgt[0][level];
	else if (dtim <= 10)
		pmgt = &iwl_pmgt[1][level];
	else
		pmgt = &iwl_pmgt[2][level];

	memset(&cmd, 0, sizeof cmd);
	if (level != 0)	/* not CAM */
		cmd.flags |= htole16(IWL_PS_ALLOW_SLEEP)|htole16(IWL_PS_ADVANCED_PM);
	if (level == 5)
		cmd.flags |= htole16(IWL_PS_FAST_PD);
	/* Retrieve PCIe Active State Power Management (ASPM). */
	reg = pci_read_config(sc->sc_dev, sc->sc_cap_off + 0x10, 1);
	if (!(reg & 0x1))	/* L0s Entry disabled. */
		cmd.flags |= htole16(IWL_PS_PCI_PMGT);
	cmd.rxtimeout = htole32(pmgt->rxtimeout * 1024);
	cmd.txtimeout = htole32(pmgt->txtimeout * 1024);

	if (dtim == 0) {
		dtim = 1;
		skip_dtim = 0;
	} else
		skip_dtim = pmgt->skip_dtim;
	if (skip_dtim != 0) {
		cmd.flags |= htole16(IWL_PS_SLEEP_OVER_DTIM);
		max = pmgt->intval[4];
		if (max == (uint32_t)-1)
			max = dtim * (skip_dtim + 1);
		else if (max > dtim)
			max = (max / dtim) * dtim;
	} else
		max = dtim;
	for (i = 0; i < 5; i++)
		cmd.intval[i] = htole32(MIN(max, pmgt->intval[i]));

	DPRINTF(sc, IWL_DEBUG_RESET, "setting power saving level to %d\n",
	    level);
	retval = iwl_cmd(sc, IWL_CMD_SET_POWER_MODE, &cmd, sizeof cmd, async);
	/*
	 * XXX: update current_pwrsave_level using completion notification
	 * in case of async mode
	 */
	if(retval==0)
		sc->current_pwrsave_level = level;

	return retval;
}

static int
iwl_send_btcoex(struct iwl_softc *sc)
{
	struct iwl_bluetooth cmd;

	memset(&cmd, 0, sizeof cmd);
	cmd.flags = IWL_BT_COEX_CHAN_ANN | IWL_BT_COEX_BT_PRIO;
	cmd.lead_time = IWL_BT_LEAD_TIME_DEF;
	cmd.max_kill = IWL_BT_MAX_KILL_DEF;
	DPRINTF(sc, IWL_DEBUG_RESET, "%s: configuring bluetooth coexistence\n",
	    __func__);
	return iwl_cmd(sc, IWL_CMD_BT_COEX, &cmd, sizeof(cmd), 0);
}

static int
iwl_send_advanced_btcoex(struct iwl_softc *sc)
{
	static const uint32_t btcoex_3wire[12] = {
		0xaaaaaaaa, 0xaaaaaaaa, 0xaeaaaaaa, 0xaaaaaaaa,
		0xcc00ff28, 0x0000aaaa, 0xcc00aaaa, 0x0000aaaa,
		0xc0004000, 0x00004000, 0xf0005000, 0xf0005000,
	};
	struct iwl6000_btcoex_config btconfig;
	struct iwl_btcoex_priotable btprio;
	struct iwl_btcoex_prot btprot;
	int error, i;

	memset(&btconfig, 0, sizeof btconfig);
	btconfig.flags = 145;
	btconfig.max_kill = 5;
	btconfig.bt3_t7_timer = 1;
	btconfig.kill_ack = htole32(0xffff0000);
	btconfig.kill_cts = htole32(0xffff0000);
	btconfig.sample_time = 2;
	btconfig.bt3_t2_timer = 0xc;
	for (i = 0; i < 12; i++)
		btconfig.lookup_table[i] = htole32(btcoex_3wire[i]);
	btconfig.valid = htole16(0xff);
	btconfig.prio_boost = 0xf0;
	DPRINTF(sc, IWL_DEBUG_RESET,
	    "%s: configuring advanced bluetooth coexistence\n", __func__);
	error = iwl_cmd(sc, IWL_CMD_BT_COEX, &btconfig, sizeof(btconfig), 1);
	if (error != 0)
		return error;

	memset(&btprio, 0, sizeof btprio);
	btprio.calib_init1 = 0x6;
	btprio.calib_init2 = 0x7;
	btprio.calib_periodic_low1 = 0x2;
	btprio.calib_periodic_low2 = 0x3;
	btprio.calib_periodic_high1 = 0x4;
	btprio.calib_periodic_high2 = 0x5;
	btprio.dtim = 0x6;
	btprio.scan52 = 0x8;
	btprio.scan24 = 0xa;
	error = iwl_cmd(sc, IWL_CMD_BT_COEX_PRIOTABLE, &btprio, sizeof(btprio),
	    1);
	if (error != 0)
		return error;

	/* Force BT state machine change. */
	memset(&btprot, 0, sizeof btprio);
	btprot.open = 1;
	btprot.type = 1;
	error = iwl_cmd(sc, IWL_CMD_BT_COEX_PROT, &btprot, sizeof(btprot), 1);
	if (error != 0)
		return error;
	btprot.open = 0;
	return iwl_cmd(sc, IWL_CMD_BT_COEX_PROT, &btprot, sizeof(btprot), 1);
}

static int
iwl_config(struct iwl_softc *sc)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	uint32_t txmask;
	uint16_t rxchain;
	int error;

	if (sc->hw_type == IWL_HW_REV_TYPE_6005) {
		/* Set radio temperature sensor offset. */
		error = iwl5000_temp_offset_calib(sc);
		if (error != 0) {
		printf("could not set temperature offset\n");
			device_printf(sc->sc_dev,
			    "%s: could not set temperature offset\n", __func__);
			return error;
		}
	}

	/* Configure valid TX chains for >=5000 Series. */
	if (sc->hw_type != IWL_HW_REV_TYPE_4965) {
		txmask = htole32(sc->txchainmask);
		DPRINTF(sc, IWL_DEBUG_RESET,
		    "%s: configuring valid TX chains 0x%x\n", __func__, txmask);
		error = iwl_cmd(sc, IWL5000_CMD_TX_ANT_CONFIG, &txmask,
		    sizeof txmask, 0);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not configure valid TX chains, "
			    "error %d\n", __func__, error);
			return error;
		}
	}

	/* Configure bluetooth coexistence. */
	if (sc->sc_flags & IWL_FLAG_ADV_BTCOEX)
		error = iwl_send_advanced_btcoex(sc);
	else
		error = iwl_send_btcoex(sc);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not configure bluetooth coexistence, error %d\n",
		    __func__, error);
		return error;
	}

	sc->rxon = &sc->rx_on[IWL_RXON_BSS_CTX];
	/* Set mode, channel, RX filter and enable RX. */
	memset(sc->rxon, 0, sizeof (struct iwl_rxon));
	IEEE80211_ADDR_COPY(sc->rxon->myaddr, IF_LLADDR(ifp));
	IEEE80211_ADDR_COPY(sc->rxon->wlap, IF_LLADDR(ifp));
	sc->rxon->chan = ieee80211_chan2ieee(ic, ic->ic_curchan);
	sc->rxon->flags = htole32(IWL_RXON_TSF | IWL_RXON_CTS_TO_SELF);
	if (IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan))
		sc->rxon->flags |= htole32(IWL_RXON_AUTO | IWL_RXON_24GHZ);
	switch (ic->ic_opmode) {
	case IEEE80211_M_STA:
		sc->rxon->mode = IWL_MODE_STA;
		sc->rxon->filter = htole32(IWL_FILTER_MULTICAST);
		break;
	case IEEE80211_M_MONITOR:
		sc->rxon->mode = IWL_MODE_MONITOR;
		sc->rxon->filter = htole32(IWL_FILTER_MULTICAST |
		    IWL_FILTER_CTL | IWL_FILTER_PROMISC);
		break;
	default:
		/* Should not get there. */
		break;
	}
	sc->rxon->cck_mask  = 0x0f;	/* not yet negotiated */
	sc->rxon->ofdm_mask = 0xff;	/* not yet negotiated */
	sc->rxon->ht_single_mask = 0xff;
	sc->rxon->ht_dual_mask = 0xff;
	sc->rxon->ht_triple_mask = 0xff;
	rxchain =
	    IWL_RXCHAIN_VALID(sc->rxchainmask) |
	    IWL_RXCHAIN_MIMO_COUNT(2) |
	    IWL_RXCHAIN_IDLE_COUNT(2);
	sc->rxon->rxchain = htole16(rxchain);
	DPRINTF(sc, IWL_DEBUG_RESET, "%s: setting configuration\n", __func__);
	error = iwl_cmd(sc, IWL_CMD_RXON, sc->rxon, sc->rxonsz, 0);
	if (error != 0) {
		device_printf(sc->sc_dev, "%s: RXON command failed\n",
		    __func__);
		return error;
	}

	if ((error = iwl_add_broadcast_node(sc, 0)) != 0) {
		device_printf(sc->sc_dev, "%s: could not add broadcast node\n",
		    __func__);
		return error;
	}

	/* Configuration has changed, set TX power accordingly. */
	if ((error = ops->set_txpower(sc, ic->ic_curchan, 0)) != 0) {
		device_printf(sc->sc_dev, "%s: could not set TX power\n",
		    __func__);
		return error;
	}

	if ((error = iwl_set_critical_temp(sc)) != 0) {
	
	   printf("could not set temperature offset");
		device_printf(sc->sc_dev,
		    "%s: could not set critical temperature\n", __func__);
		return error;
	}

	/*
	 * Any vap's iv_flags are cloned from ic->ic_flags. Setting the flags
	 * on ic will allow a vap to get the correct powersave level once
	 * it is created.
	 */
	if(sc->desired_pwrsave_level!=IWL_POWERSAVE_LVL_NONE)
		ic->ic_flags |= IEEE80211_F_PMGTON;
	else
		ic->ic_flags &= ~IEEE80211_F_PMGTON;

	if ((error = iwl_set_pslevel(sc, IWL_POWERSAVE_DTIM_VOIP_COMPATIBLE,
			sc->desired_pwrsave_level, 0)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set power saving level\n", __func__);
		return error;
	}
	return 0;
}

static int
iwl_config1(struct iwl_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	uint16_t rxchain;
	int error;
	struct ieee80211vap *vap = sc->ivap[IWL_RXON_PAN_CTX];
	struct iwl_vap *ivp = IWL_VAP(vap);

	sc->rxon = &sc->rx_on[IWL_RXON_PAN_CTX];
	IEEE80211_ADDR_COPY(sc->rxon->myaddr, ivp->macaddr);
	IEEE80211_ADDR_COPY(sc->rxon->wlap, IF_LLADDR(ifp));
	sc->rxon->chan = ieee80211_chan2ieee(ic, ic->ic_curchan);
	sc->rxon->flags = htole32(IWL_RXON_TSF);
	if (IEEE80211_IS_CHAN_2GHZ(ic->ic_curchan))
		sc->rxon->flags |= htole32(IWL_RXON_AUTO | IWL_RXON_24GHZ);
	sc->rxon->mode = IWL_MODE_P2P;
	sc->rxon->filter = htole32(IWL_FILTER_MULTICAST);
	sc->rxon->cck_mask  = 0x0f;	/* not yet negotiated */
	sc->rxon->ofdm_mask = 0xff;	/* not yet negotiated */
	sc->rxon->ht_single_mask = 0xff;
	sc->rxon->ht_dual_mask = 0xff;
	sc->rxon->ht_triple_mask = 0xff;
	rxchain =
	    IWL_RXCHAIN_VALID(sc->rxchainmask) |
	    IWL_RXCHAIN_MIMO_COUNT(2) |
	    IWL_RXCHAIN_IDLE_COUNT(2);
	sc->rxon->rxchain = htole16(rxchain);
	sc->rxon->associd = 0;
	sc->rxon->filter &= ~htole32(IWL_FILTER_BSS);

	error = iwl_cmd(sc, IWL_CMD_WIPAN_RXON, sc->rxon, sc->rxonsz, 0);
	if (error != 0) {
		device_printf(sc->sc_dev, "%s: IWL_CMD_WIPAN_RXON command failed\n",
		    __func__);
		return error;
	}

	return 0;
}

static int
iwl_auth(struct iwl_softc *sc, struct ieee80211vap *vap)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_node *ni = vap->iv_bss;
	int error;

	sc->rxon = &sc->rx_on[IWL_RXON_BSS_CTX];
	/* Update adapter configuration. */
	IEEE80211_ADDR_COPY(sc->rxon->bssid, ni->ni_bssid);
	sc->rxon->chan = ieee80211_chan2ieee(ic, ni->ni_chan);
	sc->rxon->flags = htole32(IWL_RXON_TSF | IWL_RXON_CTS_TO_SELF);
	if (IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		sc->rxon->flags |= htole32(IWL_RXON_AUTO | IWL_RXON_24GHZ);
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		sc->rxon->flags |= htole32(IWL_RXON_SHSLOT);
	if (ic->ic_flags & IEEE80211_F_SHPREAMBLE)
		sc->rxon->flags |= htole32(IWL_RXON_SHPREAMBLE);
	if (IEEE80211_IS_CHAN_A(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0;
		sc->rxon->ofdm_mask = 0x15;
	} else if (IEEE80211_IS_CHAN_B(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0x03;
		sc->rxon->ofdm_mask = 0;
	} else {
		/* Assume 802.11b/g. */
		sc->rxon->cck_mask  = 0x0f;
		sc->rxon->ofdm_mask = 0x15;
	}
	DPRINTF(sc, IWL_DEBUG_STATE, "rxon chan %d flags %x cck %x ofdm %x\n",
	    sc->rxon->chan, sc->rxon->flags, sc->rxon->cck_mask,
	    sc->rxon->ofdm_mask);
	error = iwl_cmd(sc, IWL_CMD_RXON, sc->rxon, sc->rxonsz, 0);
	if (error != 0) {
		device_printf(sc->sc_dev, "%s: RXON command failed, error %d\n",
		    __func__, error);
		return error;
	}

	/* Configuration has changed, set TX power accordingly. */
	if ((error = ops->set_txpower(sc, ni->ni_chan, 1)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set TX power, error %d\n", __func__, error);
		return error;
	}
	/*
	 * Reconfiguring RXON clears the firmware nodes table so we must
	 * add the broadcast node again.
	 */
	if ((error = iwl_add_broadcast_node(sc, 0)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not add broadcast node, error %d\n", __func__,
		    error);
		return error;
	}
	return 0;
}

static int
iwl_auth1(struct iwl_softc *sc, struct ieee80211vap *vap)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_node *ni = vap->iv_bss;
	int error;
	struct iwl_vap *ivp = IWL_VAP(vap);

	sc->rxon = &sc->rx_on[IWL_RXON_PAN_CTX];
	IEEE80211_ADDR_COPY(sc->rxon->myaddr, ivp->macaddr);
	IEEE80211_ADDR_COPY(sc->rxon->wlap, IF_LLADDR(ifp));
	/* Update adapter configuration. */
	IEEE80211_ADDR_COPY(sc->rxon->bssid, ni->ni_bssid);
	sc->rxon->chan = ieee80211_chan2ieee(ic, ni->ni_chan);
	sc->rxon->flags = htole32(IWL_RXON_TSF | IWL_RXON_CTS_TO_SELF);
	if (IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		sc->rxon->flags |= htole32(IWL_RXON_AUTO | IWL_RXON_24GHZ);
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		sc->rxon->flags |= htole32(IWL_RXON_SHSLOT);
	if (ic->ic_flags & IEEE80211_F_SHPREAMBLE)
		sc->rxon->flags |= htole32(IWL_RXON_SHPREAMBLE);
	if (IEEE80211_IS_CHAN_A(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0;
		sc->rxon->ofdm_mask = 0x15;
	} else if (IEEE80211_IS_CHAN_B(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0x03;
		sc->rxon->ofdm_mask = 0;
	} else {
		/* Assume 802.11b/g. */
		sc->rxon->cck_mask  = 0x0f;
		sc->rxon->ofdm_mask = 0x15;
	}
	DPRINTF(sc, IWL_DEBUG_STATE, "rxon chan %d flags %x cck %x ofdm %x\n",
	    sc->rxon->chan, sc->rxon->flags, sc->rxon->cck_mask,
	    sc->rxon->ofdm_mask);
	sc->rxon->mode = IWL_MODE_2STA;
	error = iwl_cmd(sc, IWL_CMD_WIPAN_RXON, sc->rxon, sc->rxonsz, 0);
	if (error != 0) {
		device_printf(sc->sc_dev, "%s: RXON command failed, error %d\n",
		    __func__, error);
		return error;
	}

	/* Configuration has changed, set TX power accordingly. */
	if ((error = ops->set_txpower(sc, ni->ni_chan, 1)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set TX power, error %d\n", __func__, error);
		return error;
	}
	/*
	 * Reconfiguring RXON clears the firmware nodes table so we must
	 * add the broadcast node again.
	 */
	if ((error = iwl_add_broadcast_node1(sc, 0)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not add broadcast node, error %d\n", __func__,
		    error);
		return error;
	}

	return 0;
}

static int
iwl_run(struct iwl_softc *sc, struct ieee80211vap *vap)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_node *ni = vap->iv_bss;
	struct iwl_node_info node;
	uint32_t htflags = 0;
	int error;

	sc->rxon = &sc->rx_on[IWL_RXON_BSS_CTX];

	if(sc->ctx == IWL_RXON_PAN_CTX) {		
		if ((error = iwl_set_pan_params(sc)) != 0) {
			device_printf(sc->sc_dev,
		 	   "%s: iwl_set_pan_params error %d\n", __func__, error);
			return error;
		}
	}

	if (ic->ic_opmode == IEEE80211_M_MONITOR)
		return 0;
	if ((error = iwl_set_timing(sc, ni)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set timing, error %d\n", __func__, error);
		return error;
	}

	/* Update adapter configuration. */
	IEEE80211_ADDR_COPY(sc->rxon->bssid, ni->ni_bssid);
	sc->rxon->associd = htole16(IEEE80211_AID(ni->ni_associd));
	sc->rxon->chan = ieee80211_chan2ieee(ic, ni->ni_chan);
	sc->rxon->flags = htole32(IWL_RXON_TSF | IWL_RXON_CTS_TO_SELF);
	if (IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		sc->rxon->flags |= htole32(IWL_RXON_AUTO | IWL_RXON_24GHZ);
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		sc->rxon->flags |= htole32(IWL_RXON_SHSLOT);
	if (ic->ic_flags & IEEE80211_F_SHPREAMBLE)
		sc->rxon->flags |= htole32(IWL_RXON_SHPREAMBLE);
	if (IEEE80211_IS_CHAN_A(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0;
		sc->rxon->ofdm_mask = 0x15;
	} else if (IEEE80211_IS_CHAN_B(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0x03;
		sc->rxon->ofdm_mask = 0;
	} else {
		/* Assume 802.11b/g. */
		sc->rxon->cck_mask  = 0x0f;
		sc->rxon->ofdm_mask = 0x15;
	}
	if (IEEE80211_IS_CHAN_HT(ni->ni_chan)) {
		htflags |= IWL_RXON_HT_PROTMODE(ic->ic_curhtprotmode);
		if (IEEE80211_IS_CHAN_HT40(ni->ni_chan)) {
			switch (ic->ic_curhtprotmode) {
			case IEEE80211_HTINFO_OPMODE_HT20PR:
				htflags |= IWL_RXON_HT_MODEPURE40;
				break;
			default:
				htflags |= IWL_RXON_HT_MODEMIXED;
				break;
			}
		}
		if (IEEE80211_IS_CHAN_HT40D(ni->ni_chan))
			htflags |= IWL_RXON_HT_HT40MINUS;
	}
	sc->rxon->flags |= htole32(htflags);
	sc->rxon->filter |= htole32(IWL_FILTER_BSS);
	DPRINTF(sc, IWL_DEBUG_STATE, "rxon chan %d flags %x\n",
	    sc->rxon->chan, sc->rxon->flags);
	error = iwl_cmd(sc, IWL_CMD_RXON, sc->rxon, sc->rxonsz, 0);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not update configuration, error %d\n", __func__,
		    error);
		return error;
	}

	/* Configuration has changed, set TX power accordingly. */
	if ((error = ops->set_txpower(sc, ni->ni_chan, 1)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set TX power, error %d\n", __func__, error);
		return error;
	}

	/* Fake a join to initialize the TX rate. */
	((struct iwl_node *)ni)->id = IWL_ID_BSS;
	iwl_newassoc(ni, 1);

	/* Add BSS node. */
	memset(&node, 0, sizeof node);
	IEEE80211_ADDR_COPY(node.macaddr, ni->ni_macaddr);
	node.id = IWL_ID_BSS;
	if (IEEE80211_IS_CHAN_HT(ni->ni_chan)) {
		switch (ni->ni_htcap & IEEE80211_HTCAP_SMPS) {
		case IEEE80211_HTCAP_SMPS_ENA:
			node.htflags |= htole32(IWL_SMPS_MIMO_DIS);
			break;
		case IEEE80211_HTCAP_SMPS_DYNAMIC:
			node.htflags |= htole32(IWL_SMPS_MIMO_PROT);
			break;
		}
		node.htflags |= htole32(IWL_AMDPU_SIZE_FACTOR(3) |
		    IWL_AMDPU_DENSITY(5));	/* 4us */
		if (IEEE80211_IS_CHAN_HT40(ni->ni_chan))
			node.htflags |= htole32(IWL_NODE_HT40);
	}
	DPRINTF(sc, IWL_DEBUG_STATE, "%s: adding BSS node\n", __func__);
	error = ops->add_node(sc, &node, 0);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not add BSS node, error %d\n", __func__, error);
		return error;
	}

	/* Setting the initial rate for node */
	ni->ni_txrate = ni->ni_rates.rs_rates[0];

	/* XXX: init rate scaling */

	#ifdef IWL_DTIM_INDICATES_UNICAST_PENDING_AT_AP
		return iwl_set_pslevel(sc, IWL_POWERSAVE_DTIM_VOIP_COMPATIBLE,
			sc->desired_pwrsave_level, 0);
	#else
		return 0;
	#endif
}

static int
iwl_run1(struct iwl_softc *sc, struct ieee80211vap *vap)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211_node *ni = vap->iv_bss;
	struct iwl_node_info node;
	uint32_t htflags = 0;
	int error;
	struct iwl_vap *ivp = IWL_VAP(vap);

	if (ic->ic_opmode == IEEE80211_M_MONITOR) {
		/* Link LED blinks while monitoring. */
		return 0;
	}

	if ((error = iwl_set_timing1(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set timing, error %d\n", __func__, error);
	}

	if ((error = iwl_updateedca1(ic)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: iwl_updateedca1, error %d\n", __func__, error);
		return error;
	}

	sc->rxon = &sc->rx_on[IWL_RXON_PAN_CTX];
	IEEE80211_ADDR_COPY(sc->rxon->myaddr, ivp->macaddr);
	IEEE80211_ADDR_COPY(sc->rxon->wlap, IF_LLADDR(ifp));
	/* Update adapter configuration. */
	IEEE80211_ADDR_COPY(sc->rxon->bssid, ni->ni_bssid);
	sc->rxon->associd = htole16(IEEE80211_AID(ni->ni_associd));
	sc->rxon->chan = ieee80211_chan2ieee(ic, ni->ni_chan);
	sc->rxon->flags = htole32(IWL_RXON_TSF | IWL_RXON_CTS_TO_SELF);
	if (IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		sc->rxon->flags |= htole32(IWL_RXON_AUTO | IWL_RXON_24GHZ);
	if (ic->ic_flags & IEEE80211_F_SHSLOT)
		sc->rxon->flags |= htole32(IWL_RXON_SHSLOT);
	if (ic->ic_flags & IEEE80211_F_SHPREAMBLE)
		sc->rxon->flags |= htole32(IWL_RXON_SHPREAMBLE);
	if (IEEE80211_IS_CHAN_A(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0;
		sc->rxon->ofdm_mask = 0x15;
	} else if (IEEE80211_IS_CHAN_B(ni->ni_chan)) {
		sc->rxon->cck_mask  = 0x03;
		sc->rxon->ofdm_mask = 0;
	} else {
		/* Assume 802.11b/g. */
		sc->rxon->cck_mask  = 0x0f;
		sc->rxon->ofdm_mask = 0x15;
	}
	if (IEEE80211_IS_CHAN_HT(ni->ni_chan)) {
		htflags |= IWL_RXON_HT_PROTMODE(ic->ic_curhtprotmode);
		if (IEEE80211_IS_CHAN_HT40(ni->ni_chan)) {
			switch (ic->ic_curhtprotmode) {
			case IEEE80211_HTINFO_OPMODE_HT20PR:
				htflags |= IWL_RXON_HT_MODEPURE40;
				break;
			default:
				htflags |= IWL_RXON_HT_MODEMIXED;
				break;
			}
		}
		if (IEEE80211_IS_CHAN_HT40D(ni->ni_chan))
			htflags |= IWL_RXON_HT_HT40MINUS;
	}
	sc->rxon->flags |= htole32(htflags);
	sc->rxon->filter |= htole32(IWL_FILTER_BSS);
	DPRINTF(sc, IWL_DEBUG_STATE, "rxon chan %d flags %x\n",
	    sc->rxon->chan, sc->rxon->flags);
	sc->rxon->mode = IWL_MODE_2STA;
	error = iwl_cmd(sc, IWL_CMD_WIPAN_RXON, sc->rxon, sc->rxonsz, 0);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not update configuration, error %d\n", __func__,
		    error);
		return error;
	}

	/* Configuration has changed, set TX power accordingly. */
	if ((error = ops->set_txpower(sc, ni->ni_chan, 1)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not set TX power, error %d\n", __func__, error);
		return error;
	}

	/* Fake a join to initialize the TX rate. */
	((struct iwl_node *)ni)->id = IWL_STA_ID;
	iwl_newassoc(ni, 1);

	/* Add BSS node. */
	memset(&node, 0, sizeof node);
	node.htflags |= htole32(STA_FLAG_PAN_STATION);
	IEEE80211_ADDR_COPY(node.macaddr, ni->ni_macaddr);
	node.id = IWL_STA_ID;
	if (IEEE80211_IS_CHAN_HT(ni->ni_chan)) {
		switch (ni->ni_htcap & IEEE80211_HTCAP_SMPS) {
		case IEEE80211_HTCAP_SMPS_ENA:
			node.htflags |= htole32(IWL_SMPS_MIMO_DIS);
			break;
		case IEEE80211_HTCAP_SMPS_DYNAMIC:
			node.htflags |= htole32(IWL_SMPS_MIMO_PROT);
			break;
		}
		node.htflags |= htole32(IWL_AMDPU_SIZE_FACTOR(3) |
		    IWL_AMDPU_DENSITY(5));	/* 4us */
		if (IEEE80211_IS_CHAN_HT40(ni->ni_chan))
			node.htflags |= htole32(IWL_NODE_HT40);
	}
	DPRINTF(sc, IWL_DEBUG_STATE, "%s: adding BSS node1\n", __func__);
	error = ops->add_node(sc, &node, 0);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not add BSS node1, error %d\n", __func__, error);
		return error;
	}

	/* Setting the initial rate for node */
	ni->ni_txrate = ni->ni_rates.rs_rates[0];

	/* XXX: init rate scaling */

	#ifdef IWL_DTIM_INDICATES_UNICAST_PENDING_AT_AP
		return iwl_set_pslevel(sc, IWL_POWERSAVE_DTIM_VOIP_COMPATIBLE,
			sc->desired_pwrsave_level, 0);
	#else
		return 0;
	#endif

	return 0;
}

/*
 * This function is called by upper layer when an ADDBA request is received
 * from another STA and before the ADDBA response is sent.
 */
static int
iwl_ampdu_rx_start(struct ieee80211_node *ni, struct ieee80211_rx_ampdu *rap,
    int baparamset, int batimeout, int baseqctl)
{
#define MS(_v, _f)	(((_v) & _f) >> _f##_S)
	struct iwl_softc *sc = ni->ni_ic->ic_ifp->if_softc;
	struct iwl_ops *ops = &sc->ops;
	struct iwl_node *wn = (void *)ni;
	struct iwl_node_info node;
	uint16_t ssn;
	uint8_t tid;
	int error;

	tid = MS(le16toh(baparamset), IEEE80211_BAPS_TID);
	ssn = MS(le16toh(baseqctl), IEEE80211_BASEQ_START);

	memset(&node, 0, sizeof node);
	node.id = wn->id;
	node.control = IWL_NODE_UPDATE;
	node.flags = IWL_FLAG_SET_ADDBA;
	node.addba_tid = tid;
	node.addba_ssn = htole16(ssn);
	DPRINTF(sc, IWL_DEBUG_RECV, "ADDBA RA=%d TID=%d SSN=%d\n",
	    wn->id, tid, ssn);
	error = ops->add_node(sc, &node, 1);
	if (error != 0)
		return error;
	return sc->sc_ampdu_rx_start(ni, rap, baparamset, batimeout, baseqctl);
#undef MS
}

/*
 * This function is called by upper layer on teardown of an HT-immediate
 * Block Ack agreement (eg. uppon receipt of a DELBA frame).
 */
static void
iwl_ampdu_rx_stop(struct ieee80211_node *ni, struct ieee80211_rx_ampdu *rap)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct iwl_softc *sc = ic->ic_ifp->if_softc;
	struct iwl_ops *ops = &sc->ops;
	struct iwl_node *wn = (void *)ni;
	struct iwl_node_info node;
	uint8_t tid;

	/* XXX: tid as an argument */
	for (tid = 0; tid < WME_NUM_TID; tid++) {
		if (&ni->ni_rx_ampdu[tid] == rap)
			break;
	}

	memset(&node, 0, sizeof node);
	node.id = wn->id;
	node.control = IWL_NODE_UPDATE;
	node.flags = IWL_FLAG_SET_DELBA;
	node.delba_tid = tid;
	DPRINTF(sc, IWL_DEBUG_RECV, "DELBA RA=%d TID=%d\n", wn->id, tid);
	(void)ops->add_node(sc, &node, 1);
	sc->sc_ampdu_rx_stop(ni, rap);
}

static int
iwl_addba_request(struct ieee80211_node *ni, struct ieee80211_tx_ampdu *tap,
    int dialogtoken, int baparamset, int batimeout)
{
	struct iwl_softc *sc = ni->ni_ic->ic_ifp->if_softc;
	int qid;

	for (qid = sc->firstaggqueue; qid < sc->ntxqs; qid++) {
		if (sc->qid2tap[qid] == NULL)
			break;
	}
	if (qid == sc->ntxqs) {
		DPRINTF(sc, IWL_DEBUG_XMIT, "%s: not free aggregation queue\n",
		    __func__);
		return 0;
	}
	tap->txa_private = malloc(sizeof(int), M_DEVBUF, M_NOWAIT);
	if (tap->txa_private == NULL) {
		device_printf(sc->sc_dev,
		    "%s: failed to alloc TX aggregation structure\n", __func__);
		return 0;
	}
	sc->qid2tap[qid] = tap;
	*(int *)tap->txa_private = qid;
	return sc->sc_addba_request(ni, tap, dialogtoken, baparamset,
	    batimeout);
}

static int
iwl_addba_response(struct ieee80211_node *ni, struct ieee80211_tx_ampdu *tap,
    int code, int baparamset, int batimeout)
{
	struct iwl_softc *sc = ni->ni_ic->ic_ifp->if_softc;
	int qid = *(int *)tap->txa_private;
	uint8_t tid = tap->txa_tid;
	int ret;

	if (code == IEEE80211_STATUS_SUCCESS) {
		ni->ni_txseqs[tid] = tap->txa_start & 0xfff;
		ret = iwl_ampdu_tx_start(ni->ni_ic, ni, tid);
		if (ret != 1)
			return ret;
	} else {
		sc->qid2tap[qid] = NULL;
		free(tap->txa_private, M_DEVBUF);
		tap->txa_private = NULL;
	}
	return sc->sc_addba_response(ni, tap, code, baparamset, batimeout);
}

/*
 * This function is called by upper layer when an ADDBA response is received
 * from another STA.
 */
static int
iwl_ampdu_tx_start(struct ieee80211com *ic, struct ieee80211_node *ni,
    uint8_t tid)
{
	struct ieee80211_tx_ampdu *tap = &ni->ni_tx_ampdu[TID_TO_WME_AC(tid)];
	struct iwl_softc *sc = ni->ni_ic->ic_ifp->if_softc;
	struct iwl_ops *ops = &sc->ops;
	struct iwl_node *wn = (void *)ni;
	struct iwl_node_info node;
	int error, qid;

	/* Enable TX for the specified RA/TID. */
	wn->disable_tid &= ~(1 << tid);
	memset(&node, 0, sizeof node);
	node.id = wn->id;
	node.control = IWL_NODE_UPDATE;
	node.flags = IWL_FLAG_SET_DISABLE_TID;
	node.disable_tid = htole16(wn->disable_tid);
	error = ops->add_node(sc, &node, 1);
	if (error != 0)
		return 0;

	if ((error = iwl_nic_lock(sc)) != 0)
		return 0;
	qid = *(int *)tap->txa_private;
	ops->ampdu_tx_start(sc, ni, qid, tid, tap->txa_start & 0xfff);
	iwl_nic_unlock(sc);

	//XXX: send link quality command ...
	return 1;
}

static void
iwl_ampdu_tx_stop(struct ieee80211_node *ni, struct ieee80211_tx_ampdu *tap)
{
	struct iwl_softc *sc = ni->ni_ic->ic_ifp->if_softc;
	struct iwl_ops *ops = &sc->ops;
	uint8_t tid = tap->txa_tid;
	int qid;

	if (tap->txa_private == NULL)
		return;

	qid = *(int *)tap->txa_private;
	if (iwl_nic_lock(sc) != 0)
		return;
	ops->ampdu_tx_stop(sc, qid, tid, tap->txa_start & 0xfff);
	iwl_nic_unlock(sc);
	sc->qid2tap[qid] = NULL;
	free(tap->txa_private, M_DEVBUF);
	tap->txa_private = NULL;
	sc->sc_addba_stop(ni, tap);
}

static void
iwl5000_ampdu_tx_start(struct iwl_softc *sc, struct ieee80211_node *ni,
    int qid, uint8_t tid, uint16_t ssn)
{
	struct iwl_node *wn = (void *)ni;

	/* Stop TX scheduler while we're changing its configuration. */
	iwl_prph_write(sc, IWL5000_SCHED_QUEUE_STATUS(qid),
	    IWL5000_TXQ_STATUS_CHGACT);

	/* Assign RA/TID translation to the queue. */
	iwl_mem_write_2(sc, sc->sched_base + IWL5000_SCHED_TRANS_TBL(qid),
	    wn->id << 4 | tid);

	/* Enable chain-building mode for the queue. */
	iwl_prph_setbits(sc, IWL5000_SCHED_QCHAIN_SEL, 1 << qid);

	/* Enable aggregation for the queue. */
	iwl_prph_setbits(sc, IWL5000_SCHED_AGGR_SEL, 1 << qid);

	/* Set starting sequence number from the ADDBA request. */
	sc->txq[qid].cur = sc->txq[qid].read = (ssn & 0xff);
	IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, qid << 8 | (ssn & 0xff));
	iwl_prph_write(sc, IWL5000_SCHED_QUEUE_RDPTR(qid), ssn);

	/* Set scheduler window size and frame limit. */
	iwl_mem_write(sc, sc->sched_base + IWL5000_SCHED_QUEUE_OFFSET(qid) + 4,
	    IWL_SCHED_LIMIT << 16 | IWL_SCHED_WINSZ);

	/* Enable interrupts for the queue. */
	iwl_prph_setbits(sc, IWL5000_SCHED_INTR_MASK, 1 << qid);

	/* Mark the queue as active. */
	iwl_prph_write(sc, IWL5000_SCHED_QUEUE_STATUS(qid),
	    IWL5000_TXQ_STATUS_ACTIVE | iwl_tid2fifo[tid]);
}

static void
iwl5000_ampdu_tx_stop(struct iwl_softc *sc, int qid, uint8_t tid, uint16_t ssn)
{
	/* Stop TX scheduler while we're changing its configuration. */
	iwl_prph_write(sc, IWL5000_SCHED_QUEUE_STATUS(qid),
	    IWL5000_TXQ_STATUS_CHGACT);

	/* Disable aggregation for the queue. */
	iwl_prph_clrbits(sc, IWL5000_SCHED_AGGR_SEL, 1 << qid);

	/* Set starting sequence number from the ADDBA request. */
	IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, qid << 8 | (ssn & 0xff));
	iwl_prph_write(sc, IWL5000_SCHED_QUEUE_RDPTR(qid), ssn);

	/* Disable interrupts for the queue. */
	iwl_prph_clrbits(sc, IWL5000_SCHED_INTR_MASK, 1 << qid);

	/* Mark the queue as inactive. */
	iwl_prph_write(sc, IWL5000_SCHED_QUEUE_STATUS(qid),
	    IWL5000_TXQ_STATUS_INACTIVE | iwl_tid2fifo[tid]);
}

/*
 * Query calibration tables from the initialization firmware.  We do this
 * only once at first boot.  Called from a process context.
 */
static int
iwl5000_query_calibration(struct iwl_softc *sc)
{
	struct iwl5000_calib_config cmd;
	int error;

	memset(&cmd, 0, sizeof cmd);
	cmd.ucode.once.enable = 0xffffffff;
	cmd.ucode.once.start  = 0xffffffff;
	cmd.ucode.once.send   = 0xffffffff;
	cmd.ucode.flags       = 0xffffffff;
	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "%s: sending calibration query\n",
	    __func__);
	error = iwl_cmd(sc, IWL5000_CMD_CALIB_CONFIG, &cmd, sizeof cmd, 0);
	if (error != 0)
		return error;

	/* Wait at most two seconds for calibration to complete. */
	if (!(sc->sc_flags & IWL_FLAG_CALIB_DONE))
		error = msleep(sc, &sc->sc_mtx, PCATCH, "iwlcal", 2 * hz);
	return error;
}

/*
 * Send calibration results to the runtime firmware.  These results were
 * obtained on first boot from the initialization firmware.
 */
static int
iwl5000_send_calibration(struct iwl_softc *sc)
{
	int idx, error;

	for (idx = 0; idx < 5; idx++) {
		if (sc->calibcmd[idx].buf == NULL)
			continue;	/* No results available. */
		DPRINTF(sc, IWL_DEBUG_CALIBRATE,
		    "send calibration result idx=%d len=%d\n", idx,
		    sc->calibcmd[idx].len);
		error = iwl_cmd(sc, IWL_CMD_PHY_CALIB, sc->calibcmd[idx].buf,
		    sc->calibcmd[idx].len, 0);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not send calibration result, error %d\n",
			    __func__, error);
			return error;
		}
	}
	return 0;
}

static int
iwl5000_send_wimax_coex(struct iwl_softc *sc)
{
	struct iwl5000_wimax_coex wimax;

#ifdef notyet
	if (sc->hw_type == IWL_HW_REV_TYPE_6050) {
		/* Enable WiMAX coexistence for combo adapters. */
		wimax.flags =
		    IWL_WIMAX_COEX_ASSOC_WA_UNMASK |
		    IWL_WIMAX_COEX_UNASSOC_WA_UNMASK |
		    IWL_WIMAX_COEX_STA_TABLE_VALID |
		    IWL_WIMAX_COEX_ENABLE;
		memcpy(wimax.events, iwl6050_wimax_events,
		    sizeof iwl6050_wimax_events);
	} else
#endif
	{
		/* Disable WiMAX coexistence. */
		wimax.flags = 0;
		memset(wimax.events, 0, sizeof wimax.events);
	}
	DPRINTF(sc, IWL_DEBUG_RESET, "%s: Configuring WiMAX coexistence\n",
	    __func__);
	return iwl_cmd(sc, IWL5000_CMD_WIMAX_COEX, &wimax, sizeof wimax, 0);
}

static int
iwl5000_crystal_calib(struct iwl_softc *sc)
{
	struct iwl5000_phy_calib_crystal cmd;

	memset(&cmd, 0, sizeof cmd);
	cmd.code = IWL5000_PHY_CALIB_CRYSTAL;
	cmd.ngroups = 1;
	cmd.isvalid = 1;
	cmd.cap_pin[0] = le32toh(sc->eeprom_crystal) & 0xff;
	cmd.cap_pin[1] = (le32toh(sc->eeprom_crystal) >> 16) & 0xff;
	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "sending crystal calibration %d, %d\n",
	    cmd.cap_pin[0], cmd.cap_pin[1]);
	return iwl_cmd(sc, IWL_CMD_PHY_CALIB, &cmd, sizeof cmd, 0);
}

static int
iwl5000_temp_offset_calib(struct iwl_softc *sc)
{
	struct iwl5000_phy_calib_temp_offset cmd;

	memset(&cmd, 0, sizeof cmd);
	cmd.code = IWL5000_PHY_CALIB_TEMP_OFFSET;
	cmd.ngroups = 1;
	cmd.isvalid = 1;
	if (sc->eeprom_temp != 0)
		cmd.offset = htole16(sc->eeprom_temp);
	else
		cmd.offset = htole16(IWL_DEFAULT_TEMP_OFFSET);
	DPRINTF(sc, IWL_DEBUG_CALIBRATE, "setting radio sensor offset to %d\n",
	    le16toh(cmd.offset));
	return iwl_cmd(sc, IWL_CMD_PHY_CALIB, &cmd, sizeof cmd, 0);
}

/*
 * This function is called after the initialization or runtime firmware
 * notifies us of its readiness (called in a process context).
 */
static int
iwl5000_post_alive(struct iwl_softc *sc)
{
	int error, qid;

	/* Switch to using ICT interrupt mode. */
	iwl5000_ict_reset(sc);

	if ((error = iwl_nic_lock(sc)) != 0)
		return error;

	/* Clear TX scheduler state in SRAM. */
	sc->sched_base = iwl_prph_read(sc, IWL_SCHED_SRAM_ADDR);
	iwl_mem_set_region_4(sc, sc->sched_base + IWL5000_SCHED_CTX_OFF, 0,
	    IWL5000_SCHED_CTX_LEN / sizeof (uint32_t));

	/* Set physical address of TX scheduler rings (1KB aligned). */
	iwl_prph_write(sc, IWL5000_SCHED_DRAM_ADDR, sc->sched_dma.paddr >> 10);

	IWL_SETBITS(sc, IWL_FH_TX_CHICKEN, IWL_FH_TX_CHICKEN_SCHED_RETRY);

	/* Enable chain mode for all queues, except command queue. */
	if(sc->uc_pan_support == IWL_UC_PAN_PRESENT)
		iwl_prph_write(sc, IWL5000_SCHED_QCHAIN_SEL, 0xffdff);
	else
		iwl_prph_write(sc, IWL5000_SCHED_QCHAIN_SEL, 0xfffef);

	iwl_prph_write(sc, IWL5000_SCHED_AGGR_SEL, 0);

	for (qid = 0; qid < IWL5000_NTXQUEUES; qid++) {
		iwl_prph_write(sc, IWL5000_SCHED_QUEUE_RDPTR(qid), 0);
		IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, qid << 8 | 0);

		iwl_mem_write(sc, sc->sched_base +
		    IWL5000_SCHED_QUEUE_OFFSET(qid), 0);
		/* Set scheduler window size and frame limit. */
		iwl_mem_write(sc, sc->sched_base +
		    IWL5000_SCHED_QUEUE_OFFSET(qid) + 4,
		    IWL_SCHED_LIMIT << 16 | IWL_SCHED_WINSZ);
	}

	/* Enable interrupts for all our 20 queues. */
	iwl_prph_write(sc, IWL5000_SCHED_INTR_MASK, 0xfffff);
	/* Identify TX FIFO rings (0-7). */
	iwl_prph_write(sc, IWL5000_SCHED_TXFACT, 0xff);

	if(sc->uc_pan_support == IWL_UC_PAN_PRESENT) {	
		/* Mark TX rings as active. */
		for (qid = 0; qid < 11; qid++) {
			static uint8_t qid2fifo[] = { 3, 2, 1, 0, 0, 4, 2, 5, 4, 7, 5 };
			iwl_prph_write(sc, IWL5000_SCHED_QUEUE_STATUS(qid),
			    IWL5000_TXQ_STATUS_ACTIVE | qid2fifo[qid]);
		}
	}
	else {
		/* Mark TX rings (4 EDCA + cmd + 2 HCCA) as active. */
		for (qid = 0; qid < 7; qid++) {
			static uint8_t qid2fifo[] = { 3, 2, 1, 0, 7, 5, 6 };
			iwl_prph_write(sc, IWL5000_SCHED_QUEUE_STATUS(qid),
			    IWL5000_TXQ_STATUS_ACTIVE | qid2fifo[qid]);
		}
	}

	iwl_nic_unlock(sc);

	/* Configure WiMAX coexistence for combo adapters. */
	error = iwl5000_send_wimax_coex(sc);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not configure WiMAX coexistence, error %d\n",
		    __func__, error);
		return error;
	}
	if (sc->hw_type != IWL_HW_REV_TYPE_5150) {
		/* Perform crystal calibration. */
		error = iwl5000_crystal_calib(sc);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: crystal calibration failed, error %d\n",
			    __func__, error);
			return error;
		}
	}
	if (!(sc->sc_flags & IWL_FLAG_CALIB_DONE)) {
		/* Query calibration from the initialization firmware. */
		if ((error = iwl5000_query_calibration(sc)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not query calibration, error %d\n",
			    __func__, error);
			return error;
		}
		/*
		 * We have the calibration results now, reboot with the
		 * runtime firmware (call ourselves recursively!)
		 */
		iwl_hw_stop(sc);
		error = iwl_hw_init(sc);
	} else {
		/* Send calibration results to runtime firmware. */
		error = iwl5000_send_calibration(sc);
	}
	return error;
}

static int
iwl5000_load_firmware_section(struct iwl_softc *sc, uint32_t dst,
    const uint8_t *section, int size)
{
	struct iwl_dma_info *dma = &sc->fw_dma;
	int error;

	/* Copy firmware section into pre-allocated DMA-safe memory. */
	memcpy(dma->vaddr, section, size);
	bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);

	if ((error = iwl_nic_lock(sc)) != 0)
		return error;

	IWL_WRITE(sc, IWL_FH_TX_CONFIG(IWL_SRVC_DMACHNL),
	    IWL_FH_TX_CONFIG_DMA_PAUSE);

	IWL_WRITE(sc, IWL_FH_SRAM_ADDR(IWL_SRVC_DMACHNL), dst);
	IWL_WRITE(sc, IWL_FH_TFBD_CTRL0(IWL_SRVC_DMACHNL),
	    IWL_LOADDR(dma->paddr));
	IWL_WRITE(sc, IWL_FH_TFBD_CTRL1(IWL_SRVC_DMACHNL),
	    IWL_HIADDR(dma->paddr) << 28 | size);
	IWL_WRITE(sc, IWL_FH_TXBUF_STATUS(IWL_SRVC_DMACHNL),
	    IWL_FH_TXBUF_STATUS_TBNUM(1) |
	    IWL_FH_TXBUF_STATUS_TBIDX(1) |
	    IWL_FH_TXBUF_STATUS_TFBD_VALID);

	/* Kick Flow Handler to start DMA transfer. */
	IWL_WRITE(sc, IWL_FH_TX_CONFIG(IWL_SRVC_DMACHNL),
	    IWL_FH_TX_CONFIG_DMA_ENA | IWL_FH_TX_CONFIG_CIRQ_HOST_ENDTFD);

	iwl_nic_unlock(sc);

	/* Wait at most five seconds for FH DMA transfer to complete. */
	return msleep(sc, &sc->sc_mtx, PCATCH, "iwlinit", 5 * hz);
}

static int
iwl5000_load_firmware(struct iwl_softc *sc)
{
	struct iwl_fw_part *fw;
	int error;

	/* Load the initialization firmware on first boot only. */
	fw = (sc->sc_flags & IWL_FLAG_CALIB_DONE) ?
	    &sc->fw.main : &sc->fw.init;

	error = iwl5000_load_firmware_section(sc, IWL_FW_TEXT_BASE,
	    fw->text, fw->textsz);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not load firmware %s section, error %d\n",
		    __func__, ".text", error);
		return error;
	}
	error = iwl5000_load_firmware_section(sc, IWL_FW_DATA_BASE,
	    fw->data, fw->datasz);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not load firmware %s section, error %d\n",
		    __func__, ".data", error);
		return error;
	}

	/* Now press "execute". */
	IWL_WRITE(sc, IWL_RESET, 0);
	return 0;
}

/*
 * Extract text and data sections from a legacy firmware image.
 */
static int
iwl_read_firmware_leg(struct iwl_softc *sc, struct iwl_fw_info *fw)
{
	const uint32_t *ptr;
	size_t hdrlen = 24;
	uint32_t rev;

	ptr = (const uint32_t *)fw->data;
	rev = le32toh(*ptr++);

	/* Check firmware API version. */
	if (IWL_FW_API(rev) <= 1) {
		device_printf(sc->sc_dev,
		    "%s: bad firmware, need API version >=2\n", __func__);
		return EINVAL;
	}
	if (IWL_FW_API(rev) >= 3) {
		/* Skip build number (version 2 header). */
		hdrlen += 4;
		ptr++;
	}
	if (fw->size < hdrlen) {
		device_printf(sc->sc_dev, "%s: firmware too short: %zu bytes\n",
		    __func__, fw->size);
		return EINVAL;
	}
	fw->main.textsz = le32toh(*ptr++);
	fw->main.datasz = le32toh(*ptr++);
	fw->init.textsz = le32toh(*ptr++);
	fw->init.datasz = le32toh(*ptr++);
	fw->boot.textsz = le32toh(*ptr++);

	/* Check that all firmware sections fit. */
	if (fw->size < hdrlen + fw->main.textsz + fw->main.datasz +
	    fw->init.textsz + fw->init.datasz + fw->boot.textsz) {
		device_printf(sc->sc_dev, "%s: firmware too short: %zu bytes\n",
		    __func__, fw->size);
		return EINVAL;
	}

	/* Get pointers to firmware sections. */
	fw->main.text = (const uint8_t *)ptr;
	fw->main.data = fw->main.text + fw->main.textsz;
	fw->init.text = fw->main.data + fw->main.datasz;
	fw->init.data = fw->init.text + fw->init.textsz;
	fw->boot.text = fw->init.data + fw->init.datasz;
	return 0;
}

/*
 * Extract text and data sections from a TLV firmware image.
 */
static int
iwl_read_firmware_tlv(struct iwl_softc *sc, struct iwl_fw_info *fw,
    uint16_t alt)
{
	const struct iwl_fw_tlv_hdr *hdr;
	const struct iwl_fw_tlv *tlv;
	const uint8_t *ptr, *end;
	uint64_t altmask;
	uint32_t len, tmp;

	if (fw->size < sizeof (*hdr)) {
		device_printf(sc->sc_dev, "%s: firmware too short: %zu bytes\n",
		    __func__, fw->size);
		return EINVAL;
	}
	hdr = (const struct iwl_fw_tlv_hdr *)fw->data;
	if (hdr->signature != htole32(IWL_FW_SIGNATURE)) {
		device_printf(sc->sc_dev, "%s: bad firmware signature 0x%08x\n",
		    __func__, le32toh(hdr->signature));
		return EINVAL;
	}
	DPRINTF(sc, IWL_DEBUG_RESET, "FW: \"%.64s\", build 0x%x\n", hdr->descr,
	    le32toh(hdr->build));

	/*
	 * Select the closest supported alternative that is less than
	 * or equal to the specified one.
	 */
	altmask = le64toh(hdr->altmask);
	while (alt > 0 && !(altmask & (1ULL << alt)))
		alt--;	/* Downgrade. */
	DPRINTF(sc, IWL_DEBUG_RESET, "using alternative %d\n", alt);

	ptr = (const uint8_t *)(hdr + 1);
	end = (const uint8_t *)(fw->data + fw->size);

	/* Parse type-length-value fields. */
	while (ptr + sizeof (*tlv) <= end) {
		tlv = (const struct iwl_fw_tlv *)ptr;
		len = le32toh(tlv->len);

		ptr += sizeof (*tlv);
		if (ptr + len > end) {
			device_printf(sc->sc_dev,
			    "%s: firmware too short: %zu bytes\n", __func__,
			    fw->size);
			return EINVAL;
		}
		/* Skip other alternatives. */
		if (tlv->alt != 0 && tlv->alt != htole16(alt))
			goto next;

		switch (le16toh(tlv->type)) {
		case IWL_FW_TLV_MAIN_TEXT:
			fw->main.text = ptr;
			fw->main.textsz = len;
			break;
		case IWL_FW_TLV_MAIN_DATA:
			fw->main.data = ptr;
			fw->main.datasz = len;
			break;
		case IWL_FW_TLV_INIT_TEXT:
			fw->init.text = ptr;
			fw->init.textsz = len;
			break;
		case IWL_FW_TLV_INIT_DATA:
			fw->init.data = ptr;
			fw->init.datasz = len;
			break;
		case IWL_FW_TLV_BOOT_TEXT:
			fw->boot.text = ptr;
			fw->boot.textsz = len;
			break;
		case IWL_FW_TLV_ENH_SENS:
			if (!len)
				sc->sc_flags |= IWL_FLAG_ENH_SENS;
			break;
		case IWL_FW_TLV_PHY_CALIB:
			tmp = htole32(*ptr);
			if (tmp < 253) {
				sc->reset_noise_gain = tmp;
				sc->noise_gain = tmp + 1;
			}
			break;
		case IWL_FW_TLV_FLAGS:
			sc->tlv_feature_flags = htole32(*ptr);
			break;
		case IWL_FW_TLV_PAN:
			sc->uc_pan_support = IWL_UC_PAN_PRESENT;
			break;
		default:
			DPRINTF(sc, IWL_DEBUG_RESET,
			    "TLV type %d not handled\n", le16toh(tlv->type));
			break;
		}
 next:		/* TLV fields are 32-bit aligned. */
		ptr += (len + 3) & ~3;
	}
	return 0;
}

static int
iwl_read_firmware(struct iwl_softc *sc)
{
	struct iwl_fw_info *fw = &sc->fw;
	int error;

	IWL_UNLOCK(sc);

	memset(fw, 0, sizeof (*fw));

	/* Read firmware image from filesystem. */
	sc->fw_fp = firmware_get(sc->fwname);
	if (sc->fw_fp == NULL) {
		device_printf(sc->sc_dev, "%s: could not read firmware %s\n",
		    __func__, sc->fwname);
		IWL_LOCK(sc);
		return EINVAL;
	}
	IWL_LOCK(sc);

	fw->size = sc->fw_fp->datasize;
	fw->data = (const uint8_t *)sc->fw_fp->data;
	if (fw->size < sizeof (uint32_t)) {
		device_printf(sc->sc_dev, "%s: firmware too short: %zu bytes\n",
		    __func__, fw->size);
		firmware_put(sc->fw_fp, FIRMWARE_UNLOAD);
		sc->fw_fp = NULL;
		return EINVAL;
	}

	/* Retrieve text and data sections. */
	if (*(const uint32_t *)fw->data != 0)	/* Legacy image. */
		error = iwl_read_firmware_leg(sc, fw);
	else
		error = iwl_read_firmware_tlv(sc, fw, 1);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not read firmware sections, error %d\n",
		    __func__, error);
		firmware_put(sc->fw_fp, FIRMWARE_UNLOAD);
		sc->fw_fp = NULL;
		return error;
	}

	/* Make sure text and data sections fit in hardware memory. */
	if (fw->main.textsz > sc->fw_text_maxsz ||
	    fw->main.datasz > sc->fw_data_maxsz ||
	    fw->init.textsz > sc->fw_text_maxsz ||
	    fw->init.datasz > sc->fw_data_maxsz ||
	    fw->boot.textsz > IWL_FW_BOOT_TEXT_MAXSZ ||
	    (fw->boot.textsz & 3) != 0) {
		device_printf(sc->sc_dev, "%s: firmware sections too large\n",
		    __func__);
		firmware_put(sc->fw_fp, FIRMWARE_UNLOAD);
		sc->fw_fp = NULL;
		return EINVAL;
	}

	/* We can proceed with loading the firmware. */
	return 0;
}

int
iwl_clock_wait(struct iwl_softc *sc)
{
	int ntries;

	/* Set "initialization complete" bit. */
	IWL_SETBITS(sc, IWL_GP_CNTRL, IWL_GP_CNTRL_INIT_DONE);

	/* Wait for clock stabilization. */
	for (ntries = 0; ntries < 2500; ntries++) {
		if (IWL_READ(sc, IWL_GP_CNTRL) & IWL_GP_CNTRL_MAC_CLOCK_READY)
			return 0;
		DELAY(10);
	}
	device_printf(sc->sc_dev,
	    "%s: timeout waiting for clock stabilization\n", __func__);
	return ETIMEDOUT;
}

int
iwl_apm_init(struct iwl_softc *sc)
{
	uint32_t reg;
	int error;

	/* Disable L0s exit timer (NMI bug workaround). */
	IWL_SETBITS(sc, IWL_GIO_CHICKEN, IWL_GIO_CHICKEN_DIS_L0S_TIMER);
	/* Don't wait for ICH L0s (ICH bug workaround). */
	IWL_SETBITS(sc, IWL_GIO_CHICKEN, IWL_GIO_CHICKEN_L1A_NO_L0S_RX);

	/* Set FH wait threshold to max (HW bug under stress workaround). */
	IWL_SETBITS(sc, IWL_DBG_HPET_MEM, 0xffff0000);

	/* Enable HAP INTA to move adapter from L1a to L0s. */
	IWL_SETBITS(sc, IWL_HW_IF_CONFIG, IWL_HW_IF_CONFIG_HAP_WAKE_L1A);

	/* Retrieve PCIe Active State Power Management (ASPM). */
	reg = pci_read_config(sc->sc_dev, sc->sc_cap_off + 0x10, 1);
	/* Workaround for HW instability in PCIe L0->L0s->L1 transition. */
	if (reg & 0x02)	/* L1 Entry enabled. */
		IWL_SETBITS(sc, IWL_GIO, IWL_GIO_L0S_ENA);
	else
		IWL_CLRBITS(sc, IWL_GIO, IWL_GIO_L0S_ENA);

	if (sc->hw_type != IWL_HW_REV_TYPE_4965 &&
	    sc->hw_type <= IWL_HW_REV_TYPE_1000)
		IWL_SETBITS(sc, IWL_ANA_PLL, IWL_ANA_PLL_INIT);

	/* Wait for clock stabilization before accessing prph. */
	if ((error = iwl_clock_wait(sc)) != 0)
		return error;

	if ((error = iwl_nic_lock(sc)) != 0)
		return error;
        {
		/* Enable DMA. */
		iwl_prph_write(sc, IWL_APMG_CLK_EN,
		    IWL_APMG_CLK_CTRL_DMA_CLK_RQT);
	}
	DELAY(20);
	/* Disable L1-Active. */
	iwl_prph_setbits(sc, IWL_APMG_PCI_STT, IWL_APMG_PCI_STT_L1A_DIS);
	iwl_nic_unlock(sc);

	return 0;
}

static void
iwl_apm_stop_master(struct iwl_softc *sc)
{
	int ntries;

	/* Stop busmaster DMA activity. */
	IWL_SETBITS(sc, IWL_RESET, IWL_RESET_STOP_MASTER);
	for (ntries = 0; ntries < 100; ntries++) {
		if (IWL_READ(sc, IWL_RESET) & IWL_RESET_MASTER_DISABLED)
			return;
		DELAY(10);
	}
	device_printf(sc->sc_dev, "%s: timeout waiting for master\n", __func__);
}

void
iwl_apm_stop(struct iwl_softc *sc)
{
	iwl_apm_stop_master(sc);

	/* Reset the entire device. */
	IWL_SETBITS(sc, IWL_RESET, IWL_RESET_SW);
	DELAY(10);
	/* Clear "initialization complete" bit. */
	IWL_CLRBITS(sc, IWL_GP_CNTRL, IWL_GP_CNTRL_INIT_DONE);
}

static int
iwl5000_nic_config(struct iwl_softc *sc)
{
	uint32_t tmp;
	int error;

	if (IWL_RFCFG_TYPE(sc->rfcfg) < 3) {
		IWL_SETBITS(sc, IWL_HW_IF_CONFIG,
		    IWL_RFCFG_TYPE(sc->rfcfg) |
		    IWL_RFCFG_STEP(sc->rfcfg) |
		    IWL_RFCFG_DASH(sc->rfcfg));
	}
	IWL_SETBITS(sc, IWL_HW_IF_CONFIG,
	    IWL_HW_IF_CONFIG_RADIO_SI | IWL_HW_IF_CONFIG_MAC_SI);

	if ((error = iwl_nic_lock(sc)) != 0)
		return error;
	iwl_prph_setbits(sc, IWL_APMG_PS, IWL_APMG_PS_EARLY_PWROFF_DIS);

	if (sc->hw_type == IWL_HW_REV_TYPE_1000) {
		/*
		 * Select first Switching Voltage Regulator (1.32V) to
		 * solve a stability issue related to noisy DC2DC line
		 * in the silicon of 1000 Series.
		 */
		tmp = iwl_prph_read(sc, IWL_APMG_DIGITAL_SVR);
		tmp &= ~IWL_APMG_DIGITAL_SVR_VOLTAGE_MASK;
		tmp |= IWL_APMG_DIGITAL_SVR_VOLTAGE_1_32;
		iwl_prph_write(sc, IWL_APMG_DIGITAL_SVR, tmp);
	}
	iwl_nic_unlock(sc);

	if (sc->sc_flags & IWL_FLAG_INTERNAL_PA) {
		/* Use internal power amplifier only. */
		IWL_WRITE(sc, IWL_GP_DRIVER, IWL_GP_DRIVER_RADIO_2X2_IPA);
	}
	if ((sc->hw_type == IWL_HW_REV_TYPE_6050 ||
	     sc->hw_type == IWL_HW_REV_TYPE_6005) && sc->calib_ver >= 6) {
		/* Indicate that ROM calibration version is >=6. */
		IWL_SETBITS(sc, IWL_GP_DRIVER, IWL_GP_DRIVER_CALIB_VER6);
	}
	if (sc->hw_type == IWL_HW_REV_TYPE_6005)
		IWL_SETBITS(sc, IWL_GP_DRIVER, IWL_GP_DRIVER_6050_1X2);

	/* Hard coding for testing 2230 NIC purpose */
	IWL_SETBITS(sc, IWL_GP_DRIVER, IWL_GP_DRIVER_REG_BIT_RADIO_IQ_INVERT);

	return 0;
}

/*
 * Take NIC ownership over Intel Active Management Technology (AMT).
 */
static int
iwl_hw_prepare(struct iwl_softc *sc)
{
	int ntries;

	/* Check if hardware is ready. */
	IWL_SETBITS(sc, IWL_HW_IF_CONFIG, IWL_HW_IF_CONFIG_NIC_READY);
	for (ntries = 0; ntries < 5; ntries++) {
		if (IWL_READ(sc, IWL_HW_IF_CONFIG) &
		    IWL_HW_IF_CONFIG_NIC_READY)
			return 0;
		DELAY(10);
	}

	/* Hardware not ready, force into ready state. */
	IWL_SETBITS(sc, IWL_HW_IF_CONFIG, IWL_HW_IF_CONFIG_PREPARE);
	for (ntries = 0; ntries < 15000; ntries++) {
		if (!(IWL_READ(sc, IWL_HW_IF_CONFIG) &
		    IWL_HW_IF_CONFIG_PREPARE_DONE))
			break;
		DELAY(10);
	}
	if (ntries == 15000)
		return ETIMEDOUT;

	/* Hardware should be ready now. */
	IWL_SETBITS(sc, IWL_HW_IF_CONFIG, IWL_HW_IF_CONFIG_NIC_READY);
	for (ntries = 0; ntries < 5; ntries++) {
		if (IWL_READ(sc, IWL_HW_IF_CONFIG) &
		    IWL_HW_IF_CONFIG_NIC_READY)
			return 0;
		DELAY(10);
	}
	return ETIMEDOUT;
}

static int
iwl_hw_init(struct iwl_softc *sc)
{
	struct iwl_ops *ops = &sc->ops;
	int error, chnl, qid;

	/* Clear pending interrupts. */
	IWL_WRITE(sc, IWL_INT, 0xffffffff);

	if ((error = iwl_apm_init(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not power ON adapter, error %d\n", __func__,
		    error);
		return error;
	}

	/* Select VMAIN power source. */
	if ((error = iwl_nic_lock(sc)) != 0)
		return error;
	iwl_prph_clrbits(sc, IWL_APMG_PS, IWL_APMG_PS_PWR_SRC_MASK);
	iwl_nic_unlock(sc);

	/* Perform adapter-specific initialization. */
	if ((error = ops->nic_config(sc)) != 0)
		return error;

	/* Initialize RX ring. */
	if ((error = iwl_nic_lock(sc)) != 0)
		return error;
	IWL_WRITE(sc, IWL_FH_RX_CONFIG, 0);
	IWL_WRITE(sc, IWL_FH_RX_WPTR, 0);
	/* Set physical address of RX ring (256-byte aligned). */
	IWL_WRITE(sc, IWL_FH_RX_BASE, sc->rxq.desc_dma.paddr >> 8);
	/* Set physical address of RX status (16-byte aligned). */
	IWL_WRITE(sc, IWL_FH_STATUS_WPTR, sc->rxq.stat_dma.paddr >> 4);
	/* Enable RX. */
	IWL_WRITE(sc, IWL_FH_RX_CONFIG,
	    IWL_FH_RX_CONFIG_ENA           |
	    IWL_FH_RX_CONFIG_IGN_RXF_EMPTY |	/* HW bug workaround */
	    IWL_FH_RX_CONFIG_IRQ_DST_HOST  |
	    IWL_FH_RX_CONFIG_SINGLE_FRAME  |
	    IWL_FH_RX_CONFIG_RB_TIMEOUT(0) |
	    IWL_FH_RX_CONFIG_NRBD(IWL_RX_RING_COUNT_LOG));
	iwl_nic_unlock(sc);
	IWL_WRITE(sc, IWL_FH_RX_WPTR, (IWL_RX_RING_COUNT - 1) & ~7);

	if ((error = iwl_nic_lock(sc)) != 0)
		return error;

	/* Initialize TX scheduler. */
	iwl_prph_write(sc, sc->sched_txfact_addr, 0);

	/* Set physical address of "keep warm" page (16-byte aligned). */
	IWL_WRITE(sc, IWL_FH_KW_ADDR, sc->kw_dma.paddr >> 4);

	/* Initialize TX rings. */
	for (qid = 0; qid < sc->ntxqs; qid++) {
		struct iwl_tx_ring *txq = &sc->txq[qid];

		/* Set physical address of TX ring (256-byte aligned). */
		IWL_WRITE(sc, IWL_FH_CBBC_QUEUE(qid),
		    txq->desc_dma.paddr >> 8);
	}
	iwl_nic_unlock(sc);

	/* Enable DMA channels. */
	for (chnl = 0; chnl < sc->ndmachnls; chnl++) {
		IWL_WRITE(sc, IWL_FH_TX_CONFIG(chnl),
		    IWL_FH_TX_CONFIG_DMA_ENA |
		    IWL_FH_TX_CONFIG_DMA_CREDIT_ENA);
	}

	/* Clear "radio off" and "commands blocked" bits. */
	IWL_WRITE(sc, IWL_UCODE_GP1_CLR, IWL_UCODE_GP1_RFKILL);
	IWL_WRITE(sc, IWL_UCODE_GP1_CLR, IWL_UCODE_GP1_CMD_BLOCKED);

	/* Clear pending interrupts. */
	IWL_WRITE(sc, IWL_INT, 0xffffffff);
	/* Enable interrupt coalescing. */
	IWL_WRITE(sc, IWL_INT_COALESCING, 512 / 8);
	/* Enable interrupts. */
	IWL_WRITE(sc, IWL_INT_MASK, sc->int_mask);

	/* _Really_ make sure "radio off" bit is cleared! */
	IWL_WRITE(sc, IWL_UCODE_GP1_CLR, IWL_UCODE_GP1_RFKILL);
	IWL_WRITE(sc, IWL_UCODE_GP1_CLR, IWL_UCODE_GP1_RFKILL);

	/* Enable shadow registers. */
	if (sc->hw_type >= IWL_HW_REV_TYPE_6000)
		IWL_SETBITS(sc, IWL_SHADOW_REG_CTRL, 0x800fffff);

	if ((error = ops->load_firmware(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not load firmware, error %d\n", __func__,
		    error);
		return error;
	}
	/* Wait at most one second for firmware alive notification. */
	if ((error = msleep(sc, &sc->sc_mtx, PCATCH, "iwlinit", hz)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: timeout waiting for adapter to initialize, error %d\n",
		    __func__, error);
		return error;
	}
	/* Do post-firmware initialization. */
	return ops->post_alive(sc);
}

static void
iwl_hw_stop(struct iwl_softc *sc)
{
	int chnl, qid, ntries;

	IWL_WRITE(sc, IWL_RESET, IWL_RESET_NEVO);

	/* Disable interrupts. */
	IWL_WRITE(sc, IWL_INT_MASK, 0);
	IWL_WRITE(sc, IWL_INT, 0xffffffff);
	IWL_WRITE(sc, IWL_FH_INT, 0xffffffff);
	sc->sc_flags &= ~IWL_FLAG_USE_ICT;

	/* Make sure we no longer hold the NIC lock. */
	iwl_nic_unlock(sc);

	/* Stop TX scheduler. */
	iwl_prph_write(sc, sc->sched_txfact_addr, 0);

	/* Stop all DMA channels. */
	if (iwl_nic_lock(sc) == 0) {
		for (chnl = 0; chnl < sc->ndmachnls; chnl++) {
			IWL_WRITE(sc, IWL_FH_TX_CONFIG(chnl), 0);
			for (ntries = 0; ntries < 200; ntries++) {
				if (IWL_READ(sc, IWL_FH_TX_STATUS) &
				    IWL_FH_TX_STATUS_IDLE(chnl))
					break;
				DELAY(10);
			}
		}
		iwl_nic_unlock(sc);
	}

	/* Stop RX ring. */
	iwl_reset_rx_ring(sc, &sc->rxq);

	/* Reset all TX rings. */
	for (qid = 0; qid < sc->ntxqs; qid++)
		iwl_reset_tx_ring(sc, &sc->txq[qid]);

	if (iwl_nic_lock(sc) == 0) {
		iwl_prph_write(sc, IWL_APMG_CLK_DIS,
		    IWL_APMG_CLK_CTRL_DMA_CLK_RQT);
		iwl_nic_unlock(sc);
	}
	DELAY(5);

	if (sc->sc_led.led_cur_mode != IWL_LED_STATIC_OFF) {
		sc->sc_led.led_cur_mode = IWL_LED_STATIC_OFF;
		iwl_set_led(sc, IWL_LED_LINK, 1, 0, IWL_LED_STATIC_OFF);
	}
	/* Power OFF adapter. */
	iwl_apm_stop(sc);
}

static void
iwl_radio_on(void *arg0, int pending)
{
	struct iwl_softc *sc = arg0;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	if (vap != NULL) {
		iwl_init(sc);
		ieee80211_init(vap);
	}
}

static void
iwl_radio_off(void *arg0, int pending)
{
	struct iwl_softc *sc = arg0;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	iwl_stop(sc);
	if (vap != NULL)
		ieee80211_stop(vap);

	/* Enable interrupts to get RF toggle notification. */
	IWL_LOCK(sc);
	IWL_WRITE(sc, IWL_INT, 0xffffffff);
	IWL_WRITE(sc, IWL_INT_MASK, sc->int_mask);
	IWL_UNLOCK(sc);
}

static void
iwl_init_locked(struct iwl_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;
	int error;

	IWL_LOCK_ASSERT(sc);

	if ((error = iwl_hw_prepare(sc)) != 0) {
		device_printf(sc->sc_dev, "%s: hardware not ready, error %d\n",
		    __func__, error);
		goto fail;
	}

	/* Initialize interrupt mask to default value. */
	sc->int_mask = IWL_INT_MASK_DEF;
	sc->sc_flags &= ~IWL_FLAG_USE_ICT;

	/* Check that the radio is not disabled by hardware switch. */
	if (!(IWL_READ(sc, IWL_GP_CNTRL) & IWL_GP_CNTRL_RFKILL)) {
		device_printf(sc->sc_dev,
		    "radio is disabled by hardware switch\n");
		/* Enable interrupts to get RF toggle notifications. */
		IWL_WRITE(sc, IWL_INT, 0xffffffff);
		IWL_WRITE(sc, IWL_INT_MASK, sc->int_mask);
		return;
	}

	/* Read firmware images from the filesystem. */
	if ((error = iwl_read_firmware(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not read firmware, error %d\n", __func__,
		    error);
		goto fail;
	}

	/* Initialize hardware and upload firmware. */
	error = iwl_hw_init(sc);
	firmware_put(sc->fw_fp, FIRMWARE_UNLOAD);
	sc->fw_fp = NULL;
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not initialize hardware, error %d\n", __func__,
		    error);
		goto fail;
	}

	/* Configure adapter now that it is ready. */
	if ((error = iwl_config(sc)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not configure device, error %d\n", __func__,
		    error);
		goto fail;
	}

	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	ifp->if_drv_flags |= IFF_DRV_RUNNING;

	callout_reset(&sc->watchdog_to, hz, iwl_watchdog, sc);
	return;

fail:	iwl_stop_locked(sc);
}

static void
iwl_init(void *arg)
{
	struct iwl_softc *sc = arg;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;

	IWL_LOCK(sc);
	iwl_init_locked(sc);
	IWL_UNLOCK(sc);

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		ieee80211_start_all(ic);
}

static void
iwl_stop_locked(struct iwl_softc *sc)
{
	struct ifnet *ifp = sc->sc_ifp;

	IWL_LOCK_ASSERT(sc);

	sc->sc_tx_timer = 0;
	sc->sc_scan_timer = 0;
	callout_stop(&sc->watchdog_to);
	callout_stop(&sc->calib_to);
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

	/* Power OFF hardware. */
	iwl_hw_stop(sc);
}

static void
iwl_stop(struct iwl_softc *sc)
{
	IWL_LOCK(sc);
	iwl_stop_locked(sc);
	IWL_UNLOCK(sc);
}

/*
 * Callback from net80211 to force a channel change.
 */
static void
iwl_set_channel(struct ieee80211com *ic)
{
	const struct ieee80211_channel *c = ic->ic_curchan;
	struct ifnet *ifp = ic->ic_ifp;
	struct iwl_softc *sc = ifp->if_softc;

	IWL_LOCK(sc);
	sc->sc_rxtap.wr_chan_freq = htole16(c->ic_freq);
	sc->sc_rxtap.wr_chan_flags = htole16(c->ic_flags);
	sc->sc_txtap.wt_chan_freq = htole16(c->ic_freq);
	sc->sc_txtap.wt_chan_flags = htole16(c->ic_flags);
	IWL_UNLOCK(sc);
}

static void
iwl_hw_reset(void *arg0, int pending)
{
	struct iwl_softc *sc = arg0;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211com *ic = ifp->if_l2com;

	iwl_stop(sc);
	iwl_init(sc);
	ieee80211_notify_radio(ic, 1);
}

int
iwl_set_pan_params(struct iwl_softc *sc)
{
	struct iwl_pan_params_cmd cmd;
	int slot0 = 300, slot1 = 0;
	int bcnint;
	int error = 0;

	/*
	 * If the PAN context is inactive, then we don't need
	 * to update the PAN parameters
	 */
	if (sc->ctx != IWL_RXON_PAN_CTX)
		return 0;

	memset(&cmd, 0, sizeof(cmd));

	/* only 2 slots are currently allowed */
	cmd.num_slots = 2;

	cmd.slots[0].type = IWL_RXON_BSS_CTX; /* BSS */
	cmd.slots[1].type = IWL_RXON_PAN_CTX; /* PAN */

	cmd.flags |= htole16(IWL_PAN_PARAMS_FLG_SLOTTED_MODE);
	bcnint = BEACON_INTERVAL_DEFAULT;
	slot0 = (bcnint >> 1);
	slot1 = (bcnint - slot0);

	if(sc->uc_scan_progress == 1) {
		slot0 = bcnint * 3 - IWL_SLOT_TIME_MIN;
		slot1 = IWL_SLOT_TIME_MIN;
	}
	cmd.slots[0].time = htole16(slot0);
	cmd.slots[1].time = htole16(slot1);

	error = iwl_cmd(sc, IWL_CMD_WIPAN_PARAMS, &cmd, sizeof(cmd), 0);
	if (error != 0) {
		device_printf(sc->sc_dev, "%s: IWL_CMD_WIPAN_PARAMS command failed, error %d\n",
		    __func__, error);
		return error;
	}

	return 0;
}
