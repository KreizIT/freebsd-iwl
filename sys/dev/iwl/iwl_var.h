/*	$FreeBSD$	*/

/*-
 * Copyright (c) 2011 Intel Corporation
 * Copyright (c) 2007, 2008
 *	Damien Bergamini <damien.bergamini@free.fr>
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

#ifndef	__iwl_var_h__
#define	__iwl_var_h__

#include "iwl_tx.h"

#define	TT_UP		1
#define	TT_DOWN		0

enum iwl_rxon_ctx_id {
        IWL_RXON_BSS_CTX,
        IWL_RXON_PAN_CTX,
        IWL_NUM_RXON_CTX
};

#define IWL_UC_PAN_PRESENT		1

/* ADD / MODIFY STATION Command (Op Code 18) -  byte 76-18 -bit13
	STA_FLAG_PAN_STATION bit:
	This bit is set (1) for a station in PAN mode */
#define STA_FLAG_PAN_STATION		(1 << 13)

#define BEACON_INTERVAL_DEFAULT		200
#define IWL_SLOT_TIME_MIN		20

struct iwl_pan_slot {
	uint16_t time;
	uint8_t type;
	uint8_t reserved;
} __packed;

struct iwl_pan_params_cmd {
	uint16_t flags;
#define	IWL_PAN_PARAMS_FLG_SLOTTED_MODE	(1 << 3)

	uint8_t reserved;
	uint8_t num_slots;
	struct iwl_pan_slot slots[10];
} __packed;

#define LINK_QUAL_AC_NUM 4

struct iwl_link_qual_general_params {
	uint8_t flags;

	/* No entries at or above this (driver chosen) index contain MIMO */
	uint8_t mimo_delimiter;

	/* Best single antenna to use for single stream (legacy, SISO). */
	uint8_t single_stream_ant_msk;	/* LINK_QUAL_ANT_* */

	/* Best antennas to use for MIMO (unused for 4965, assumes both). */
	uint8_t dual_stream_ant_msk;	/* LINK_QUAL_ANT_* */

	/*
	 * If driver needs to use different initial rates for different
	 * EDCA QOS access categories (as implemented by tx fifos 0-3),
	 * this table will set that up, by indicating the indexes in the
	 * rs_table[LINK_QUAL_MAX_RETRY_NUM] rate table at which to start.
	 * Otherwise, driver should set all entries to 0.
	 *
	 * Entry usage:
	 * 0 = Background, 1 = Best Effort (normal), 2 = Video, 3 = Voice
	 * TX FIFOs above 3 use same value (typically 0) as TX FIFO 3.
	 */
	uint8_t start_rate_index[LINK_QUAL_AC_NUM];
} __packed;

#define LINK_QUAL_MAX_RETRY_NUM 16

struct iwl_link_qual_agg_params {

	/*
	 * Maximum number of uSec in aggregation.
	 * default set to 4000 (4 milliseconds) if not configured in .cfg
	 */
	uint16_t agg_time_limit;

	/*
	 * Number of Tx retries allowed for a frame, before that frame will
	 * no longer be considered for the start of an aggregation sequence
	 * (scheduler will then try to tx it as single frame).
	 * Driver should set this to 3.
	 */
	uint8_t agg_dis_start_th;

	/*
	 * Maximum number of frames in aggregation.
	 * 0 = no limit (default).  1 = no aggregation.
	 * Other values = max # frames in aggregation.
	 */
	uint8_t agg_frame_cnt_limit;

	uint32_t reserved;
} __packed;

struct iwl_link_quality_cmd {
	/* Index of destination/recipient station in uCode's station table */
	uint8_t sta_id;
	uint8_t reserved1;
	uint16_t control;	/* not used */
	struct iwl_link_qual_general_params general_params;
	struct iwl_link_qual_agg_params agg_params;

	/*
	 * Rate info; when using rate-scaling, Tx command's initial_rate_index
	 * specifies 1st Tx rate attempted, via index into this table.
	 * 4965 devices works its way through table when retrying Tx.
	 */
	struct {
		uint32_t rate_n_flags;	/* RATE_MCS_*, IWL_RATE_* */
	} rs_table[LINK_QUAL_MAX_RETRY_NUM];
	uint32_t reserved2;
} __packed;

struct iwl_led_mode
{
	uint8_t		led_cur_mode;
	uint64_t	led_cur_bt;
	uint64_t	led_last_bt;
	uint64_t	led_cur_tpt;
	uint64_t	led_last_tpt;
	uint64_t	led_bt_diff;
	int		led_cur_time;
	int		led_last_time;
};

struct iwl_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
	uint64_t	wr_tsft;
	uint8_t		wr_flags;
	uint8_t		wr_rate;
	uint16_t	wr_chan_freq;
	uint16_t	wr_chan_flags;
	int8_t		wr_dbm_antsignal;
	int8_t		wr_dbm_antnoise;
} __packed;

#define IWL_RX_RADIOTAP_PRESENT						\
	((1 << IEEE80211_RADIOTAP_TSFT) |				\
	 (1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL) |				\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |			\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTNOISE))

struct iwl_softc;

struct iwl_rx_data {
	struct mbuf	*m;
	bus_dmamap_t	map;
};

struct iwl_rx_ring {
	struct iwl_dma_info	desc_dma;
	struct iwl_dma_info	stat_dma;
	uint32_t		*desc;
	struct iwl_rx_status	*stat;
	struct iwl_rx_data	data[IWL_RX_RING_COUNT];
	bus_dma_tag_t		data_dmat;
	int			cur;
};

	

struct iwl_calib_state {
	uint8_t		state;
#define IWL_CALIB_STATE_INIT	0
#define IWL_CALIB_STATE_ASSOC	1
#define IWL_CALIB_STATE_RUN	2

	u_int		nbeacons;
	uint32_t	noise[3];
	uint32_t	rssi[3];
	uint32_t	ofdm_x1;
	uint32_t	ofdm_mrc_x1;
	uint32_t	ofdm_x4;
	uint32_t	ofdm_mrc_x4;
	uint32_t	cck_x4;
	uint32_t	cck_mrc_x4;
	uint32_t	bad_plcp_ofdm;
	uint32_t	fa_ofdm;
	uint32_t	bad_plcp_cck;
	uint32_t	fa_cck;
	uint32_t	low_fa;
	uint8_t		cck_state;
#define IWL_CCK_STATE_INIT	0
#define IWL_CCK_STATE_LOFA	1
#define IWL_CCK_STATE_HIFA	2

	uint8_t		noise_samples[20];
	u_int		cur_noise_sample;
	uint8_t		noise_ref;
	uint32_t	energy_samples[10];
	u_int		cur_energy_sample;
	uint32_t	energy_cck;
};

struct iwl_calib_info {
	uint8_t		*buf;
	u_int		len;
};

struct iwl_fw_part {
	const uint8_t	*text;
	uint32_t	textsz;
	const uint8_t	*data;
	uint32_t	datasz;
};

struct iwl_fw_info {
	const uint8_t		*data;
	size_t			size;
	struct iwl_fw_part	init;
	struct iwl_fw_part	main;
	struct iwl_fw_part	boot;
};

struct iwl_node {
	struct ieee80211_node           ni;     /* must be the first */
	uint16_t                        disable_tid;
	uint8_t                         id;
	uint32_t                        ridx[256];
	struct {
		uint64_t                bitmap;
		int                     startidx;
		int                     nframes;
	} agg[IEEE80211_TID_SIZE];
};

struct iwl_ops {
	int		(*load_firmware)(struct iwl_softc *);
	void		(*read_eeprom)(struct iwl_softc *);
	int		(*post_alive)(struct iwl_softc *);
	int		(*nic_config)(struct iwl_softc *);
	void		(*update_sched)(struct iwl_softc *, int, int, uint8_t,
			    uint16_t);
	void		(*get_temperature)(struct iwl_softc *);
	int		(*get_rssi)(struct iwl_softc *, struct iwl_rx_stat *);
	int		(*set_txpower)(struct iwl_softc *,
			    struct ieee80211_channel *, int);
	int		(*init_gains)(struct iwl_softc *);
	int		(*set_gains)(struct iwl_softc *);
	int		(*add_node)(struct iwl_softc *, struct iwl_node_info *,
			    int);
	void		(*tx_done)(struct iwl_softc *, struct iwl_rx_desc *,
			    struct iwl_rx_data *);
	void		(*ampdu_tx_start)(struct iwl_softc *,
			    struct ieee80211_node *, int, uint8_t, uint16_t);
	void		(*ampdu_tx_stop)(struct iwl_softc *, int, uint8_t,
			    uint16_t);
};

struct iwl_vap {
	struct ieee80211vap	iv_vap;
	uint8_t			iv_ridx;

	int			(*iv_newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);
	int 			ctx;
	int			beacon_int;
	uint8_t			macaddr[IEEE80211_ADDR_LEN];
};
#define	IWL_VAP(_vap)	((struct iwl_vap *)(_vap))

struct iwl_softc {
	device_t		sc_dev;

	struct ifnet		*sc_ifp;
	int			sc_debug;

	struct mtx		sc_mtx;

	u_int			sc_flags;
#define IWL_FLAG_HAS_OTPROM	(1 << 1)
#define IWL_FLAG_CALIB_DONE	(1 << 2)
#define IWL_FLAG_USE_ICT	(1 << 3)
#define IWL_FLAG_INTERNAL_PA	(1 << 4)
#define IWL_FLAG_HAS_11N	(1 << 6)
#define IWL_FLAG_ENH_SENS	(1 << 7)
#define IWL_FLAG_ADV_BTCOEX	(1 << 8)

	uint8_t 		hw_type;

	struct iwl_ops		ops;
	const char		*fwname;
	const struct iwl_sensitivity_limits
				*limits;
	int			ntxqs;
	int			firstaggqueue;
	int			ndmachnls;
	uint8_t			broadcast_id;
	int			rxonsz;
	int			schedsz;
	uint32_t		fw_text_maxsz;
	uint32_t		fw_data_maxsz;
	uint32_t		fwsz;
	bus_size_t		sched_txfact_addr;
	uint32_t		reset_noise_gain;
	uint32_t		noise_gain;

	/* TX scheduler rings. */
	struct iwl_dma_info	sched_dma;
	uint16_t		*sched;
	uint32_t		sched_base;

	/* "Keep Warm" page. */
	struct iwl_dma_info	kw_dma;

	/* Firmware image. */
	const struct firmware	*fw_fp;

	/* Firmware DMA transfer. */
	struct iwl_dma_info	fw_dma;

	/* ICT table. */
	struct iwl_dma_info	ict_dma;
	uint32_t		*ict;
	int			ict_cur;

	/* TX/RX rings. */
	struct iwl_tx_ring	txq[IWL5000_NTXQUEUES];
	struct iwl_rx_ring	rxq;

	int			mem_rid;
	struct resource		*mem;
	bus_space_tag_t		sc_st;
	bus_space_handle_t	sc_sh;
	int			irq_rid;
	struct resource		*irq;
	void 			*sc_ih;
	bus_size_t		sc_sz;
	int			sc_cap_off;	/* PCIe Capabilities. */

	/* Tasks used by the driver */
	struct task		sc_reinit_task;
	struct task		sc_radioon_task;
	struct task		sc_radiooff_task;

	struct callout		calib_to;
	int			calib_cnt;
	struct iwl_calib_state	calib;
	struct callout		watchdog_to;
	struct callout		ct_kill_exit_to;
	struct iwl_fw_info	fw;
	struct iwl_calib_info	calibcmd[5];
	uint32_t		errptr;

	struct iwl_rx_stat	last_rx_stat;
	int			last_rx_valid;
	struct iwl_ucode_info	ucode_info;
	struct iwl_rxon 	rx_on[IWL_NUM_RXON_CTX];
	struct iwl_rxon		*rxon;
	int			ctx;
	struct ieee80211vap	*ivap[IWL_NUM_RXON_CTX];
	uint8_t			uc_pan_support;
	uint8_t			uc_scan_progress;

	uint32_t		rawtemp;
	int			temp;
	int			noise;
	uint32_t		qfullmsk;
	uint32_t		prom_base;
	struct iwl_eeprom_chan	eeprom_channels[IWL_NBANDS][IWL_MAX_CHAN_PER_BAND];
	uint16_t		rfcfg;
	uint8_t			calib_ver;
	char			eeprom_domain[4];
	uint32_t		eeprom_crystal;
	int16_t			eeprom_temp;
	int16_t			eeprom_temp_high;
	int16_t			eeprom_voltage;
	int8_t			maxpwr2GHz;
	int8_t			maxpwr5GHz;
	int8_t			maxpwr[IEEE80211_CHAN_MAX];

	uint32_t		tlv_feature_flags;

	int32_t			temp_off;
	uint32_t		int_mask;
	uint8_t			ntxchains;
	uint8_t			nrxchains;
	uint8_t			txchainmask;
	uint8_t			rxchainmask;
	uint8_t			chainmask;

	int			sc_tx_timer;
	int			sc_scan_timer;

	struct ieee80211_tx_ampdu *qid2tap[IWL5000_NTXQUEUES];

	int			(*sc_ampdu_rx_start)(struct ieee80211_node *,
				    struct ieee80211_rx_ampdu *, int, int, int);
	void			(*sc_ampdu_rx_stop)(struct ieee80211_node *,
				    struct ieee80211_rx_ampdu *);
	int			(*sc_addba_request)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *, int, int, int);
	int			(*sc_addba_response)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *, int, int, int);
	void			(*sc_addba_stop)(struct ieee80211_node *,
				    struct ieee80211_tx_ampdu *);

	struct	iwl_led_mode sc_led;

	struct	iwl_rx_radiotap_header sc_rxtap;
	struct	iwl_tx_radiotap_header sc_txtap;

	/* The power save level originally configured by user */
	int			desired_pwrsave_level;

	/* The current power save level, this may differ from the configured value due to
	 * thermal throttling etc.
	 * */
	int			current_pwrsave_level;

};

#define IWL_LOCK_INIT(_sc) \
	mtx_init(&(_sc)->sc_mtx, device_get_nameunit((_sc)->sc_dev), \
	    MTX_NETWORK_LOCK, MTX_DEF)
#define IWL_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define IWL_LOCK_ASSERT(_sc)	mtx_assert(&(_sc)->sc_mtx, MA_OWNED)
#define IWL_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define IWL_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->sc_mtx)

#define IWL_POWERSAVE_LVL_NONE 			0
#define IWL_POWERSAVE_LVL_VOIP_COMPATIBLE 	1
#define IWL_POWERSAVE_LVL_MAX 			5

/*
 * By default we enable power saving. An alternative is to by default use best
 * speed/latency mode (no power saving), which would be in line with the
 * previous driver implementation. The default level can be changed easily from
 * the line below.
 *
 * Note that once the interface wlan0 etc. is created, the user/system can
 * change the mode using ifconfig/ioctl.
 */
#define IWL_POWERSAVE_LVL_DEFAULT	IWL_POWERSAVE_LVL_VOIP_COMPATIBLE

/* DTIM value to pass in for IWL_POWERSAVE_LVL_VOIP_COMPATIBLE */
#define IWL_POWERSAVE_DTIM_VOIP_COMPATIBLE 2

/*
 * If IWL_DTIM_INDICATES_UNICAST_PENDING_AT_AP is defined, then power saving
 * (including the power saving done for unicast traffic) becomes proportional
 * to the DTIM period received from the AP. Otherwise the constant DTIM
 * period IWL_POWERSAVE_DTIM_VOIP_COMPATIBLE is used.
 *
 * Per the 802.11 spec DTIM value as applicable to power saving seems to be
 * relevant only for indicating the frequency at which broadcast/multicast
 * data is sent to the PS STAs.
 * However in practice some APs may also send the unicast traffic along with
 * the DTIM.
 */
#define IWL_DTIM_INDICATES_UNICAST_PENDING_AT_AP
#define IWL_DESC(x) case x: return #x
#define COUNTOF( array ) ( sizeof( array )/sizeof( array[0] ) )

#endif /* __iwl_var_h__ */
