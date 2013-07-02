/*	$FreeBSD$	*/

/*-
 * Copyright (c) 2011 Intel Corporation
 * Copyright (c) 2007, 2008
 *	Damien Bergamini <damien.bergamini@free.fr>
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

#include "iwl_csr.h"
#include "iwl_dma.h"

#define IWL_RX_RING_COUNT_LOG	6
#define IWL_RX_RING_COUNT	(1 << IWL_RX_RING_COUNT_LOG)

#define IWL4965_NDMACHNLS	7
#define IWL5000_NDMACHNLS	8

#define IWL_SRVC_DMACHNL	9

#define IWL_ICT_SIZE		4096
#define IWL_ICT_COUNT		(IWL_ICT_SIZE / sizeof (uint32_t))

/* RX buffers must be large enough to hold a full 4K A-MPDU. */
#define IWL_RBUF_SIZE		(4 * 1024)

#if defined(__LP64__)
/* HW supports 36-bit DMA addresses. */
#define IWL_LOADDR(paddr)	((uint32_t)(paddr))
#define IWL_HIADDR(paddr)	(((paddr) >> 32) & 0xf)
#else
#define IWL_LOADDR(paddr)	(paddr)
#define IWL_HIADDR(paddr)	(0)
#endif

/*
 * NIC internal memory offsets.
 */
#define IWL_APMG_CLK_CTRL		0x3000
#define IWL_APMG_CLK_EN			0x3004
#define IWL_APMG_CLK_DIS		0x3008
#define IWL_APMG_PS			0x300c
#define IWL_APMG_DIGITAL_SVR		0x3058
#define IWL_APMG_ANALOG_SVR		0x306c
#define IWL_APMG_PCI_STT		0x3010
#define IWL_BSM_WR_CTRL			0x3400
#define IWL_BSM_WR_MEM_SRC		0x3404
#define IWL_BSM_WR_MEM_DST		0x3408
#define IWL_BSM_WR_DWCOUNT		0x340c
#define IWL_BSM_DRAM_TEXT_ADDR		0x3490
#define IWL_BSM_DRAM_TEXT_SIZE		0x3494
#define IWL_BSM_DRAM_DATA_ADDR		0x3498
#define IWL_BSM_DRAM_DATA_SIZE		0x349c
#define IWL_BSM_SRAM_BASE		0x3800

/* Possible flags for register IWL_BSM_WR_CTRL. */
#define IWL_BSM_WR_CTRL_START_EN	(1 << 30)
#define IWL_BSM_WR_CTRL_START		(1 << 31)

/* Possible flags for registers IWL_APMG_CLK_*. */
#define IWL_APMG_CLK_CTRL_DMA_CLK_RQT	(1 <<  9)
#define IWL_APMG_CLK_CTRL_BSM_CLK_RQT	(1 << 11)

/* Possible flags for register IWL_APMG_PS. */
#define IWL_APMG_PS_EARLY_PWROFF_DIS	(1 << 22)
#define IWL_APMG_PS_PWR_SRC(x)		((x) << 24)
#define IWL_APMG_PS_PWR_SRC_VMAIN	0
#define IWL_APMG_PS_PWR_SRC_VAUX	2
#define IWL_APMG_PS_PWR_SRC_MASK	IWL_APMG_PS_PWR_SRC(3)
#define IWL_APMG_PS_RESET_REQ		(1 << 26)

/* Possible flags for register IWL_APMG_DIGITAL_SVR. */
#define IWL_APMG_DIGITAL_SVR_VOLTAGE(x)	(((x) & 0xf) << 5)
#define IWL_APMG_DIGITAL_SVR_VOLTAGE_MASK	\
	IWL_APMG_DIGITAL_SVR_VOLTAGE(0xf)
#define IWL_APMG_DIGITAL_SVR_VOLTAGE_1_32	\
	IWL_APMG_DIGITAL_SVR_VOLTAGE(3)

/* Possible flags for IWL_APMG_PCI_STT. */
#define IWL_APMG_PCI_STT_L1A_DIS	(1 << 11)

/* Possible flags for register IWL_BSM_DRAM_TEXT_SIZE. */
#define IWL_FW_UPDATED			(1 << 31)

typedef signed char s8;

struct iwl_rx_status {
	uint16_t	closed_count;
	uint16_t	closed_rx_count;
	uint16_t	finished_count;
	uint16_t	finished_rx_count;
	uint32_t	reserved[2];
} __packed;

struct iwl_rx_desc {
	uint32_t	len;
	uint8_t		type;
#define IWL_UC_READY			1
#define IWL_ADD_NODE_DONE		24
#define IWL_TX_DONE			28
#define IWL5000_CALIBRATION_RESULT	102
#define IWL5000_CALIBRATION_DONE	103
#define IWL_START_SCAN			130
#define IWL_STOP_SCAN			132
#define IWL_RX_STATISTICS		156
#define IWL_BEACON_STATISTICS		157
#define IWL_STATE_CHANGED		161
#define IWL_BEACON_MISSED		162
#define IWL_RX_PHY			192
#define IWL_MPDU_RX_DONE		193
#define IWL_RX_DONE			195
#define IWL_RX_COMPRESSED_BA		197

	uint8_t		flags;
	uint8_t		idx;
	uint8_t		qid;
} __packed;

/* Possible RX status flags. */
#define IWL_RX_NO_CRC_ERR	(1 <<  0)
#define IWL_RX_NO_OVFL_ERR	(1 <<  1)
/* Shortcut for the above. */
#define IWL_RX_NOERROR	(IWL_RX_NO_CRC_ERR | IWL_RX_NO_OVFL_ERR)
#define IWL_RX_MPDU_MIC_OK	(1 <<  6)
#define IWL_RX_CIPHER_MASK	(7 <<  8)
#define IWL_RX_CIPHER_CCMP	(2 <<  8)
#define IWL_RX_MPDU_DEC		(1 << 11)
#define IWL_RX_DECRYPT_MASK	(3 << 11)
#define IWL_RX_DECRYPT_OK	(3 << 11)

/* Antenna flags, used in various commands. */
#define IWL_ANT_A	(1 << 0)
#define IWL_ANT_B	(1 << 1)
#define IWL_ANT_C	(1 << 2)
/* Shortcuts. */
#define IWL_ANT_AB	(IWL_ANT_A | IWL_ANT_B)
#define IWL_ANT_BC	(IWL_ANT_B | IWL_ANT_C)
#define IWL_ANT_ABC	(IWL_ANT_A | IWL_ANT_B | IWL_ANT_C)

/* Structure for command IWL_CMD_RXON. */
struct iwl_rxon {
	uint8_t		myaddr[IEEE80211_ADDR_LEN];
	uint16_t	reserved1;
	uint8_t		bssid[IEEE80211_ADDR_LEN];
	uint16_t	reserved2;
	uint8_t		wlap[IEEE80211_ADDR_LEN];
	uint16_t	reserved3;
	uint8_t		mode;
#define IWL_MODE_HOSTAP		1
#define IWL_MODE_STA		3
#define IWL_MODE_IBSS		4
#define IWL_MODE_MONITOR	6
#define IWL_MODE_2STA		8
#define IWL_MODE_P2P		9

	uint8_t		air;
	uint16_t	rxchain;
#define IWL_RXCHAIN_DRIVER_FORCE	(1 << 0)
#define IWL_RXCHAIN_VALID(x)		(((x) & IWL_ANT_ABC) << 1)
#define IWL_RXCHAIN_FORCE_SEL(x)	(((x) & IWL_ANT_ABC) << 4)
#define IWL_RXCHAIN_FORCE_MIMO_SEL(x)	(((x) & IWL_ANT_ABC) << 7)
#define IWL_RXCHAIN_IDLE_COUNT(x)	((x) << 10)
#define IWL_RXCHAIN_MIMO_COUNT(x)	((x) << 12)
#define IWL_RXCHAIN_MIMO_FORCE		(1 << 14)

	uint8_t		ofdm_mask;
	uint8_t		cck_mask;
	uint16_t	associd;
	uint32_t	flags;
#define IWL_RXON_24GHZ		(1 <<  0)
#define IWL_RXON_CCK		(1 <<  1)
#define IWL_RXON_AUTO		(1 <<  2)
#define IWL_RXON_SHSLOT		(1 <<  4)
#define IWL_RXON_SHPREAMBLE	(1 <<  5)
#define IWL_RXON_NODIVERSITY	(1 <<  7)
#define IWL_RXON_ANTENNA_A	(1 <<  8)
#define IWL_RXON_ANTENNA_B	(1 <<  9)
#define IWL_RXON_TSF		(1 << 15)
#define IWL_RXON_HT_HT40MINUS	(1 << 22)
#define IWL_RXON_HT_PROTMODE(x)	(x << 23)
#define IWL_RXON_HT_MODEPURE40	(1 << 25)
#define IWL_RXON_HT_MODEMIXED	(2 << 25)
#define IWL_RXON_CTS_TO_SELF	(1 << 30)

	uint32_t	filter;
#define IWL_FILTER_PROMISC	(1 << 0)
#define IWL_FILTER_CTL		(1 << 1)
#define IWL_FILTER_MULTICAST	(1 << 2)
#define IWL_FILTER_NODECRYPT	(1 << 3)
#define IWL_FILTER_BSS		(1 << 5)
#define IWL_FILTER_BEACON	(1 << 6)

	uint8_t		chan;
	uint8_t		reserved4;
	uint8_t		ht_single_mask;
	uint8_t		ht_dual_mask;
	/* The following fields are for >=5000 Series only. */
	uint8_t		ht_triple_mask;
	uint8_t		reserved5;
	uint16_t	acquisition;
	uint16_t	reserved6;
} __packed;

#define IWL4965_RXONSZ	(sizeof (struct iwl_rxon) - 6)
#define IWL5000_RXONSZ	(sizeof (struct iwl_rxon))

/* Structure for command IWL_CMD_ASSOCIATE. */
struct iwl_assoc {
	uint32_t	flags;
	uint32_t	filter;
	uint8_t		ofdm_mask;
	uint8_t		cck_mask;
	uint16_t	reserved;
} __packed;

/* Structure for command IWL_CMD_EDCA_PARAMS. */
struct iwl_edca_params {
	uint32_t	flags;
#define IWL_EDCA_UPDATE	(1 << 0)
#define IWL_EDCA_TXOP	(1 << 4)

	struct {
		uint16_t	cwmin;
		uint16_t	cwmax;
		uint8_t		aifsn;
		uint8_t		reserved;
		uint16_t	txoplimit;
	} __packed	ac[WME_NUM_AC];
} __packed;

/* Structure for command IWL_CMD_TIMING. */
struct iwl_cmd_timing {
	uint64_t	tstamp;
	uint16_t	bintval;
	uint16_t	atim;
	uint32_t	binitval;
	uint16_t	lintval;
	uint8_t		dtim_period;
	uint8_t		reserved;
} __packed;

/* Structure for command IWL_CMD_ADD_NODE. */
struct iwl_node_info {
	uint8_t		control;
#define IWL_NODE_UPDATE			(1 << 0)

	uint8_t		reserved1[3];

	uint8_t		macaddr[IEEE80211_ADDR_LEN];
	uint16_t	reserved2;
	uint8_t		id;
#define IWL_ID_BSS			0
#define IWL_STA_ID			1

#define	IWL_PAN_BCAST_ID		14
#define	IWL_BROADCAST_ID		15
#define IWL5000_ID_BROADCAST		15
#define IWL4965_ID_BROADCAST		31

	uint8_t		flags;
#define IWL_FLAG_SET_KEY		(1 << 0)
#define IWL_FLAG_SET_DISABLE_TID	(1 << 1)
#define IWL_FLAG_SET_TXRATE		(1 << 2)
#define IWL_FLAG_SET_ADDBA		(1 << 3)
#define IWL_FLAG_SET_DELBA		(1 << 4)

	uint16_t	reserved3;
	uint16_t	kflags;
#define IWL_KFLAG_CCMP			(1 <<  1)
#define IWL_KFLAG_MAP			(1 <<  3)
#define IWL_KFLAG_KID(kid)		((kid) << 8)
#define IWL_KFLAG_INVALID		(1 << 11)
#define IWL_KFLAG_GROUP			(1 << 14)

	uint8_t		tsc2;	/* TKIP TSC2 */
	uint8_t		reserved4;
	uint16_t	ttak[5];
	uint8_t		kid;
	uint8_t		reserved5;
	uint8_t		key[16];
	/* The following 3 fields are for 5000 Series only. */
	uint64_t	tsc;
	uint8_t		rxmic[8];
	uint8_t		txmic[8];

	uint32_t	htflags;
#define IWL_SMPS_MIMO_PROT		(1 << 17)
#define IWL_AMDPU_SIZE_FACTOR(x)	((x) << 19)
#define IWL_NODE_HT40			(1 << 21)
#define IWL_SMPS_MIMO_DIS		(1 << 22)
#define IWL_AMDPU_DENSITY(x)		((x) << 23)

	uint32_t	mask;
	uint16_t	disable_tid;
	uint16_t	reserved6;
	uint8_t		addba_tid;
	uint8_t		delba_tid;
	uint16_t	addba_ssn;
	uint32_t	reserved7;
} __packed;

struct iwl4965_node_info {
	uint8_t		control;
	uint8_t		reserved1[3];
	uint8_t		macaddr[IEEE80211_ADDR_LEN];
	uint16_t	reserved2;
	uint8_t		id;
	uint8_t		flags;
	uint16_t	reserved3;
	uint16_t	kflags;
	uint8_t		tsc2;	/* TKIP TSC2 */
	uint8_t		reserved4;
	uint16_t	ttak[5];
	uint8_t		kid;
	uint8_t		reserved5;
	uint8_t		key[16];
	uint32_t	htflags;
	uint32_t	mask;
	uint16_t	disable_tid;
	uint16_t	reserved6;
	uint8_t		addba_tid;
	uint8_t		delba_tid;
	uint16_t	addba_ssn;
	uint32_t	reserved7;
} __packed;

#define IWL_RFLAG_MCS		(1 << 8)
#define IWL_RFLAG_CCK		(1 << 9)
#define IWL_RFLAG_GREENFIELD	(1 << 10)
#define IWL_RFLAG_HT40		(1 << 11)
#define IWL_RFLAG_DUPLICATE	(1 << 12)
#define IWL_RFLAG_SGI		(1 << 13)
#define IWL_RFLAG_ANT(x)	((x) << 14)

/* Structure for command IWL_CMD_LINK_QUALITY. */
#define IWL_MAX_TX_RETRIES	16
struct iwl_cmd_link_quality {
	uint8_t		id;
	uint8_t		reserved1;
	uint16_t	ctl;
	uint8_t		flags;
	uint8_t		mimo;
	uint8_t		antmsk_1stream;
	uint8_t		antmsk_2stream;
	uint8_t		ridx[WME_NUM_AC];
	uint16_t	ampdu_limit;
	uint8_t		ampdu_threshold;
	uint8_t		ampdu_max;
	uint32_t	reserved2;
	uint32_t	retry[IWL_MAX_TX_RETRIES];
	uint32_t	reserved3;
} __packed;

/* Structure for command IWL_CMD_SET_LED. */
struct iwl_cmd_led {
	uint32_t	unit;	/* multiplier (in usecs) */
	uint8_t		which;
#define IWL_LED_ACTIVITY	1
#define IWL_LED_LINK		2

	uint8_t		off;
	uint8_t		on;
	uint8_t		reserved;
} __packed;

/* Structure for command IWL5000_CMD_WIMAX_COEX. */
struct iwl5000_wimax_coex {
	uint32_t	flags;
#define IWL_WIMAX_COEX_STA_TABLE_VALID		(1 << 0)
#define IWL_WIMAX_COEX_UNASSOC_WA_UNMASK	(1 << 2)
#define IWL_WIMAX_COEX_ASSOC_WA_UNMASK		(1 << 3)
#define IWL_WIMAX_COEX_ENABLE			(1 << 7)

	struct iwl5000_wimax_event {
		uint8_t	request;
		uint8_t	window;
		uint8_t	reserved;
		uint8_t	flags;
	} __packed	events[16];
} __packed;

/* Structures for command IWL5000_CMD_CALIB_CONFIG. */
struct iwl5000_calib_elem {
	uint32_t	enable;
	uint32_t	start;
	uint32_t	send;
	uint32_t	apply;
	uint32_t	reserved;
} __packed;

struct iwl5000_calib_status {
	struct iwl5000_calib_elem	once;
	struct iwl5000_calib_elem	perd;
	uint32_t			flags;
} __packed;

struct iwl5000_calib_config {
	struct iwl5000_calib_status	ucode;
	struct iwl5000_calib_status	driver;
	uint32_t			reserved;
} __packed;

/* Structure for command IWL_CMD_SET_POWER_MODE. */
struct iwl_pmgt_cmd {
	uint16_t	flags;
#define IWL_PS_ALLOW_SLEEP	(1 << 0)
#define IWL_PS_NOTIFY		(1 << 1)
#define IWL_PS_SLEEP_OVER_DTIM	(1 << 2)
#define IWL_PS_PCI_PMGT		(1 << 3)
#define IWL_PS_FAST_PD		(1 << 4)
#define IWL_PS_BEACON_FILTERING	(1 << 5)
#define IWL_PS_SHADOW_REG	(1 << 6)
#define IWL_PS_CT_KILL		(1 << 7)
#define IWL_PS_BT_SCD		(1 << 8)
#define IWL_PS_ADVANCED_PM	(1 << 9)

	uint8_t		keepalive;
	uint8_t		debug;
	uint32_t	rxtimeout;
	uint32_t	txtimeout;
	uint32_t	intval[5];
	uint32_t	beacons;
} __packed;

/* Structure for command IWL_CMD_TXPOWER (4965AGN only.) */
#define IWL_RIDX_MAX	32
struct iwl4965_cmd_txpower {
	uint8_t		band;
	uint8_t		reserved1;
	uint8_t		chan;
	uint8_t		reserved2;
	struct {
		uint8_t	rf_gain[2];
		uint8_t	dsp_gain[2];
	} __packed	power[IWL_RIDX_MAX + 1];
} __packed;

/* Structure for command IWL_CMD_TXPOWER_DBM (5000 Series only.) */
struct iwl5000_cmd_txpower {
	int8_t	global_limit;	/* in half-dBm */
#define IWL5000_TXPOWER_AUTO		0x7f
#define IWL5000_TXPOWER_MAX_DBM		16

	uint8_t	flags;
#define IWL5000_TXPOWER_NO_CLOSED	(1 << 6)

	int8_t	srv_limit;	/* in half-dBm */
	uint8_t	reserved;
} __packed;

/* Structures for command IWL_CMD_BLUETOOTH. */
struct iwl_bluetooth {
	uint8_t		flags;
#define IWL_BT_COEX_CHAN_ANN	(1 << 0)
#define IWL_BT_COEX_BT_PRIO	(1 << 1)
#define IWL_BT_COEX_2_WIRE	(1 << 2)

	uint8_t		lead_time;
#define IWL_BT_LEAD_TIME_DEF	30

	uint8_t		max_kill;
#define IWL_BT_MAX_KILL_DEF	5

	uint8_t		reserved;
	uint32_t	kill_ack;
	uint32_t	kill_cts;
} __packed;

struct iwl6000_btcoex_config {
	uint8_t		flags;
	uint8_t		lead_time;
	uint8_t		max_kill;
	uint8_t		bt3_t7_timer;
	uint32_t	kill_ack;
	uint32_t	kill_cts;
	uint8_t		sample_time;
	uint8_t		bt3_t2_timer;
	uint16_t	bt4_reaction;
	uint32_t	lookup_table[12];
	uint16_t	bt4_decision;
	uint16_t	valid;
	uint8_t		prio_boost;
	uint8_t		tx_prio_boost;
	uint16_t	rx_prio_boost;
} __packed;

struct iwl_btcoex_priotable {
	uint8_t		calib_init1;
	uint8_t		calib_init2;
	uint8_t		calib_periodic_low1;
	uint8_t		calib_periodic_low2;
	uint8_t		calib_periodic_high1;
	uint8_t		calib_periodic_high2;
	uint8_t		dtim;
	uint8_t		scan52;
	uint8_t		scan24;
	uint8_t		reserved[7];
} __packed;

struct iwl_btcoex_prot {
	uint8_t		open;
	uint8_t		type;
	uint8_t		reserved[2];
} __packed;

/* Structure for command IWL_CMD_SET_CRITICAL_TEMP. */
struct iwl_critical_temp {

	uint32_t	tempM;
	uint32_t	reserved;
	uint32_t	tempR;
/* degK <-> degC conversion macros. */
#define IWL_CTOK(c)	((c) + 273)
#define IWL_KTOC(k)	((k) - 273)
#define IWL_CTOMUK(c)	(((c) * 1000000) + 273150000)
} __packed;

/* Structures for command IWL_CMD_SET_SENSITIVITY. */
struct iwl_sensitivity_cmd {
	uint16_t	which;
#define IWL_SENSITIVITY_DEFAULTTBL	0
#define IWL_SENSITIVITY_WORKTBL		1

	uint16_t	energy_cck;
	uint16_t	energy_ofdm;
	uint16_t	corr_ofdm_x1;
	uint16_t	corr_ofdm_mrc_x1;
	uint16_t	corr_cck_mrc_x4;
	uint16_t	corr_ofdm_x4;
	uint16_t	corr_ofdm_mrc_x4;
	uint16_t	corr_barker;
	uint16_t	corr_barker_mrc;
	uint16_t	corr_cck_x4;
	uint16_t	energy_ofdm_th;
} __packed;

struct iwl_enhanced_sensitivity_cmd {
	uint16_t	which;
	uint16_t	energy_cck;
	uint16_t	energy_ofdm;
	uint16_t	corr_ofdm_x1;
	uint16_t	corr_ofdm_mrc_x1;
	uint16_t	corr_cck_mrc_x4;
	uint16_t	corr_ofdm_x4;
	uint16_t	corr_ofdm_mrc_x4;
	uint16_t	corr_barker;
	uint16_t	corr_barker_mrc;
	uint16_t	corr_cck_x4;
	uint16_t	energy_ofdm_th;
	/* "Enhanced" part. */
	uint16_t	ina_det_ofdm;
	uint16_t	ina_det_cck;
	uint16_t	corr_11_9_en;
	uint16_t	ofdm_det_slope_mrc;
	uint16_t	ofdm_det_icept_mrc;
	uint16_t	ofdm_det_slope;
	uint16_t	ofdm_det_icept;
	uint16_t	cck_det_slope_mrc;
	uint16_t	cck_det_icept_mrc;
	uint16_t	cck_det_slope;
	uint16_t	cck_det_icept;
	uint16_t	reserved;
} __packed;

/* Structures for command IWL_CMD_PHY_CALIB. */
struct iwl_phy_calib {
	uint8_t	code;
#define IWL4965_PHY_CALIB_DIFF_GAIN		7
#define IWL5000_PHY_CALIB_DC			8
#define IWL5000_PHY_CALIB_LO			9
#define IWL5000_PHY_CALIB_TX_IQ			11
#define IWL5000_PHY_CALIB_CRYSTAL		15
#define IWL5000_PHY_CALIB_BASE_BAND		16
#define IWL5000_PHY_CALIB_TX_IQ_PERIODIC	17
#define IWL5000_PHY_CALIB_TEMP_OFFSET		18

#define IWL5000_PHY_CALIB_RESET_NOISE_GAIN	18
#define IWL5000_PHY_CALIB_NOISE_GAIN		19

	uint8_t	group;
	uint8_t	ngroups;
	uint8_t	isvalid;
} __packed;

struct iwl5000_phy_calib_crystal {
	uint8_t	code;
	uint8_t	group;
	uint8_t	ngroups;
	uint8_t	isvalid;

	uint8_t	cap_pin[2];
	uint8_t	reserved[2];
} __packed;

struct iwl5000_phy_calib_temp_offset {
	uint8_t		code;
	uint8_t		group;
	uint8_t		ngroups;
	uint8_t		isvalid;
	int16_t		offset;
#define IWL_DEFAULT_TEMP_OFFSET	2700

	uint16_t	reserved;
} __packed;

struct iwl_phy_calib_gain {
	uint8_t	code;
	uint8_t	group;
	uint8_t	ngroups;
	uint8_t	isvalid;

	int8_t	gain[3];
	uint8_t	reserved;
} __packed;

/* Structure for command IWL_CMD_SPECTRUM_MEASUREMENT. */
struct iwl_spectrum_cmd {
	uint16_t	len;
	uint8_t		token;
	uint8_t		id;
	uint8_t		origin;
	uint8_t		periodic;
	uint16_t	timeout;
	uint32_t	start;
	uint32_t	reserved1;
	uint32_t	flags;
	uint32_t	filter;
	uint16_t	nchan;
	uint16_t	reserved2;
	struct {
		uint32_t	duration;
		uint8_t		chan;
		uint8_t		type;
#define IWL_MEASUREMENT_BASIC		(1 << 0)
#define IWL_MEASUREMENT_CCA		(1 << 1)
#define IWL_MEASUREMENT_RPI_HISTOGRAM	(1 << 2)
#define IWL_MEASUREMENT_NOISE_HISTOGRAM	(1 << 3)
#define IWL_MEASUREMENT_FRAME		(1 << 4)
#define IWL_MEASUREMENT_IDLE		(1 << 7)

		uint16_t	reserved;
	} __packed	chan[10];
} __packed;

/* Structure for IWL_UC_READY notification. */
#define IWL_NATTEN_GROUPS	5
struct iwl_ucode_info {
	uint8_t		minor;
	uint8_t		major;
	uint16_t	reserved1;
	uint8_t		revision[8];
	uint8_t		type;
	uint8_t		subtype;
#define IWL_UCODE_RUNTIME	0
#define IWL_UCODE_INIT		9

	uint16_t	reserved2;
	uint32_t	logptr;
	uint32_t	errptr;
	uint32_t	tstamp;
	uint32_t	valid;

	/* The following fields are for UCODE_INIT only. */
	int32_t		volt;
	struct {
		int32_t	chan20MHz;
		int32_t	chan40MHz;
	} __packed	temp[4];
	int32_t		atten[IWL_NATTEN_GROUPS][2];
} __packed;


struct iwl4965_tx_stat {
	uint8_t		nframes;
	uint8_t		btkillcnt;
	uint8_t		rtsfailcnt;
	uint8_t		ackfailcnt;
	uint32_t	rate;
	uint16_t	duration;
	uint16_t	reserved;
	uint32_t	power[2];
	uint32_t	status;
} __packed;

/* Structure for IWL_BEACON_MISSED notification. */
struct iwl_beacon_missed {
	uint32_t	consecutive;
	uint32_t	total;
	uint32_t	expected;
	uint32_t	received;
} __packed;

/* Structure for IWL_MPDU_RX_DONE notification. */
struct iwl_rx_mpdu {
	uint16_t	len;
	uint16_t	reserved;
} __packed;

/* Structures for IWL_RX_DONE and IWL_MPDU_RX_DONE notifications. */
struct iwl4965_rx_phystat {
	uint16_t	antenna;
	uint16_t	agc;
	uint8_t		rssi[6];
} __packed;

struct iwl5000_rx_phystat {
	uint32_t	reserved1;
	uint32_t	agc;
	uint16_t	rssi[3];
} __packed;

struct iwl_rx_stat {
	uint8_t		phy_len;
	uint8_t		cfg_phy_len;
#define IWL_STAT_MAXLEN	20

	uint8_t		id;
	uint8_t		reserved1;
	uint64_t	tstamp;
	uint32_t	beacon;
	uint16_t	flags;
#define IWL_STAT_FLAG_SHPREAMBLE	(1 << 2)

	uint16_t	chan;
	uint8_t		phybuf[32];
	uint32_t	rate;
	uint16_t	len;
	uint16_t	reserve3;
} __packed;

#define IWL_RSSI_TO_DBM	44

/* Structure for IWL_RX_COMPRESSED_BA notification. */
struct iwl_compressed_ba {
	uint8_t		macaddr[IEEE80211_ADDR_LEN];
	uint16_t	reserved;
	uint8_t		id;
	uint8_t		tid;
	uint16_t	seq;
	uint64_t	bitmap;
	uint16_t	qid;
	uint16_t	ssn;
} __packed;

/* Structure for IWL_SPECTRUM_MEASUREMENT notification. */
struct iwl_spectrum_notif {
	uint8_t		id;
	uint8_t		token;
	uint8_t		idx;
	uint8_t		state;
#define IWL_MEASUREMENT_START	0
#define IWL_MEASUREMENT_STOP	1

	uint32_t	start;
	uint8_t		band;
	uint8_t		chan;
	uint8_t		type;
	uint8_t		reserved1;
	uint32_t	cca_ofdm;
	uint32_t	cca_cck;
	uint32_t	cca_time;
	uint8_t		basic;
	uint8_t		reserved2[3];
	uint32_t	ofdm[8];
	uint32_t	cck[8];
	uint32_t	stop;
	uint32_t	status;
#define IWL_MEASUREMENT_OK		0
#define IWL_MEASUREMENT_CONCURRENT	1
#define IWL_MEASUREMENT_CSA_CONFLICT	2
#define IWL_MEASUREMENT_TGH_CONFLICT	3
#define IWL_MEASUREMENT_STOPPED		6
#define IWL_MEASUREMENT_TIMEOUT		7
#define IWL_MEASUREMENT_FAILED		8
} __packed;

/* Structures for IWL_{RX,BEACON}_STATISTICS notification. */
struct iwl_rx_phy_stats {
	uint32_t	ina;
	uint32_t	fina;
	uint32_t	bad_plcp;
	uint32_t	bad_crc32;
	uint32_t	overrun;
	uint32_t	eoverrun;
	uint32_t	good_crc32;
	uint32_t	fa;
	uint32_t	bad_fina_sync;
	uint32_t	sfd_timeout;
	uint32_t	fina_timeout;
	uint32_t	no_rts_ack;
	uint32_t	rxe_limit;
	uint32_t	ack;
	uint32_t	cts;
	uint32_t	ba_resp;
	uint32_t	dsp_kill;
	uint32_t	bad_mh;
	uint32_t	rssi_sum;
	uint32_t	reserved;
} __packed;

struct iwl_rx_general_stats {
	uint32_t	bad_cts;
	uint32_t	bad_ack;
	uint32_t	not_bss;
	uint32_t	filtered;
	uint32_t	bad_chan;
	uint32_t	beacons;
	uint32_t	missed_beacons;
	uint32_t	adc_saturated;	/* time in 0.8us */
	uint32_t	ina_searched;	/* time in 0.8us */
	uint32_t	noise[3];
	uint32_t	flags;
	uint32_t	load;
	uint32_t	fa;
	uint32_t	rssi[3];
	uint32_t	energy[3];
} __packed;

struct iwl_rx_ht_phy_stats {
	uint32_t	bad_plcp;
	uint32_t	overrun;
	uint32_t	eoverrun;
	uint32_t	good_crc32;
	uint32_t	bad_crc32;
	uint32_t	bad_mh;
	uint32_t	good_ampdu_crc32;
	uint32_t	ampdu;
	uint32_t	fragment;
	uint32_t	reserved;
} __packed;

struct iwl_rx_stats {
	struct iwl_rx_phy_stats		ofdm;
	struct iwl_rx_phy_stats		cck;
	struct iwl_rx_general_stats	general;
	struct iwl_rx_ht_phy_stats	ht;
} __packed;

struct iwl_tx_stats {
	uint32_t	preamble;
	uint32_t	rx_detected;
	uint32_t	bt_defer;
	uint32_t	bt_kill;
	uint32_t	short_len;
	uint32_t	cts_timeout;
	uint32_t	ack_timeout;
	uint32_t	exp_ack;
	uint32_t	ack;
	uint32_t	msdu;
	uint32_t	busrt_err1;
	uint32_t	burst_err2;
	uint32_t	cts_collision;
	uint32_t	ack_collision;
	uint32_t	ba_timeout;
	uint32_t	ba_resched;
	uint32_t	query_ampdu;
	uint32_t	query;
	uint32_t	query_ampdu_frag;
	uint32_t	query_mismatch;
	uint32_t	not_ready;
	uint32_t	underrun;
	uint32_t	bt_ht_kill;
	uint32_t	rx_ba_resp;
	uint32_t	reserved[2];
} __packed;

struct iwl_general_stats {
	uint32_t	temp;
	uint32_t	temp_m;
	uint32_t	burst_check;
	uint32_t	burst;
	uint32_t	reserved1[4];
	uint32_t	sleep;
	uint32_t	slot_out;
	uint32_t	slot_idle;
	uint32_t	ttl_tstamp;
	uint32_t	tx_ant_a;
	uint32_t	tx_ant_b;
	uint32_t	exec;
	uint32_t	probe;
	uint32_t	reserved2[2];
	uint32_t	rx_enabled;
	uint32_t	reserved3[3];
} __packed;

struct iwl_stats {
	uint32_t			flags;
	struct iwl_rx_stats		rx;
	struct iwl_tx_stats		tx;
	struct iwl_general_stats	general;
} __packed;


/* Firmware error dump. */
struct iwl_fw_dump {
	uint32_t	valid;
	uint32_t	id;
	uint32_t	pc;
	uint32_t	branch_link[2];
	uint32_t	interrupt_link[2];
	uint32_t	error_data[2];
	uint32_t	src_line;
	uint32_t	tsf;
	uint32_t	time[2];
} __packed;

/* TLV firmware header. */
struct iwl_fw_tlv_hdr {
	uint32_t	zero;	/* Always 0, to differentiate from legacy. */
	uint32_t	signature;
#define IWL_FW_SIGNATURE	0x0a4c5749	/* "IWL\n" */

	uint8_t		descr[64];
	uint32_t	rev;
#define IWL_FW_API(x)		(((x) >> 8) & 0xff)

	uint32_t	build;
	uint64_t	altmask;
} __packed;

/* TLV header. */
struct iwl_fw_tlv {
	uint16_t	type;
#define IWL_FW_TLV_MAIN_TEXT		1
#define IWL_FW_TLV_MAIN_DATA		2
#define IWL_FW_TLV_INIT_TEXT		3
#define IWL_FW_TLV_INIT_DATA		4
#define IWL_FW_TLV_BOOT_TEXT		5
#define IWL_FW_TLV_PBREQ_MAXLEN		6
#define IWL_FW_TLV_PAN			7
#define IWL_FW_TLV_ENH_SENS		14
#define IWL_FW_TLV_PHY_CALIB		15
#define IWL_FW_TLV_FLAGS		18

	uint16_t	alt;
	uint32_t	len;
} __packed;

#define IWL_FW_TLV_FLAGS_NEW_SCAN_BITPOS 1

#define IWL4965_FW_TEXT_MAXSZ		( 96 * 1024)
#define IWL4965_FW_DATA_MAXSZ		( 40 * 1024)
#define IWL5000_FW_TEXT_MAXSZ		(256 * 1024)
#define IWL5000_FW_DATA_MAXSZ		( 80 * 1024)
#define IWL_FW_BOOT_TEXT_MAXSZ		1024
#define IWL4965_FWSZ			(IWL4965_FW_TEXT_MAXSZ + IWL4965_FW_DATA_MAXSZ)
#define IWL5000_FWSZ			IWL5000_FW_TEXT_MAXSZ

/*
 * Offsets into EEPROM.
 */
#define IWL_EEPROM_MAC		0x015
#define IWL_EEPROM_SKU_CAP	0x045
#define IWL_EEPROM_RFCFG	0x048
#define IWL_EEPROM_MAC_PAN	0x049

#define IWL4965_EEPROM_DOMAIN		0x060
#define IWL4965_EEPROM_BAND1		0x063
#define IWL5000_EEPROM_REG		0x066
#define IWL5000_EEPROM_CAL		0x067
#define IWL4965_EEPROM_BAND2		0x072
#define IWL4965_EEPROM_BAND3		0x080
#define IWL4965_EEPROM_BAND4		0x08d
#define IWL4965_EEPROM_BAND5		0x099
#define IWL4965_EEPROM_BAND6		0x0a0
#define IWL4965_EEPROM_BAND7		0x0a8
#define IWL4965_EEPROM_MAXPOW		0x0e8
#define IWL4965_EEPROM_VOLTAGE		0x0e9
#define IWL4965_EEPROM_BANDS		0x0ea
/* Indirect offsets. */
#define IWL5000_EEPROM_DOMAIN		0x001
#define IWL5000_EEPROM_BAND1		0x004
#define IWL5000_EEPROM_BAND2		0x013
#define IWL5000_EEPROM_BAND3		0x021
#define IWL5000_EEPROM_BAND4		0x02e
#define IWL5000_EEPROM_BAND5		0x03a
#define IWL5000_EEPROM_BAND6		0x041
#define IWL6000_EEPROM_BAND6		0x040
#define IWL5000_EEPROM_BAND7		0x049
#define IWL6000_EEPROM_ENHINFO		0x054
#define IWL5000_EEPROM_CRYSTAL		0x128
#define IWL5000_EEPROM_TEMP		0x12a
#define IWL5000_EEPROM_VOLT		0x12b

/* Possible flags for IWL_EEPROM_SKU_CAP. */
#define IWL_EEPROM_SKU_CAP_11N	(1 << 6)
#define IWL_EEPROM_SKU_CAP_AMT	(1 << 7)
#define IWL_EEPROM_SKU_CAP_IPAN	(1 << 8)

/* Possible flags for IWL_EEPROM_RFCFG. */
#define IWL_RFCFG_TYPE(x)	(((x) >>  0) & 0x3)
#define IWL_RFCFG_STEP(x)	(((x) >>  2) & 0x3)
#define IWL_RFCFG_DASH(x)	(((x) >>  4) & 0x3)
#define IWL_RFCFG_TXANTMSK(x)	(((x) >>  8) & 0xf)
#define IWL_RFCFG_RXANTMSK(x)	(((x) >> 12) & 0xf)

struct iwl_eeprom_chan {
	uint8_t	flags;
#define IWL_EEPROM_CHAN_VALID	(1 << 0)
#define IWL_EEPROM_CHAN_IBSS	(1 << 1)
#define IWL_EEPROM_CHAN_ACTIVE	(1 << 3)
#define IWL_EEPROM_CHAN_RADAR	(1 << 4)

	int8_t	maxpwr;
} __packed;

struct iwl_eeprom_enhinfo {
	uint8_t		flags;
#define IWL_ENHINFO_VALID	0x01
#define IWL_ENHINFO_5GHZ	0x02
#define IWL_ENHINFO_OFDM	0x04
#define IWL_ENHINFO_HT40	0x08
#define IWL_ENHINFO_HTAP	0x10
#define IWL_ENHINFO_RES1	0x20
#define IWL_ENHINFO_RES2	0x40
#define IWL_ENHINFO_COMMON	0x80

	uint8_t		chan;
	int8_t		chain[3];	/* max power in half-dBm */
	uint8_t		reserved;
	int8_t		mimo2;		/* max power in half-dBm */
	int8_t		mimo3;		/* max power in half-dBm */
} __packed;

struct iwl5000_eeprom_calib_hdr {
	uint8_t		version;
	uint8_t		pa_type;
	uint16_t	volt;
} __packed;

#define IWL_NSAMPLES	3

#define IWL_NBANDS	8
/*
 * Offsets of channels descriptions in EEPROM.
 */
static const uint32_t iwl5000_regulatory_bands[IWL_NBANDS] = {
	IWL5000_EEPROM_BAND1,
	IWL5000_EEPROM_BAND2,
	IWL5000_EEPROM_BAND3,
	IWL5000_EEPROM_BAND4,
	IWL5000_EEPROM_BAND5,
	IWL5000_EEPROM_BAND6,
	IWL5000_EEPROM_BAND7
};

static const uint32_t iwl6000_regulatory_bands[IWL_NBANDS] = {
	IWL5000_EEPROM_BAND1,
	IWL5000_EEPROM_BAND2,
	IWL5000_EEPROM_BAND3,
	IWL5000_EEPROM_BAND4,
	IWL5000_EEPROM_BAND5,
	IWL6000_EEPROM_BAND6,
	IWL5000_EEPROM_BAND7
};

#define IWL_CHAN_BANDS_COUNT	 7
#define IWL_MAX_CHAN_PER_BAND	14
static const struct iwl_chan_band {
	uint8_t	nchan;
	uint8_t	chan[IWL_MAX_CHAN_PER_BAND];
} iwl_bands[] = {
	/* 20MHz channels, 2GHz band. */
	{ 14, { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 } },
	/* 20MHz channels, 5GHz band. */
	{ 13, { 183, 184, 185, 187, 188, 189, 192, 196, 7, 8, 11, 12, 16 } },
	{ 12, { 34, 36, 38, 40, 42, 44, 46, 48, 52, 56, 60, 64 } },
	{ 11, { 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140 } },
	{  6, { 145, 149, 153, 157, 161, 165 } },
	/* 40MHz channels (primary channels), 2GHz band. */
	{  7, { 1, 2, 3, 4, 5, 6, 7 } },
	/* 40MHz channels (primary channels), 5GHz band. */
	{ 11, { 36, 44, 52, 60, 100, 108, 116, 124, 132, 149, 157 } }
};

#define IWL1000_OTP_NBLOCKS	3
#define IWL6000_OTP_NBLOCKS	4
#define IWL6050_OTP_NBLOCKS	7

/* HW rate indices. */
#define IWL_RIDX_CCK1		0
#define IWL_RIDX_OFDM6		4

#define IWL4965_MAX_PWR_INDEX	107

/*
 * RF Tx gain values from highest to lowest power (values obtained from
 * the reference driver.)
 */
static const uint8_t iwl4965_rf_gain_2ghz[IWL4965_MAX_PWR_INDEX + 1] = {
	0x3f, 0x3f, 0x3f, 0x3e, 0x3e, 0x3e, 0x3d, 0x3d, 0x3d, 0x3c, 0x3c,
	0x3c, 0x3b, 0x3b, 0x3b, 0x3a, 0x3a, 0x3a, 0x39, 0x39, 0x39, 0x38,
	0x38, 0x38, 0x37, 0x37, 0x37, 0x36, 0x36, 0x36, 0x35, 0x35, 0x35,
	0x34, 0x34, 0x34, 0x33, 0x33, 0x33, 0x32, 0x32, 0x32, 0x31, 0x31,
	0x31, 0x30, 0x30, 0x30, 0x06, 0x06, 0x06, 0x05, 0x05, 0x05, 0x04,
	0x04, 0x04, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t iwl4965_rf_gain_5ghz[IWL4965_MAX_PWR_INDEX + 1] = {
	0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3e, 0x3e, 0x3e, 0x3d, 0x3d, 0x3d,
	0x3c, 0x3c, 0x3c, 0x3b, 0x3b, 0x3b, 0x3a, 0x3a, 0x3a, 0x39, 0x39,
	0x39, 0x38, 0x38, 0x38, 0x37, 0x37, 0x37, 0x36, 0x36, 0x36, 0x35,
	0x35, 0x35, 0x34, 0x34, 0x34, 0x33, 0x33, 0x33, 0x32, 0x32, 0x32,
	0x31, 0x31, 0x31, 0x30, 0x30, 0x30, 0x25, 0x25, 0x25, 0x24, 0x24,
	0x24, 0x23, 0x23, 0x23, 0x22, 0x18, 0x18, 0x17, 0x17, 0x17, 0x16,
	0x16, 0x16, 0x15, 0x15, 0x15, 0x14, 0x14, 0x14, 0x13, 0x13, 0x13,
	0x12, 0x08, 0x08, 0x07, 0x07, 0x07, 0x06, 0x06, 0x06, 0x05, 0x05,
	0x05, 0x04, 0x04, 0x04, 0x03, 0x03, 0x03, 0x02, 0x02, 0x02, 0x01,
	0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*
 * DSP pre-DAC gain values from highest to lowest power (values obtained
 * from the reference driver.)
 */
static const uint8_t iwl4965_dsp_gain_2ghz[IWL4965_MAX_PWR_INDEX + 1] = {
	0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68,
	0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e,
	0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62,
	0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68,
	0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e,
	0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62,
	0x6e, 0x68, 0x62, 0x61, 0x60, 0x5f, 0x5e, 0x5d, 0x5c, 0x5b, 0x5a,
	0x59, 0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51, 0x50, 0x4f,
	0x4e, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48, 0x47, 0x46, 0x45, 0x44,
	0x43, 0x42, 0x41, 0x40, 0x3f, 0x3e, 0x3d, 0x3c, 0x3b
};

static const uint8_t iwl4965_dsp_gain_5ghz[IWL4965_MAX_PWR_INDEX + 1] = {
	0x7b, 0x75, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62,
	0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68,
	0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e,
	0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62,
	0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68,
	0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e,
	0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62,
	0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68,
	0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e, 0x68, 0x62, 0x6e,
	0x68, 0x62, 0x6e, 0x68, 0x62, 0x5d, 0x58, 0x53, 0x4e
};

/*
 * Power saving settings (values obtained from the reference driver.)
 */
#define IWL_NDTIMRANGES		3
#define IWL_NPOWERLEVELS	6
static const struct iwl_pmgt {
	uint32_t	rxtimeout;
	uint32_t	txtimeout;
	uint32_t	intval[5];
	int		skip_dtim;
} iwl_pmgt[IWL_NDTIMRANGES][IWL_NPOWERLEVELS] = {
	/* DTIM <= 2 */
	{
	{   0,   0, {  0,  0,  0,  0,  0 }, 0 },	/* CAM */
	{ 200, 500, {  1,  2,  2,  2, -1 }, 0 },	/* PS level 1 */
	{ 200, 300, {  1,  2,  2,  2, -1 }, 0 },	/* PS level 2 */
	{  50, 100, {  2,  2,  2,  2, -1 }, 0 },	/* PS level 3 */
	{  50,  25, {  2,  2,  4,  4, -1 }, 1 },	/* PS level 4 */
	{  25,  25, {  2,  2,  4,  6, -1 }, 2 }		/* PS level 5 */
	},
	/* 3 <= DTIM <= 10 */
	{
	{   0,   0, {  0,  0,  0,  0,  0 }, 0 },	/* CAM */
	{ 200, 500, {  1,  2,  3,  4,  4 }, 0 },	/* PS level 1 */
	{ 200, 300, {  1,  2,  3,  4,  7 }, 0 },	/* PS level 2 */
	{  50, 100, {  2,  4,  6,  7,  9 }, 0 },	/* PS level 3 */
	{  50,  25, {  2,  4,  6,  9, 10 }, 1 },	/* PS level 4 */
	{  25,  25, {  2,  4,  7, 10, 10 }, 2 }		/* PS level 5 */
	},
	/* DTIM >= 11 */
	{
	{   0,   0, {  0,  0,  0,  0,  0 }, 0 },	/* CAM */
	{ 200, 500, {  1,  2,  3,  4, -1 }, 0 },	/* PS level 1 */
	{ 200, 300, {  2,  4,  6,  7, -1 }, 0 },	/* PS level 2 */
	{  50, 100, {  2,  7,  9,  9, -1 }, 0 },	/* PS level 3 */
	{  50,  25, {  2,  7,  9,  9, -1 }, 0 },	/* PS level 4 */
	{  25,  25, {  4,  7, 10, 10, -1 }, 0 }		/* PS level 5 */
	}
};

struct iwl_sensitivity_limits {
	uint32_t	min_ofdm_x1;
	uint32_t	max_ofdm_x1;
	uint32_t	min_ofdm_mrc_x1;
	uint32_t	max_ofdm_mrc_x1;
	uint32_t	min_ofdm_x4;
	uint32_t	max_ofdm_x4;
	uint32_t	min_ofdm_mrc_x4;
	uint32_t	max_ofdm_mrc_x4;
	uint32_t	min_cck_x4;
	uint32_t	max_cck_x4;
	uint32_t	min_cck_mrc_x4;
	uint32_t	max_cck_mrc_x4;
	uint32_t	min_energy_cck;
	uint32_t	energy_cck;
	uint32_t	energy_ofdm;
};

/*
 * RX sensitivity limits (values obtained from the reference driver.)
 */
static const struct iwl_sensitivity_limits iwl4965_sensitivity_limits = {
	105, 140,
	220, 270,
	 85, 120,
	170, 210,
	125, 200,
	200, 400,
	 97,
	100,
	100
};

static const struct iwl_sensitivity_limits iwl5000_sensitivity_limits = {
	120, 120,	/* min = max for performance bug in DSP. */
	240, 240,	/* min = max for performance bug in DSP. */
	 90, 120,
	170, 210,
	125, 200,
	170, 400,
	 95,
	 95,
	 95
};

static const struct iwl_sensitivity_limits iwl5150_sensitivity_limits = {
	105, 105,	/* min = max for performance bug in DSP. */
	220, 220,	/* min = max for performance bug in DSP. */
	 90, 120,
	170, 210,
	125, 200,
	170, 400,
	 95,
	 95,
	 95
};

static const struct iwl_sensitivity_limits iwl1000_sensitivity_limits = {
	120, 155,
	240, 290,
	 90, 120,
	170, 210,
	125, 200,
	170, 400,
	 95,
	 95,
	 95
};

static const struct iwl_sensitivity_limits iwl6000_sensitivity_limits = {
	105, 110,
	192, 232,
	 80, 145,
	128, 232,
	125, 175,
	160, 310,
	 97,
	 97,
	100
};
/* Get value from linux kernel 3.2.+ */
static const struct iwl_sensitivity_limits iwl2230_sensitivity_limits = {
    105,110,
    128,232,
	80,145,
	128,232,
	125,175,
	160,310,
	97,
	97,
	110
};

/* Map TID to TX scheduler's FIFO. */
static const uint8_t iwl_tid2fifo[] = {
	1, 0, 0, 1, 2, 2, 3, 3, 7, 7, 7, 7, 7, 7, 7, 7, 3
};

/* WiFi/WiMAX coexist event priority table for 6050. */
static const struct iwl5000_wimax_event iwl6050_wimax_events[] = {
	{ 0x04, 0x03, 0x00, 0x00 },
	{ 0x04, 0x03, 0x00, 0x03 },
	{ 0x04, 0x03, 0x00, 0x03 },
	{ 0x04, 0x03, 0x00, 0x03 },
	{ 0x04, 0x03, 0x00, 0x00 },
	{ 0x04, 0x03, 0x00, 0x07 },
	{ 0x04, 0x03, 0x00, 0x00 },
	{ 0x04, 0x03, 0x00, 0x03 },
	{ 0x04, 0x03, 0x00, 0x03 },
	{ 0x04, 0x03, 0x00, 0x00 },
	{ 0x06, 0x03, 0x00, 0x07 },
	{ 0x04, 0x03, 0x00, 0x00 },
	{ 0x06, 0x06, 0x00, 0x03 },
	{ 0x04, 0x03, 0x00, 0x07 },
	{ 0x04, 0x03, 0x00, 0x00 },
	{ 0x04, 0x03, 0x00, 0x00 }
};

/* Firmware errors. */
static const char * const iwl_fw_errmsg[] = {
	"OK",
	"FAIL",
	"BAD_PARAM",
	"BAD_CHECKSUM",
	"NMI_INTERRUPT_WDG",
	"SYSASSERT",
	"FATAL_ERROR",
	"BAD_COMMAND",
	"HW_ERROR_TUNE_LOCK",
	"HW_ERROR_TEMPERATURE",
	"ILLEGAL_CHAN_FREQ",
	"VCC_NOT_STABLE",
	"FH_ERROR",
	"NMI_INTERRUPT_HOST",
	"NMI_INTERRUPT_ACTION_PT",
	"NMI_INTERRUPT_UNKNOWN",
	"UCODE_VERSION_MISMATCH",
	"HW_ERROR_ABS_LOCK",
	"HW_ERROR_CAL_LOCK_FAIL",
	"NMI_INTERRUPT_INST_ACTION_PT",
	"NMI_INTERRUPT_DATA_ACTION_PT",
	"NMI_TRM_HW_ER",
	"NMI_INTERRUPT_TRM",
	"NMI_INTERRUPT_BREAKPOINT"
	"DEBUG_0",
	"DEBUG_1",
	"DEBUG_2",
	"DEBUG_3",
	"ADVANCED_SYSASSERT"
};

/* Find least significant bit that is set. */
#define IWL_LSB(x)	((((x) - 1) & (x)) ^ (x))

#define IWL_READ(sc, reg)						\
	bus_space_read_4((sc)->sc_st, (sc)->sc_sh, (reg))

#define IWL_WRITE(sc, reg, val)						\
	bus_space_write_4((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define IWL_WRITE_1(sc, reg, val)					\
	bus_space_write_1((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define IWL_SETBITS(sc, reg, mask)					\
	IWL_WRITE(sc, reg, IWL_READ(sc, reg) | (mask))

#define IWL_CLRBITS(sc, reg, mask)					\
	IWL_WRITE(sc, reg, IWL_READ(sc, reg) & ~(mask))

#define IWL_BARRIER_WRITE(sc)						\
	bus_space_barrier((sc)->sc_st, (sc)->sc_sh, 0, (sc)->sc_sz,	\
	    BUS_SPACE_BARRIER_WRITE)

#define IWL_BARRIER_READ_WRITE(sc)					\
	bus_space_barrier((sc)->sc_st, (sc)->sc_sh, 0, (sc)->sc_sz,	\
	    BUS_SPACE_BARRIER_READ | BUS_SPACE_BARRIER_WRITE)
