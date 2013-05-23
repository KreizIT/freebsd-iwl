/*	$FreeBSD$	*/

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

#ifndef	__iwl_tx_h__
#define	__iwl_tx_h__

#define	IWL_TX_RING_COUNT		256
#define	IWL_TX_RING_LOMARK		192
#define	IWL_TX_RING_HIMARK		224

#define	IWL4965_NTXQUEUES		16
#define	IWL5000_NTXQUEUES		20

#define	IWL4965_FIRSTAGGQUEUE		7
#define	IWL5000_FIRSTAGGQUEUE		10

#define	IWL_CMD_QUEUE_NUM		4
#define	IWL_PAN_CMD_QUEUE		9

#define	IWL_SCHED_WINSZ			64
#define	IWL_SCHED_LIMIT			64
#define	IWL5000_SCHED_COUNT		(IWL_TX_RING_COUNT + IWL_SCHED_WINSZ)
#define	IWL5000_SCHEDSZ			(IWL5000_NTXQUEUES * IWL5000_SCHED_COUNT * 2)

/* Structures for IWL_TX_DONE notification. */
#define	IWL_TX_SUCCESS			0x00
#define	IWL_TX_FAIL			0x80	/* all failures	have 0x80 set */
#define	IWL_TX_FAIL_SHORT_LIMIT		0x82	/* too many RTS	retries	*/
#define	IWL_TX_FAIL_LONG_LIMIT		0x83	/* too many retries */
#define	IWL_TX_FAIL_FIFO_UNDERRRUN	0x84	/* tx fifo not kept running */
#define	IWL_TX_FAIL_DEST_IN_PS		0x88	/* sta found in	power save */
#define	IWL_TX_FAIL_TX_LOCKED		0x90	/* waiting to see traffic */

/* Maximum number of DMA segments for TX. */
#define	IWL_MAX_SCATTER			20

#define	IWL_CMD_SCRATCH_OFFSET		12

#define	IWL_TX_RADIOTAP_PRESENT						\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE)	|				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL))

struct iwl_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	uint8_t		wt_flags;
	uint8_t		wt_rate;
	uint16_t	wt_chan_freq;
	uint16_t	wt_chan_flags;
} __packed;

struct iwl_dma_info {
	bus_dma_tag_t		tag;
	bus_dmamap_t		map;
	bus_dma_segment_t	seg;
	bus_addr_t		paddr;
	caddr_t			vaddr;
	bus_size_t		size;
};

struct iwl_tx_desc {
	uint8_t		reserved1[3];
	uint8_t		nsegs;
	struct {
		uint32_t	addr;
		uint16_t	len;
	} __packed	segs[IWL_MAX_SCATTER];
	/* Pad to 128 bytes. */
	uint32_t	reserved2;
} __packed;

struct iwl_tx_cmd {
	uint8_t	code;
#define IWL_CMD_RXON			16
#define IWL_CMD_RXON_ASSOC		17
#define IWL_CMD_EDCA_PARAMS		19
#define IWL_CMD_TIMING			20
#define IWL_CMD_ADD_NODE		24
#define IWL_CMD_TX_DATA			28
#define IWL_CMD_LINK_QUALITY		78
#define IWL_CMD_SET_LED			72
#define IWL5000_CMD_WIMAX_COEX		90
#define IWL5000_CMD_CALIB_CONFIG	101
#define IWL5000_CMD_CALIB_RESULT	102
#define IWL5000_CMD_CALIB_COMPLETE	103
#define IWL_CMD_SET_POWER_MODE		119
#define IWL_CMD_SCAN			128
#define IWL_CMD_SCAN_RESULTS		131
#define IWL_CMD_TXPOWER_DBM		149
#define IWL_CMD_TXPOWER			151
#define IWL5000_CMD_TX_ANT_CONFIG	152
#define IWL_CMD_BT_COEX			155
#define IWL_CMD_GET_STATISTICS		156
#define IWL_CMD_SET_CRITICAL_TEMP	164
#define IWL_CMD_SET_SENSITIVITY		168
#define IWL_CMD_PHY_CALIB		176
#define IWL_CMD_BT_COEX_PRIOTABLE	204
#define IWL_CMD_BT_COEX_PROT		205
/* PAN commands */
#define IWL_CMD_WIPAN_PARAMS			0xb2
#define IWL_CMD_WIPAN_RXON			0xb3
#define IWL_CMD_WIPAN_RXON_TIMING		0xb4
#define IWL_CMD_WIPAN_RXON_ASSOC		0xb6
#define IWL_CMD_WIPAN_QOS_PARAM			0xb7
#define IWL_CMD_WIPAN_WEPKEY			0xb8
#define IWL_CMD_WIPAN_P2P_CHANNEL_SWITCH	0xb9
#define IWL_CMD_WIPAN_NOA_NOTIFICATION		0xbc
#define IWL_CMD_WIPAN_DEACTIVATION_COMPLETE	0xbd

	uint8_t	flags;
	uint8_t	idx;
	uint8_t	qid;
	uint8_t	data[136];
} __packed;

/* Structure for command IWL_CMD_TX_DATA. */
struct iwl_cmd_data {
	uint16_t	len;
	uint16_t	lnext;
	uint32_t	flags;
#define IWL_TX_NEED_PROTECTION	(1 <<  0)	/* 5000 only */
#define IWL_TX_NEED_RTS		(1 <<  1)
#define IWL_TX_NEED_CTS		(1 <<  2)
#define IWL_TX_NEED_ACK		(1 <<  3)
#define IWL_TX_LINKQ		(1 <<  4)
#define IWL_TX_IMM_BA		(1 <<  6)
#define IWL_TX_FULL_TXOP	(1 <<  7)
#define IWL_TX_BT_DISABLE	(1 << 12)	/* bluetooth coexistence */
#define IWL_TX_AUTO_SEQ		(1 << 13)
#define IWL_TX_MORE_FRAG	(1 << 14)
#define IWL_TX_INSERT_TSTAMP	(1 << 16)
#define IWL_TX_NEED_PADDING	(1 << 20)

	uint32_t	scratch;
	uint32_t	rate;

	uint8_t		id;
	uint8_t		security;
#define IWL_CIPHER_WEP40	1
#define IWL_CIPHER_CCMP		2
#define IWL_CIPHER_TKIP		3
#define IWL_CIPHER_WEP104	9

	uint8_t		linkq;
	uint8_t		reserved2;
	uint8_t		key[16];
	uint16_t	fnext;
	uint16_t	reserved3;
	uint32_t	lifetime;
#define IWL_LIFETIME_INFINITE	0xffffffff

	uint32_t	loaddr;
	uint8_t		hiaddr;
	uint8_t		rts_ntries;
	uint8_t		data_ntries;
	uint8_t		tid;
	uint16_t	timeout;
	uint16_t	txop;
} __packed;

struct iwl_tx_data {
	bus_dmamap_t		map;
	bus_addr_t		cmd_paddr;
	bus_addr_t		scratch_paddr;
	struct mbuf		*m;
	struct ieee80211_node	*ni;
};

struct iwl_tx_ring {
	struct iwl_dma_info	desc_dma;
	struct iwl_dma_info	cmd_dma;
	struct iwl_tx_desc	*desc;
	struct iwl_tx_cmd	*cmd;
	struct iwl_tx_data	data[IWL_TX_RING_COUNT];
	bus_dma_tag_t		data_dmat;
	int			qid;
	int			queued;
	int			cur;
	int			read;
};

struct iwl5000_tx_stat {
	uint8_t		nframes;
	uint8_t		btkillcnt;
	uint8_t		rtsfailcnt;
	uint8_t		ackfailcnt;
	uint32_t	rate;
	uint16_t	duration;
	uint16_t	reserved;
	uint32_t	power[2];
	uint32_t	info;
	uint16_t	seq;
	uint16_t	len;
	uint8_t		tlc;
	uint8_t		ratid;
	uint8_t		fc[2];
	uint16_t	status;
	uint16_t	sequence;
} __packed;

struct iwl_softc;

extern int	iwl_alloc_tx_ring(struct iwl_softc *, struct iwl_tx_ring *,
		    int);
extern void	iwl_reset_tx_ring(struct iwl_softc *, struct iwl_tx_ring *);
extern void	iwl_free_tx_ring(struct iwl_softc *, struct iwl_tx_ring *);
extern int	iwl_alloc_sched(struct iwl_softc *);
extern void	iwl_free_sched(struct iwl_softc *);
extern int	iwl_alloc_kw(struct iwl_softc *);
extern void	iwl_free_kw(struct iwl_softc *);
extern void	iwl5000_update_sched(struct iwl_softc *, int, int, uint8_t,
		    uint16_t);
#ifdef notyet
extern void	iwl5000_reset_sched(struct iwl_softc *, int, int);
#endif
extern int	iwl_tx_data(struct iwl_softc *, struct mbuf *,
		    struct ieee80211_node *);
extern int	iwl_tx_data_raw(struct iwl_softc *, struct mbuf *,
		    struct ieee80211_node *,
		    const struct ieee80211_bpf_params *params);
extern int	iwl_raw_xmit(struct ieee80211_node *, struct mbuf *,
		    const struct ieee80211_bpf_params *);
extern int	iwl_cmd(struct iwl_softc *, int, const void *, int, int);
extern int	rate2plcp(int rate);

#endif /* __iwl_tx_h__ */
