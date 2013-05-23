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

#ifndef	__iwl_scan_h__
#define	__iwl_scan_h__

#define	IWL_SCAN_CRC_TH_DISABLED	0
#define	IWL_SCAN_CRC_TH_DEFAULT		htole16(1)
#define	IWL_SCAN_CRC_TH_NEVER		htole16(0xffff)

/* Maximum size of a scan command. */
#define IWL_SCAN_MAXSZ			(MCLBYTES - 4)

#define IWL_ACTIVE_DWELL_TIME_24	(30)	/* all times in msec */
#define IWL_ACTIVE_DWELL_TIME_52	(20)
#define IWL_ACTIVE_DWELL_FACTOR_24	(3)
#define IWL_ACTIVE_DWELL_FACTOR_52	(2)

#define IWL_PASSIVE_DWELL_TIME_24	(20)	/* all times in msec */
#define IWL_PASSIVE_DWELL_TIME_52	(10)
#define IWL_PASSIVE_DWELL_BASE		(100)
#define IWL_CHANNEL_TUNE_TIME		(5)

#define IWL_SCAN_CHAN_TIMEOUT		2

/* Structures for command IWL_CMD_SCAN. */
struct iwl_scan_essid {
	uint8_t		id;
	uint8_t		len;
	uint8_t		data[IEEE80211_NWID_LEN];
} __packed;

struct iwl_scan_hdr {
	uint16_t	len;
	uint8_t		reserved1;
	uint8_t		nchan;
	uint16_t	quiet_time;
	uint16_t	quiet_threshold;
	uint16_t	crc_threshold;
	uint16_t	rxchain;
	uint32_t	max_svc;	/* background scans */
	uint32_t	pause_svc;	/* background scans */
	uint32_t	flags;
	uint32_t	filter;

	/* Followed by a struct iwl_cmd_data. */
	/* Followed by an array of 20 structs iwl_scan_essid. */
	/* Followed by probe request body. */
	/* Followed by an array of ``nchan'' structs iwl_scan_chan. */
} __packed;

/* Structure for IWL_START_SCAN notification. */
struct iwl_start_scan {
	uint64_t	tstamp;
	uint32_t	tbeacon;
	uint8_t		chan;
	uint8_t		band;
	uint16_t	reserved;
	uint32_t	status;
} __packed;

/* Structure for IWL_STOP_SCAN notification. */
struct iwl_stop_scan {
	uint8_t		nchan;
	uint8_t		status;
	uint8_t		reserved;
	uint8_t		chan;
	uint64_t	tsf;
} __packed;

struct iwl_scan_chan {
	uint32_t	flags;
#define IWL_CHAN_ACTIVE		(1 << 0)
#define IWL_CHAN_PASSIVE	(0 << 0)
#define IWL_CHAN_NPBREQS(x)	(((1 << (x)) - 1) << 1)

	uint16_t	chan;
	uint8_t		rf_gain;
	uint8_t		dsp_gain;
	uint16_t	active;		/* msecs */
	uint16_t	passive;	/* msecs */
} __packed;

uint8_t		*ieee80211_add_ssid(uint8_t *, const uint8_t *, u_int);
extern int	iwl_scan(struct iwl_softc *);
extern void	iwl_scan_start(struct ieee80211com *);
extern void	iwl_scan_end(struct ieee80211com *);
extern void	iwl_scan_curchan(struct ieee80211_scan_state *, unsigned long);
extern void	iwl_scan_mindwell(struct ieee80211_scan_state *);
extern uint16_t	iwl_get_active_dwell(struct iwl_softc *sc, struct ieee80211_channel *c);
extern uint16_t	iwl_get_passive_dwell(struct iwl_softc *sc, struct ieee80211_channel *c);

#endif /* __iwl_scan_h__ */
