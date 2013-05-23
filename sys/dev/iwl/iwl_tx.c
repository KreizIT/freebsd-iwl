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

static const uint8_t iwl_bss_ac_to_queue[] = {
	2, 3, 1, 0,
};

static const uint8_t iwl_pan_ac_to_queue[] = {
	5, 4, 6, 7,
};

int
iwl_alloc_tx_ring(struct iwl_softc *sc, struct iwl_tx_ring *ring, int qid)
{
	bus_addr_t paddr;
	bus_size_t size;
	int i, error;

	ring->qid = qid;
	ring->queued = 0;
	ring->cur = 0;

	/* Allocate TX descriptors (256-byte aligned). */
	size = IWL_TX_RING_COUNT * sizeof (struct iwl_tx_desc);
	error = iwl_dma_contig_alloc(sc, &ring->desc_dma, (void **)&ring->desc, size, 256);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not allocate TX ring DMA memory, error %d\n",
		    __func__, error);
		goto fail;
	}

	size = IWL_TX_RING_COUNT * sizeof (struct iwl_tx_cmd);
	error = iwl_dma_contig_alloc(sc, &ring->cmd_dma, (void **)&ring->cmd, size, 4);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not allocate TX cmd DMA memory, error %d\n",
		    __func__, error);
		goto fail;
	}

	error = bus_dma_tag_create(bus_get_dma_tag(sc->sc_dev), 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, MCLBYTES,
	    IWL_MAX_SCATTER - 1, MCLBYTES, BUS_DMA_NOWAIT, NULL, NULL,
	    &ring->data_dmat);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not create TX buf DMA tag, error %d\n",
		    __func__, error);
		goto fail;
	}

	paddr = ring->cmd_dma.paddr;
	for (i = 0; i < IWL_TX_RING_COUNT; i++) {
		struct iwl_tx_data *data = &ring->data[i];

		data->cmd_paddr = paddr;
		data->scratch_paddr = paddr + 12;
		paddr += sizeof (struct iwl_tx_cmd);

		error = bus_dmamap_create(ring->data_dmat, 0, &data->map);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not create TX buf DMA map, error %d\n",
			    __func__, error);
			goto fail;
		}
	}
	return 0;

fail:	iwl_free_tx_ring(sc, ring);
	return error;
}

void
iwl_reset_tx_ring(struct iwl_softc *sc, struct iwl_tx_ring *ring)
{
	int i;

	for (i = 0; i < IWL_TX_RING_COUNT; i++) {
		struct iwl_tx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dmat, data->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dmat, data->map);
			m_freem(data->m);
			data->m = NULL;
		}
	}
	/* Clear TX descriptors. */
	memset(ring->desc, 0, ring->desc_dma.size);
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);
	sc->qfullmsk &= ~(1 << ring->qid);
	ring->queued = 0;
	ring->cur = 0;
}

void
iwl_free_tx_ring(struct iwl_softc *sc, struct iwl_tx_ring *ring)
{
	int i;

	iwl_dma_contig_free(&ring->desc_dma);
	iwl_dma_contig_free(&ring->cmd_dma);

	for (i = 0; i < IWL_TX_RING_COUNT; i++) {
		struct iwl_tx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dmat, data->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dmat, data->map);
			m_freem(data->m);
		}
		if (data->map != NULL)
			bus_dmamap_destroy(ring->data_dmat, data->map);
	}
	if (ring->data_dmat != NULL) {
		bus_dma_tag_destroy(ring->data_dmat);
		ring->data_dmat = NULL;
	}
}

int
iwl_alloc_sched(struct iwl_softc *sc)
{
	/* TX scheduler rings must be aligned on a 1KB boundary. */
	return iwl_dma_contig_alloc(sc, &sc->sched_dma, (void **)&sc->sched,
	    sc->schedsz, 1024);
}

void
iwl_free_sched(struct iwl_softc *sc)
{
	iwl_dma_contig_free(&sc->sched_dma);
}

int
iwl_alloc_kw(struct iwl_softc *sc)
{
	/* "Keep Warm" page must be aligned on a 4KB boundary. */
	return iwl_dma_contig_alloc(sc, &sc->kw_dma, NULL, 4096, 4096);
}

void
iwl_free_kw(struct iwl_softc *sc)
{
	iwl_dma_contig_free(&sc->kw_dma);
}

void
iwl5000_update_sched(struct iwl_softc *sc, int qid, int idx, uint8_t id,
    uint16_t len)
{
	uint16_t *w = &sc->sched[qid * IWL5000_SCHED_COUNT + idx];

	*w = htole16(id << 12 | (len + 8));
	bus_dmamap_sync(sc->sched_dma.tag, sc->sched_dma.map,
	    BUS_DMASYNC_PREWRITE);
	if (idx < IWL_SCHED_WINSZ) {
		*(w + IWL_TX_RING_COUNT) = *w;
		bus_dmamap_sync(sc->sched_dma.tag, sc->sched_dma.map,
		    BUS_DMASYNC_PREWRITE);
	}
}

#ifdef notyet
void
iwl5000_reset_sched(struct iwl_softc *sc, int qid, int idx)
{
	uint16_t *w = &sc->sched[qid * IWL5000_SCHED_COUNT + idx];

	*w = (*w & htole16(0xf000)) | htole16(1);
	bus_dmamap_sync(sc->sched_dma.tag, sc->sched_dma.map,
	    BUS_DMASYNC_PREWRITE);
	if (idx < IWL_SCHED_WINSZ) {
		*(w + IWL_TX_RING_COUNT) = *w;
		bus_dmamap_sync(sc->sched_dma.tag, sc->sched_dma.map,
		    BUS_DMASYNC_PREWRITE);
	}
}
#endif

int
iwl_tx_data(struct iwl_softc *sc, struct mbuf *m, struct ieee80211_node *ni)
{
	struct iwl_ops *ops = &sc->ops;
	const struct ieee80211_txparam *tp;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ni->ni_ic;
	struct iwl_node *wn = (void *)ni;
	struct iwl_tx_ring *ring;
	struct iwl_tx_desc *desc;
	struct iwl_tx_data *data;
	struct iwl_tx_cmd *cmd;
	struct iwl_cmd_data *tx;
	struct ieee80211_frame *wh;
	struct ieee80211_key *k = NULL;
	struct mbuf *m1;
	uint32_t flags;
	uint16_t qos;
	u_int hdrlen;
	bus_dma_segment_t *seg, segs[IWL_MAX_SCATTER];
	uint8_t tid, ridx, txant, type;
	int ac, i, totlen, error, pad, nsegs = 0, rate;
	struct iwl_vap *ivp = IWL_VAP(vap);

	IWL_LOCK_ASSERT(sc);

	wh = mtod(m, struct ieee80211_frame *);
	hdrlen = ieee80211_anyhdrsize(wh);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;

	/* Select EDCA Access Category and TX ring for this frame. */
	if (IEEE80211_QOS_HAS_SEQ(wh)) {
		qos = ((const struct ieee80211_qosframe *)wh)->i_qos[0];
		tid = qos & IEEE80211_QOS_TID;
	} else {
		qos = 0;
		tid = 0;
	}

	if(ivp->ctx == IWL_RXON_PAN_CTX)
		ac = iwl_pan_ac_to_queue[M_WME_GETAC(m)];
	else
		ac = iwl_bss_ac_to_queue[M_WME_GETAC(m)];

	if (IEEE80211_QOS_HAS_SEQ(wh) &&
	    IEEE80211_AMPDU_RUNNING(&ni->ni_tx_ampdu[ac])) {
		struct ieee80211_tx_ampdu *tap = &ni->ni_tx_ampdu[ac];

		ring = &sc->txq[*(int *)tap->txa_private];
		*(uint16_t *)wh->i_seq =
		    htole16(ni->ni_txseqs[tid] << IEEE80211_SEQ_SEQ_SHIFT);
		ni->ni_txseqs[tid]++;
	} else 
		ring = &sc->txq[ac];

	desc = &ring->desc[ring->cur];
	data = &ring->data[ring->cur];

	/* Choose a TX rate index. */
	tp = &vap->iv_txparms[ieee80211_chan2mode(ni->ni_chan)];
	if (type == IEEE80211_FC0_TYPE_MGT)
		rate = tp->mgmtrate;
	else if (IEEE80211_IS_MULTICAST(wh->i_addr1))
		rate = tp->mcastrate;
	else if (tp->ucastrate != IEEE80211_FIXED_RATE_NONE)
		rate = tp->ucastrate;
	else
		rate = ni->ni_txrate;

	ridx = ic->ic_rt->rateCodeToIndex[rate];

	/* Encrypt the frame if need be. */
	if (wh->i_fc[1] & IEEE80211_FC1_WEP) {
		/* Retrieve key for TX. */
		k = ieee80211_crypto_encap(ni, m);
		if (k == NULL) {
			m_freem(m);
			return ENOBUFS;
		}
		/* 802.11 header may have moved. */
		wh = mtod(m, struct ieee80211_frame *);
	}
	totlen = m->m_pkthdr.len;

	if (ieee80211_radiotap_active_vap(vap)) {
		struct iwl_tx_radiotap_header *tap = &sc->sc_txtap;

		tap->wt_flags = 0;
		tap->wt_rate = rate;
		if (k != NULL)
			tap->wt_flags |= IEEE80211_RADIOTAP_F_WEP;

		ieee80211_radiotap_tx(vap, m);
	}

	/* Prepare TX firmware command. */
	cmd = &ring->cmd[ring->cur];
	cmd->code = IWL_CMD_TX_DATA;
	cmd->flags = 0;
	cmd->qid = ring->qid;
	cmd->idx = ring->cur;

	tx = (struct iwl_cmd_data *)cmd->data;
	/* NB: No need to clear tx, all fields are reinitialized here. */
	tx->scratch = 0;	/* clear "scratch" area */

	flags = 0;
	if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		/* Unicast frame, check if an ACK is expected. */
		if (!qos || (qos & IEEE80211_QOS_ACKPOLICY) !=
		    IEEE80211_QOS_ACKPOLICY_NOACK)
			flags |= IWL_TX_NEED_ACK;
	}
	if ((wh->i_fc[0] &
	    (IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_MASK)) ==
	    (IEEE80211_FC0_TYPE_CTL | IEEE80211_FC0_SUBTYPE_BAR))
		flags |= IWL_TX_IMM_BA;		/* Cannot happen yet. */

	if (wh->i_fc[1] & IEEE80211_FC1_MORE_FRAG)
		flags |= IWL_TX_MORE_FRAG;	/* Cannot happen yet. */

	/* Check if frame must be protected using RTS/CTS or CTS-to-self. */
	if (!IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		/* NB: Group frames are sent using CCK in 802.11b/g. */
		if (totlen + IEEE80211_CRC_LEN > vap->iv_rtsthreshold) {
			flags |= IWL_TX_NEED_RTS;
		} else if ((ic->ic_flags & IEEE80211_F_USEPROT) &&
		    ridx >= IWL_RIDX_OFDM6) {
			if (ic->ic_protmode == IEEE80211_PROT_CTSONLY)
				flags |= IWL_TX_NEED_CTS;
			else if (ic->ic_protmode == IEEE80211_PROT_RTSCTS)
				flags |= IWL_TX_NEED_RTS;
		}
		if (flags & (IWL_TX_NEED_RTS | IWL_TX_NEED_CTS)) {
			if (sc->hw_type != IWL_HW_REV_TYPE_4965) {
				/* 5000 autoselects RTS/CTS or CTS-to-self. */
				flags &= ~(IWL_TX_NEED_RTS | IWL_TX_NEED_CTS);
				flags |= IWL_TX_NEED_PROTECTION;
			} else
				flags |= IWL_TX_FULL_TXOP;
		}
	}

	if (IEEE80211_IS_MULTICAST(wh->i_addr1) ||
	    type != IEEE80211_FC0_TYPE_DATA) {
		if(ivp->ctx == IWL_RXON_PAN_CTX)
			tx->id = IWL_PAN_BCAST_ID;
		else
			tx->id = IWL_BROADCAST_ID;
	}
	else
		tx->id = wn->id;

	if (type == IEEE80211_FC0_TYPE_MGT) {
		uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

		/* Tell HW to set timestamp in probe responses. */
		if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			flags |= IWL_TX_INSERT_TSTAMP;
		if (subtype == IEEE80211_FC0_SUBTYPE_ASSOC_REQ ||
		    subtype == IEEE80211_FC0_SUBTYPE_REASSOC_REQ)
			tx->timeout = htole16(3);
		else
			tx->timeout = htole16(2);
	} else
		tx->timeout = htole16(0);

	if (hdrlen & 3) {
		/* First segment length must be a multiple of 4. */
		flags |= IWL_TX_NEED_PADDING;
		pad = 4 - (hdrlen & 3);
	} else
		pad = 0;

	tx->len = htole16(totlen);
	tx->tid = tid;
	tx->rts_ntries = 60;
	tx->data_ntries = 15;
	tx->lifetime = htole32(IWL_LIFETIME_INFINITE);
	tx->rate = wn->ridx[rate];
	if ((tx->id == IWL_PAN_BCAST_ID) || (tx->id == IWL_BROADCAST_ID)) {
		/* Group or management frame. */
		tx->linkq = 0;
		/* XXX Alternate between antenna A and B? */
		txant = IWL_LSB(sc->txchainmask);
		tx->rate |= htole32(IWL_RFLAG_ANT(txant));
	} else {
		tx->linkq = ni->ni_rates.rs_nrates - ridx - 1;
		flags |= IWL_TX_LINKQ;	/* enable MRR */
	}
	/* Set physical address of "scratch area". */
	tx->loaddr = htole32(IWL_LOADDR(data->scratch_paddr));
	tx->hiaddr = IWL_HIADDR(data->scratch_paddr);

	/* Copy 802.11 header in TX command. */
	memcpy((uint8_t *)(tx + 1), wh, hdrlen);

	/* Trim 802.11 header. */
	m_adj(m, hdrlen);
	tx->security = 0;
	tx->flags = htole32(flags);

	error = bus_dmamap_load_mbuf_sg(ring->data_dmat, data->map, m, segs,
	    &nsegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		if (error != EFBIG) {
			device_printf(sc->sc_dev,
			    "%s: can't map mbuf (error %d)\n", __func__, error);
			m_freem(m);
			return error;
		}
		/* Too many DMA segments, linearize mbuf. */
		m1 = m_collapse(m, M_DONTWAIT, IWL_MAX_SCATTER);
		if (m1 == NULL) {
			device_printf(sc->sc_dev,
			    "%s: could not defrag mbuf\n", __func__);
			m_freem(m);
			return ENOBUFS;
		}
		m = m1;

		error = bus_dmamap_load_mbuf_sg(ring->data_dmat, data->map, m,
		    segs, &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: can't map mbuf (error %d)\n", __func__, error);
			m_freem(m);
			return error;
		}
	}

	data->m = m;
	data->ni = ni;

	DPRINTF(sc, IWL_DEBUG_XMIT, "%s: qid %d idx %d len %d nsegs %d\n",
	    __func__, ring->qid, ring->cur, m->m_pkthdr.len, nsegs);

	/* Fill TX descriptor. */
	desc->nsegs = 1;
	if (m->m_len != 0)
		desc->nsegs += nsegs;
	/* First DMA segment is used by the TX command. */
	desc->segs[0].addr = htole32(IWL_LOADDR(data->cmd_paddr));
	desc->segs[0].len  = htole16(IWL_HIADDR(data->cmd_paddr) |
	    (4 + sizeof (*tx) + hdrlen + pad) << 4);
	/* Other DMA segments are for data payload. */
	seg = &segs[0];
	for (i = 1; i <= nsegs; i++) {
		desc->segs[i].addr = htole32(IWL_LOADDR(seg->ds_addr));
		desc->segs[i].len  = htole16(IWL_HIADDR(seg->ds_addr) |
		    seg->ds_len << 4);
		seg++;
	}

	bus_dmamap_sync(ring->data_dmat, data->map, BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(ring->data_dmat, ring->cmd_dma.map,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

	/* Update TX scheduler. */
	if (ring->qid >= sc->firstaggqueue)
		ops->update_sched(sc, ring->qid, ring->cur, tx->id, totlen);

	/* Kick TX ring. */
	ring->cur = (ring->cur + 1) % IWL_TX_RING_COUNT;
	IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, ring->qid << 8 | ring->cur);

	/* Mark TX ring as full if we reach a certain threshold. */
	if (++ring->queued > IWL_TX_RING_HIMARK)
		sc->qfullmsk |= 1 << ring->qid;

	return 0;
}

int
iwl_tx_data_raw(struct iwl_softc *sc, struct mbuf *m,
    struct ieee80211_node *ni, const struct ieee80211_bpf_params *params)
{
	struct iwl_ops *ops = &sc->ops;
	struct ifnet *ifp = sc->sc_ifp;
	struct ieee80211vap *vap = ni->ni_vap;
	struct ieee80211com *ic = ifp->if_l2com;
	struct iwl_tx_cmd *cmd;
	struct iwl_cmd_data *tx;
	struct ieee80211_frame *wh;
	struct iwl_tx_ring *ring;
	struct iwl_tx_desc *desc;
	struct iwl_tx_data *data;
	struct mbuf *m1;
	bus_dma_segment_t *seg, segs[IWL_MAX_SCATTER];
	uint32_t flags;
	u_int hdrlen;
	int ac, totlen, error, pad, nsegs = 0, i, rate;
	uint8_t ridx, type, txant;
	struct iwl_vap *ivp = IWL_VAP(vap);

	IWL_LOCK_ASSERT(sc);

	wh = mtod(m, struct ieee80211_frame *);
	hdrlen = ieee80211_anyhdrsize(wh);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;

	ac = params->ibp_pri & 3;

	ring = &sc->txq[ac];
	desc = &ring->desc[ring->cur];
	data = &ring->data[ring->cur];

	/* Choose a TX rate index. */
	rate = params->ibp_rate0;
	ridx = ic->ic_rt->rateCodeToIndex[rate];
	if (ridx == (uint8_t)-1) {
		/* XXX fall back to mcast/mgmt rate? */
		m_freem(m);
		return EINVAL;
	}

	totlen = m->m_pkthdr.len;

	/* Prepare TX firmware command. */
	cmd = &ring->cmd[ring->cur];
	cmd->code = IWL_CMD_TX_DATA;
	cmd->flags = 0;
	cmd->qid = ring->qid;
	cmd->idx = ring->cur;

	tx = (struct iwl_cmd_data *)cmd->data;
	/* NB: No need to clear tx, all fields are reinitialized here. */
	tx->scratch = 0;	/* clear "scratch" area */

	flags = 0;
	if ((params->ibp_flags & IEEE80211_BPF_NOACK) == 0)
		flags |= IWL_TX_NEED_ACK;
	if (params->ibp_flags & IEEE80211_BPF_RTS) {
		if (sc->hw_type != IWL_HW_REV_TYPE_4965) {
			/* 5000 autoselects RTS/CTS or CTS-to-self. */
			flags &= ~IWL_TX_NEED_RTS;
			flags |= IWL_TX_NEED_PROTECTION;
		} else
			flags |= IWL_TX_NEED_RTS | IWL_TX_FULL_TXOP;
	}
	if (params->ibp_flags & IEEE80211_BPF_CTS) {
		if (sc->hw_type != IWL_HW_REV_TYPE_4965) {
			/* 5000 autoselects RTS/CTS or CTS-to-self. */
			flags &= ~IWL_TX_NEED_CTS;
			flags |= IWL_TX_NEED_PROTECTION;
		} else
			flags |= IWL_TX_NEED_CTS | IWL_TX_FULL_TXOP;
	}
	if (type == IEEE80211_FC0_TYPE_MGT) {
		uint8_t subtype = wh->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

		/* Tell HW to set timestamp in probe responses. */
		if (subtype == IEEE80211_FC0_SUBTYPE_PROBE_RESP)
			flags |= IWL_TX_INSERT_TSTAMP;

		if (subtype == IEEE80211_FC0_SUBTYPE_ASSOC_REQ ||
		    subtype == IEEE80211_FC0_SUBTYPE_REASSOC_REQ)
			tx->timeout = htole16(3);
		else
			tx->timeout = htole16(2);
	} else
		tx->timeout = htole16(0);

	if (hdrlen & 3) {
		/* First segment length must be a multiple of 4. */
		flags |= IWL_TX_NEED_PADDING;
		pad = 4 - (hdrlen & 3);
	} else
		pad = 0;

	if (ieee80211_radiotap_active_vap(vap)) {
		struct iwl_tx_radiotap_header *tap = &sc->sc_txtap;

		tap->wt_flags = 0;
		tap->wt_rate = rate;

		ieee80211_radiotap_tx(vap, m);
	}

	tx->len = htole16(totlen);
	tx->tid = 0;
	if(ivp->ctx == IWL_RXON_PAN_CTX)
		tx->id = IWL_PAN_BCAST_ID;
	else
		tx->id = IWL_BROADCAST_ID;
	tx->rts_ntries = params->ibp_try1;
	tx->data_ntries = params->ibp_try0;
	tx->lifetime = htole32(IWL_LIFETIME_INFINITE);
	tx->rate = htole32(rate2plcp(rate));
	if (ridx < IWL_RIDX_OFDM6 &&
	    IEEE80211_IS_CHAN_2GHZ(ni->ni_chan))
		tx->rate |= htole32(IWL_RFLAG_CCK);
	/* Group or management frame. */
	tx->linkq = 0;
	txant = IWL_LSB(sc->txchainmask);
	tx->rate |= htole32(IWL_RFLAG_ANT(txant));
	/* Set physical address of "scratch area". */
	tx->loaddr = htole32(IWL_LOADDR(data->scratch_paddr));
	tx->hiaddr = IWL_HIADDR(data->scratch_paddr);

	/* Copy 802.11 header in TX command. */
	memcpy((uint8_t *)(tx + 1), wh, hdrlen);

	/* Trim 802.11 header. */
	m_adj(m, hdrlen);
	tx->security = 0;
	tx->flags = htole32(flags);

	error = bus_dmamap_load_mbuf_sg(ring->data_dmat, data->map, m, segs,
	    &nsegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		if (error != EFBIG) {
			device_printf(sc->sc_dev,
			    "%s: can't map mbuf (error %d)\n", __func__, error);
			m_freem(m);
			return error;
		}
		/* Too many DMA segments, linearize mbuf. */
		m1 = m_collapse(m, M_DONTWAIT, IWL_MAX_SCATTER);
		if (m1 == NULL) {
			device_printf(sc->sc_dev,
			    "%s: could not defrag mbuf\n", __func__);
			m_freem(m);
			return ENOBUFS;
		}
		m = m1;

		error = bus_dmamap_load_mbuf_sg(ring->data_dmat, data->map, m,
		    segs, &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: can't map mbuf (error %d)\n", __func__, error);
			m_freem(m);
			return error;
		}
	}

	data->m = m;
	data->ni = ni;

	DPRINTF(sc, IWL_DEBUG_XMIT, "%s: qid %d idx %d len %d nsegs %d\n",
	    __func__, ring->qid, ring->cur, m->m_pkthdr.len, nsegs);

	/* Fill TX descriptor. */
	desc->nsegs = 1;
	if (m->m_len != 0)
		desc->nsegs += nsegs;
	/* First DMA segment is used by the TX command. */
	desc->segs[0].addr = htole32(IWL_LOADDR(data->cmd_paddr));
	desc->segs[0].len  = htole16(IWL_HIADDR(data->cmd_paddr) |
	    (4 + sizeof (*tx) + hdrlen + pad) << 4);
	/* Other DMA segments are for data payload. */
	seg = &segs[0];
	for (i = 1; i <= nsegs; i++) {
		desc->segs[i].addr = htole32(IWL_LOADDR(seg->ds_addr));
		desc->segs[i].len  = htole16(IWL_HIADDR(seg->ds_addr) |
		    seg->ds_len << 4);
		seg++;
	}

	bus_dmamap_sync(ring->data_dmat, data->map, BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(ring->data_dmat, ring->cmd_dma.map,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

	/* Update TX scheduler. */
	if (ring->qid >= sc->firstaggqueue)
		ops->update_sched(sc, ring->qid, ring->cur, tx->id, totlen);

	/* Kick TX ring. */
	ring->cur = (ring->cur + 1) % IWL_TX_RING_COUNT;
	IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, ring->qid << 8 | ring->cur);

	/* Mark TX ring as full if we reach a certain threshold. */
	if (++ring->queued > IWL_TX_RING_HIMARK)
		sc->qfullmsk |= 1 << ring->qid;

	return 0;
}

int
iwl_raw_xmit(struct ieee80211_node *ni, struct mbuf *m,
    const struct ieee80211_bpf_params *params)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct ifnet *ifp = ic->ic_ifp;
	struct iwl_softc *sc = ifp->if_softc;
	int error = 0;

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
		ieee80211_free_node(ni);
		m_freem(m);
		return ENETDOWN;
	}

	IWL_LOCK(sc);
	if (params == NULL) {
		/*
		 * Legacy path; interpret frame contents to decide
		 * precisely how to send the frame.
		 */
		error = iwl_tx_data(sc, m, ni);
	} else {
		/*
		 * Caller supplied explicit parameters to use in
		 * sending the frame.
		 */
		error = iwl_tx_data_raw(sc, m, ni, params);
	}
	if (error != 0) {
		/* NB: m is reclaimed on tx failure */
		ieee80211_free_node(ni);
		ifp->if_oerrors++;
	}

	sc->sc_tx_timer = 5;

	IWL_UNLOCK(sc);
	return error;
}

/*
 * Send a command to the firmware.
 */
int
iwl_cmd(struct iwl_softc *sc, int code, const void *buf, int size, int async)
{
	struct iwl_tx_ring *ring;
	struct iwl_tx_desc *desc;
	struct iwl_tx_data *data;
	struct iwl_tx_cmd *cmd;
	struct mbuf *m;
	bus_addr_t paddr;
	int totlen, error;
	int cmd_queue_num;

	if((sc->uc_scan_progress == 1) && (code != IWL_CMD_SCAN)) {
		//printf("\nScanning in progress..not sending cmd %x.", code);
		return 0;
	}

	if (async == 0)
		IWL_LOCK_ASSERT(sc);

	if(sc->uc_pan_support == IWL_UC_PAN_PRESENT)
		cmd_queue_num = IWL_PAN_CMD_QUEUE;
	else
		cmd_queue_num = IWL_CMD_QUEUE_NUM;

	ring = &sc->txq[cmd_queue_num];

	desc = &ring->desc[ring->cur];
	data = &ring->data[ring->cur];
	totlen = 4 + size;

	if (size > sizeof cmd->data) {
		/* Command is too large to fit in a descriptor. */
		if (totlen > MCLBYTES)
			return EINVAL;
		m = m_getjcl(M_DONTWAIT, MT_DATA, M_PKTHDR, MJUMPAGESIZE);
		if (m == NULL)
			return ENOMEM;
		cmd = mtod(m, struct iwl_tx_cmd *);
		error = bus_dmamap_load(ring->data_dmat, data->map, cmd,
		    totlen, iwl_dma_map_addr, &paddr, BUS_DMA_NOWAIT);
		if (error != 0) {
			m_freem(m);
			return error;
		}
		data->m = m;
	} else {
		cmd = &ring->cmd[ring->cur];
		paddr = data->cmd_paddr;
	}

	cmd->code = code;
	cmd->flags = 0;
	cmd->qid = ring->qid;
	cmd->idx = ring->cur;
	memcpy(cmd->data, buf, size);

	desc->nsegs = 1;
	desc->segs[0].addr = htole32(IWL_LOADDR(paddr));
	desc->segs[0].len  = htole16(IWL_HIADDR(paddr) | totlen << 4);

	DPRINTF(sc, IWL_DEBUG_CMD, "%s: %s (0x%x) flags %d qid %d idx %d\n",
	    __func__, iwl_intr_str(cmd->code), cmd->code,
	    cmd->flags, cmd->qid, cmd->idx);

	if (size > sizeof cmd->data)
		bus_dmamap_sync(ring->data_dmat, data->map,
		    BUS_DMASYNC_PREWRITE);
	else
		bus_dmamap_sync(ring->data_dmat, ring->cmd_dma.map,
		    BUS_DMASYNC_PREWRITE);

	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

	/* Kick command ring. */
	ring->cur = (ring->cur + 1) % IWL_TX_RING_COUNT;
	IWL_WRITE(sc, IWL_HBUS_TARG_WRPTR, ring->qid << 8 | ring->cur);

	return async ? 0 : msleep(desc, &sc->sc_mtx, PCATCH, "iwlcmd", hz);
}
