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

#ifndef	__iwl_dma_h__
#define	__iwl_dma_h__

/*
 * This	file implements/defines	DMA Engine (Flow Handler - FH) Registers
 * of Section 7.4 of PRM
 */

/*
 * Flow-Handler	registers
 */
#define	IWL_FH_TFBD_CTRL0(qid)			(0x1900	+ (qid)	* 8)
#define	IWL_FH_TFBD_CTRL1(qid)			(0x1904	+ (qid)	* 8)
#define	IWL_FH_KW_ADDR				0x197c
#define	IWL_FH_SRAM_ADDR(qid)			(0x19a4	+ (qid)	* 4)
#define	IWL_FH_CBBC_QUEUE(qid)			(0x19d0	+ (qid)	* 4)
#define	IWL_FH_STATUS_WPTR			0x1bc0
#define	IWL_FH_RX_BASE				0x1bc4
#define	IWL_FH_RX_WPTR				0x1bc8
#define	IWL_FH_RX_CONFIG			0x1c00
#define	IWL_FH_RX_STATUS			0x1c44
#define	IWL_FH_TX_CONFIG(qid)			(0x1d00	+ (qid)	* 32)
#define	IWL_FH_TXBUF_STATUS(qid)		(0x1d08	+ (qid)	* 32)
#define	IWL_FH_TX_CHICKEN			0x1e98
#define	IWL_FH_TX_STATUS			0x1eb0

/* Possible flags/values for register IWL_FH_TX_CONFIG.	*/
#define	IWL_FH_TX_CONFIG_DMA_PAUSE		0
#define	IWL_FH_TX_CONFIG_DMA_ENA		(1 << 31)
#define	IWL_FH_TX_CONFIG_CIRQ_HOST_ENDTFD	(1 << 20)
#define	IWL_FH_TX_CONFIG_DMA_CREDIT_ENA		(1 <<  3)

/* Possible flags/values for register IWL_FH_TXBUF_STATUS. */
#define	IWL_FH_TXBUF_STATUS_TBNUM(x)		((x) <<	20)
#define	IWL_FH_TXBUF_STATUS_TBIDX(x)		((x) <<	12)
#define	IWL_FH_TXBUF_STATUS_TFBD_VALID		3

/* Possible flags for register IWL_FH_TX_CHICKEN. */
#define	IWL_FH_TX_CHICKEN_SCHED_RETRY		(1 << 1)

/* Possible flags for register IWL_FH_TX_STATUS. */
#define IWL_FH_TX_STATUS_IDLE(chnl)		(1 << ((chnl) + 16))

/*
 * TX scheduler	registers.
 */
#define	IWL_SCHED_BASE				0xa02c00
#define	IWL_SCHED_SRAM_ADDR			(IWL_SCHED_BASE	+ 0x000)
#define	IWL5000_SCHED_DRAM_ADDR			(IWL_SCHED_BASE	+ 0x008)
#define	IWL5000_SCHED_TXFACT			(IWL_SCHED_BASE	+ 0x010)
#define	IWL5000_SCHED_QUEUE_RDPTR(qid)		(IWL_SCHED_BASE	+ 0x068	+ (qid)	* 4)
#define	IWL5000_SCHED_QCHAIN_SEL		(IWL_SCHED_BASE	+ 0x0e8)
#define	IWL5000_SCHED_INTR_MASK			(IWL_SCHED_BASE	+ 0x108)
#define	IWL5000_SCHED_QUEUE_STATUS(qid)		(IWL_SCHED_BASE	+ 0x10c	+ (qid)	* 4)
#define	IWL5000_SCHED_AGGR_SEL			(IWL_SCHED_BASE	+ 0x248)

/* Possible flags for register IWL_SCHED_QUEUE_STATUS. */
#define	IWL5000_TXQ_STATUS_ACTIVE		0x00ff0018
#define	IWL5000_TXQ_STATUS_INACTIVE		0x00ff0010
#define	IWL5000_TXQ_STATUS_CHGACT		(1 << 19)
/*
 * Offsets in TX scheduler's SRAM.
 */
#define	IWL5000_SCHED_CTX_OFF			0x600
#define	IWL5000_SCHED_CTX_LEN			520
#define	IWL5000_SCHED_QUEUE_OFFSET(qid)		(0x600 + (qid) * 8)
#define	IWL5000_SCHED_TRANS_TBL(qid)		(0x7e0 + (qid) * 2)


/* Possible flags for register IWL_FH_RX_CONFIG. */
#define	IWL_FH_RX_CONFIG_ENA			(1 << 31)
#define	IWL_FH_RX_CONFIG_NRBD(x)		((x) <<	20)
#define	IWL_FH_RX_CONFIG_RB_SIZE_8K		(1 << 16)
#define	IWL_FH_RX_CONFIG_SINGLE_FRAME		(1 << 15)
#define	IWL_FH_RX_CONFIG_IRQ_DST_HOST		(1 << 12)
#define	IWL_FH_RX_CONFIG_RB_TIMEOUT(x)		((x) <<	4)
#define	IWL_FH_RX_CONFIG_IGN_RXF_EMPTY		(1 <<  2)

/* Possible flags for register IWL_FH_RX_STATUS. */
#define	IWL_FH_RX_STATUS_IDLE			(1 << 24)

#endif /* __iwl_dma_h__	*/
