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

#ifndef	__iwl_common_h__
#define	__iwl_common_h__

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/mbuf.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/limits.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <sys/syslog.h>
#include <sys/errno.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/clock.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_ioctl.h>

#include "iwl_reg.h"
#include "iwl_var.h"
#include "iwl_scan.h"
#include "iwl_led.h"

#define IWL_DEBUG
#ifdef IWL_DEBUG
enum {
	IWL_DEBUG_XMIT		= 0x00000001,	/* basic xmit operation */
	IWL_DEBUG_RECV		= 0x00000002,	/* basic recv operation */
	IWL_DEBUG_STATE		= 0x00000004,	/* 802.11 state transitions */
	IWL_DEBUG_TXPOW		= 0x00000008,	/* tx power processing */
	IWL_DEBUG_RESET		= 0x00000010,	/* reset processing */
	IWL_DEBUG_OPS		= 0x00000020,	/* iwl_ops processing */
	IWL_DEBUG_BEACON 	= 0x00000040,	/* beacon handling */
	IWL_DEBUG_WATCHDOG 	= 0x00000080,	/* watchdog timeout */
	IWL_DEBUG_INTR		= 0x00000100,	/* ISR */
	IWL_DEBUG_CALIBRATE	= 0x00000200,	/* periodic calibration */
	IWL_DEBUG_NODE		= 0x00000400,	/* node management */
	IWL_DEBUG_LED		= 0x00000800,	/* led management */
	IWL_DEBUG_CMD		= 0x00001000,	/* cmd submission */
	IWL_DEBUG_FATAL		= 0x80000000,	/* fatal errors */
	IWL_DEBUG_ANY		= 0xffffffff
};

#define DPRINTF(sc, m, fmt, ...) do {			\
	if (sc->sc_debug & (m))				\
		printf(fmt, __VA_ARGS__);		\
} while (0)

extern const char * iwl_intr_str(uint8_t cmd);

#else
#define DPRINTF(sc, m, fmt, ...) do { (void) sc; } while (0)
#endif



extern void	iwl_dma_map_addr(void *, bus_dma_segment_t *, int, int);
extern int	iwl_dma_contig_alloc(struct iwl_softc *, struct iwl_dma_info *,
		    void **, bus_size_t, bus_size_t);
extern void	iwl_dma_contig_free(struct iwl_dma_info *);

extern int	iwl_attach(device_t);
extern int	iwl_detach(device_t);
extern int	iwl_shutdown(device_t);
extern int	iwl_suspend(device_t);
extern int	iwl_resume(device_t);
extern void	iwl_intr(void *);
int		iwl_nic_lock(struct iwl_softc *);
int		iwl_clock_wait(struct iwl_softc *);
int		iwl_apm_init(struct iwl_softc *);
void 		iwl_apm_stop(struct iwl_softc *);
uint32_t 	iwl_prph_read(struct iwl_softc *, uint32_t);
extern  void	iwl_prph_write(struct iwl_softc *, uint32_t , uint32_t);
extern void	iwl_prph_setbits(struct iwl_softc *, uint32_t, uint32_t);
extern  void	iwl_prph_clrbits(struct iwl_softc *, uint32_t, uint32_t);
extern void 	iwl_nic_unlock(struct iwl_softc *);
extern struct	iwl_eeprom_chan *iwl_find_eeprom_channel(struct iwl_softc *,
		    struct ieee80211_channel *);
extern int	iwl_set_pan_params(struct iwl_softc *);
extern int	iwl_set_pslevel(struct iwl_softc *, int, int, int);

#endif /* __iwl_common_h__ */
