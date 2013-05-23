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

void
iwl_set_led(struct iwl_softc *sc, uint8_t which, uint8_t off, uint8_t on, uint8_t mode)
{
	struct iwl_cmd_led led;

	/* Clear microcode LED ownership. */
	IWL_CLRBITS(sc, IWL_LED, IWL_LED_BSM_CTRL);

	led.which = which;

	if(mode == IWL_LED_SLOW_BLINK)
		led.unit = htole32(IWL_LED_UNIT * 5); /* on/off in unit of 10ms */
	else
		led.unit = htole32(IWL_LED_UNIT);

	led.off = off;
	led.on  = on;

	(void)iwl_cmd(sc, IWL_CMD_SET_LED, &led, sizeof led, 1);
}

void
iwl_led_pattern(struct iwl_softc *sc)
{
	int i,j;

	for(i = 0; i < IWL_MAX_BLINK_TBL; i++) {
		if((sc->sc_led.led_last_tpt) > (blink_tbl[i].tpt * 1024 * 1024))
			break;
	}

	for(j = 0; j < IWL_MAX_BLINK_TBL; j++) {
		if((sc->sc_led.led_cur_tpt) > (blink_tbl[j].tpt * 1024 * 1024))
			break;
	}

	if (i != j)
		iwl_set_led(sc, IWL_LED_LINK,blink_tbl[j].off_time,
		    blink_tbl[j].on_time, IWL_LED_INT_BLINK);
}
