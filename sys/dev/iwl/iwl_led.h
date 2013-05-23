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

#ifndef __iwl_led_h__
#define __iwl_led_h__

#define	IWL_MAX_BLINK_TBL	10
#define	IWL_LED_STATIC_ON	0
#define	IWL_LED_STATIC_OFF	1
#define	IWL_LED_SLOW_BLINK	2
#define	IWL_LED_INT_BLINK	3
#define	IWL_LED_UNIT		0x1388 /* 5 ms */

static const struct {
	uint16_t tpt;	/* Mb/s */
	uint8_t on_time;
	uint8_t off_time;
} blink_tbl[] =
{
	{300, 5,  5},
	{200, 8,  8},
	{100, 11, 11},
	{70,  13, 13},
	{50,  15, 15},
	{20,  17, 17},
	{10,  19, 19},
	{5,   22, 22},
	{1,   26, 26},
	{0,   33, 33},
	/* SOLID_ON */
};

extern void iwl_set_led(struct iwl_softc *, uint8_t, uint8_t, uint8_t,uint8_t);
extern void iwl_led_pattern(struct iwl_softc *);

#endif /* __iwl_led_h__ */
