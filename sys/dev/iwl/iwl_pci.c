/*-
 * Copyright (c) 2011 Intel Corporation
 * Copyright (c) 2007-2009
 *	Damien Bergamini <damien.bergamini@free.fr>
 * Copyright (c) 2008
 *	Benjamin Close <benjsc@FreeBSD.org>
 * Copyright (c) 2008 Sam Leffler, Errno Consulting
 *
 * Permission to use, copy, modify, and	distribute this	software for any
 * purpose with	or without fee is hereby granted, provided that	the above
 * copyright notice and	this permission	notice appear in all copies.
 *
 * THE SOFTWARE	IS PROVIDED "AS	IS" AND	THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH	REGARD TO THIS SOFTWARE	INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS.	IN NO EVENT SHALL THE AUTHOR BE	LIABLE FOR
 * ANY SPECIAL,	DIRECT,	INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING	FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT	OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE	OF THIS	SOFTWARE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "iwl_common.h"

static int iwl_pci_probe(device_t);
static int iwl_pci_attach(device_t);
static int iwl_pci_detach(device_t);
static int iwl_pci_shutdown(device_t);
static int iwl_pci_suspend(device_t);
static int iwl_pci_resume(device_t);

struct iwl_ident {
	uint16_t	vendor;
	uint16_t	device;
	const char	*name;
};

static const struct iwl_ident iwl_ident_table[]	= {
	{ 0x8086, 0x0082, "Intel(R) Centrino(R) Advanced-N 6205"},
	{ 0x8086, 0x0085, "Intel(R) Centrino(R) Advanced-N 6205"},
	{ 0x8086, 0x0891, "Intel(R) Centrino(R) Wireless-N 2200"},
	{ 0x8086, 0x08AE, "Intel(R) Centrino(R) Wireless-N 100"},
	{ 0x8086, 0x08AF, "Intel(R) Centrino(R) Wireless-N 100"},
	{ 0x8086, 0x422b, "Intel(R) Centrino(R) Ultimate-N 6300"},
	{ 0x8086, 0x4238, "Intel(R) Centrino(R) Ultimate-N 6300"},
	{ 0, 0,	NULL }
};

static int
iwl_pci_probe(device_t dev)
{
	const struct iwl_ident *ident;

	for (ident = iwl_ident_table; ident->name != NULL; ident++) {
		if (pci_get_vendor(dev) == ident->vendor &&
		    pci_get_device(dev) == ident->device) {
			device_set_desc(dev, ident->name);
			return 0;
		}
	}
	return ENXIO;
}

static int
iwl_pci_attach(device_t dev)
{

	struct iwl_softc *sc = (struct iwl_softc *)device_get_softc(dev);
	uint32_t reg;
	int error, result;

	sc->sc_dev = dev;

	/*
	 * Get the offset of the PCI Express Capability	Structure in PCI
	 * Configuration Space.
	 */
	error = pci_find_cap(dev, PCIY_EXPRESS, &sc->sc_cap_off);
	if (error != 0) {
		device_printf(dev, "PCIe capability structure not found!\n");
		return error;
	}

	/* Clear device-specific "PCI retry timeout" register (41h). */
	pci_write_config(dev, 0x41, 0, 1);

	/* Hardware bug	workaround. */
	reg = pci_read_config(dev, PCIR_COMMAND, 1);
	if (reg & PCIM_CMD_INTxDIS) {
		reg &= ~PCIM_CMD_INTxDIS;
		pci_write_config(dev, PCIR_COMMAND, reg, 1);
	}

	/* Enable bus-mastering. */
	pci_enable_busmaster(dev);

	sc->mem_rid = PCIR_BAR(0);
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "can't map mem space\n");
		error = ENOMEM;
		return error;
	}
	sc->sc_st = rman_get_bustag(sc->mem);
	sc->sc_sh = rman_get_bushandle(sc->mem);

	sc->irq_rid = 0;
	if ((result = pci_msi_count(dev)) == 1 &&
	    pci_alloc_msi(dev, &result) == 0)
		sc->irq_rid = 1;
	/* Install interrupt handler. */
	sc->irq	= bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE |	RF_SHAREABLE);
	if (sc->irq == NULL) {
		device_printf(dev, "can't map interrupt\n");
		error = ENOMEM;
		goto fail;
	}

	error = iwl_attach(dev); /* chipset specific attach function */
	if (error != 0)
		return error;

	/*
	 * Hook our interrupt after all initialization is complete.
	 */
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, iwl_intr, sc, &sc->sc_ih);
	if (error != 0) {
		device_printf(dev, "can't establish interrupt, error %d\n",
		    error);
		goto fail;
	}

	return 0;

fail:
	iwl_detach(dev);
	return error;

}

static int
iwl_pci_detach(device_t dev)
{
	struct iwl_softc *sc = device_get_softc(dev);

	iwl_detach(dev); /* chipset specific detach function */

	if (sc->irq != NULL) {
		bus_teardown_intr(dev, sc->irq, sc->sc_ih);
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
		if (sc->irq_rid == 1)
			pci_release_msi(dev);
	}

	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem);

	return 0;
}

static int
iwl_pci_shutdown(device_t dev)
{
	iwl_shutdown(dev); /* chipset specific shutdown	function */

	return 0;
}

static int
iwl_pci_suspend(device_t dev)
{
	iwl_suspend(dev);  /* chipset specific suspend function	*/

	return 0;
}

static int
iwl_pci_resume(device_t dev)
{
	iwl_resume(dev); /* chipset specific resume function */

	return 0;
}

static device_method_t iwl_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		iwl_pci_probe),
	DEVMETHOD(device_attach,	iwl_pci_attach),
	DEVMETHOD(device_detach,	iwl_pci_detach),
	DEVMETHOD(device_shutdown,	iwl_pci_shutdown),
	DEVMETHOD(device_suspend,	iwl_pci_suspend),
	DEVMETHOD(device_resume,	iwl_pci_resume),
	{ 0, 0 }
};

static driver_t iwl_driver = {
	"iwl",
	iwl_pci_methods,
	sizeof (struct iwl_softc)
};

static devclass_t iwl_devclass;

DRIVER_MODULE(iwl, pci, iwl_driver, iwl_devclass, 0, 0);
MODULE_DEPEND(iwl, pci, 1, 1, 1);
MODULE_DEPEND(iwl, firmware, 1, 1, 1);
MODULE_DEPEND(iwl, wlan, 1, 1, 1);
