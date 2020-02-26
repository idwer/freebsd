/* copied from FreeBSD sys/dev/iwx/if_iwx.c */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_wlan.h"
#include "opt_iwx.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/linker.h>

#include <machine/bus.h>
#include <machine/endian.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <net/bpf.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/iwx/if_iwxreg.h>
#include <dev/iwx/if_iwxvar.h>
#include <dev/iwx/if_iwx_config.h>
#include <dev/iwx/if_iwx_debug.h>
#include <dev/iwx/if_iwx_notif_wait.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_binding.h>
#include <dev/iwx/if_iwx_phy_db.h>
#include <dev/iwx/if_iwx_mac_ctxt.h>
#include <dev/iwx/if_iwx_phy_ctxt.h>
#include <dev/iwx/if_iwx_time_event.h>
#include <dev/iwx/if_iwx_power.h>
#include <dev/iwx/if_iwx_scan.h>
#include <dev/iwx/if_iwx_sf.h>
#include <dev/iwx/if_iwx_sta.h>

#include <dev/iwx/if_iwx_pcie_trans.h>
#include <dev/iwx/if_iwx_fw.h>

/*
 * Autoconf glue-sniffing
 */
#define	PCI_VENDOR_INTEL		0x8086
#define	PCI_PRODUCT_INTEL_WL_22500_1	0x2723

static const struct iwx_devices {
	uint16_t		device;
	const struct iwx_cfg	*cfg;
} iwx_devices[] = {
	{ PCI_PRODUCT_INTEL_WL_22500_1, &iwx22500_cfg },
};

static int
iwx_probe(device_t dev)
{
	int i;

	for (i = 0; i < nitems(iwx_devices); i++) {
		if (pci_get_vendor(dev) == PCI_VENDOR_INTEL &&
		    pci_get_device(dev) == iwx_devices[i].device) {
			device_set_desc(dev, iwx_devices[i].cfg->name);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static device_method_t iwx_pci_methods[] = {
        /* Device interface */
	DEVMETHOD(device_probe,         iwx_probe),
#if 0
        DEVMETHOD(device_attach,        iwx_attach),
        DEVMETHOD(device_detach,        iwx_detach),
        DEVMETHOD(device_suspend,       iwx_suspend),
        DEVMETHOD(device_resume,        iwx_resume),
#endif

        DEVMETHOD_END
};

static driver_t iwx_pci_driver = {
        "iwx",
        iwx_pci_methods,
        sizeof (struct iwx_softc)
};

static devclass_t iwx_devclass;

DRIVER_MODULE(iwx, pci, iwx_pci_driver, iwx_devclass, NULL, NULL);
MODULE_PNP_INFO("U16:device;P:#;T:vendor=0x8086", pci, iwx_pci_driver,
    iwx_devices, nitems(iwx_devices));
MODULE_DEPEND(iwx, firmware, 1, 1, 1);
MODULE_DEPEND(iwx, pci, 1, 1, 1);
MODULE_DEPEND(iwx, wlan, 1, 1, 1);
