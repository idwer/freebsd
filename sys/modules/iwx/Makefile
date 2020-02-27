# $FreeBSD$

.PATH:  ${SRCTOP}/sys/dev/iwx

KMOD=	if_iwx
# Main driver
SRCS=	if_iwx.c if_iwx_binding.c if_iwx_fw.c if_iwx_mac_ctxt.c
SRCS+=	if_iwx_notif_wait.c if_iwx_phy_ctxt.c if_iwx_phy_db.c
SRCS+=	if_iwx_power.c if_iwx_scan.c if_iwx_sf.c if_iwx_sta.c
SRCS+=	if_iwx_time_event.c if_iwx_util.c
# bus layer
SRCS+=	if_iwx_pcie_trans.c
SRCS+=	device_if.h bus_if.h pci_if.h opt_wlan.h opt_iwx.h

CFLAGS+= -DIWX_DEBUG

.include <bsd.kmod.mk>
