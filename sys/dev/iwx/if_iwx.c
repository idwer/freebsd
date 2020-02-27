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

const uint8_t iwx_nvm_channels[] = {
	/* 2.4 GHz */
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	/* 5 GHz */
	36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92,
	96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
	149, 153, 157, 161, 165, 169, 173, 177, 181
};
_Static_assert(nitems(iwx_nvm_channels) <= IWX_NUM_CHANNELS,
    "IWX_NUM_CHANNELS is too small");

#define IWX_N_HW_ADDR_MASK	0xF

#define IWX_UCODE_ALIVE_TIMEOUT	hz
#define IWX_UCODE_CALIB_TIMEOUT	(2*hz)

struct iwx_nvm_section {
	uint16_t length;
	uint8_t *data;
};

struct iwx_alive_data {
	int valid;
	uint32_t scd_base_addr;
};

// static int	iwm_store_cscheme(struct iwm_softc *, const uint8_t *, size_t);
// static int	iwm_firmware_store_section(struct iwm_softc *,
//                                            enum iwm_ucode_type,
//                                            const uint8_t *, size_t);
// static int	iwm_set_default_calib(struct iwm_softc *, const void *);
// static void	iwm_fw_info_free(struct iwm_fw_info *);
// static int	iwm_read_firmware(struct iwm_softc *);
// static int	iwm_alloc_fwmem(struct iwm_softc *);
// static int	iwm_alloc_sched(struct iwm_softc *);
// static int	iwm_alloc_kw(struct iwm_softc *);
// static int	iwm_alloc_ict(struct iwm_softc *);
// static int	iwm_alloc_rx_ring(struct iwm_softc *, struct iwm_rx_ring *);
static void	iwx_reset_rx_ring(struct iwx_softc *, struct iwx_rx_ring *);
// static void	iwm_free_rx_ring(struct iwm_softc *, struct iwm_rx_ring *);
// static int	iwm_alloc_tx_ring(struct iwm_softc *, struct iwm_tx_ring *,
//                                   int);
static void	iwx_reset_tx_ring(struct iwx_softc *, struct iwx_tx_ring *);
// static void	iwm_free_tx_ring(struct iwm_softc *, struct iwm_tx_ring *);
static void	iwx_enable_interrupts(struct iwx_softc *);
static void	iwx_restore_interrupts(struct iwx_softc *);
static void	iwx_disable_interrupts(struct iwx_softc *);
static void	iwx_ict_reset(struct iwx_softc *);
// static int	iwm_allow_mcast(struct ieee80211vap *, struct iwm_softc *);
static void	iwx_stop_device(struct iwx_softc *);
static void	iwx_nic_config(struct iwx_softc *);
// static int	iwm_nic_rx_init(struct iwm_softc *);
// static int	iwm_nic_tx_init(struct iwm_softc *);
// static int	iwm_nic_init(struct iwm_softc *);
static int	iwx_trans_pcie_fw_alive(struct iwx_softc *, uint32_t);
static int	iwx_nvm_read_chunk(struct iwx_softc *, uint16_t, uint16_t,
                                    uint16_t, uint8_t *, uint16_t *);
static int	iwx_nvm_read_section(struct iwx_softc *, uint16_t, uint8_t *,
		uint16_t *, uint32_t);
// static uint32_t	iwm_eeprom_channel_flags(uint16_t);
// static void	iwm_add_channel_band(struct iwm_softc *,
// 		    struct ieee80211_channel[], int, int *, int, size_t,
// 		    const uint8_t[]);
// static void	iwm_init_channel_map(struct ieee80211com *, int, int *,
// 		    struct ieee80211_channel[]);
static struct iwx_nvm_data *
	iwx_parse_nvm_data(struct iwx_softc *, const uint16_t *,
			   const uint16_t *, const uint16_t *,
			   const uint16_t *, const uint16_t *,
			   const uint16_t *);
// static void	iwm_free_nvm_data(struct iwm_nvm_data *);
static void	iwx_set_hw_address_family_8000(struct iwx_softc *,
					       struct iwx_nvm_data *,
					       const uint16_t *,
					       const uint16_t *);
static int	iwx_get_sku(const struct iwx_softc *, const uint16_t *,
			    const uint16_t *);
static int	iwx_get_nvm_version(const struct iwx_softc *, const uint16_t *);
static int	iwx_get_radio_cfg(const struct iwx_softc *, const uint16_t *,
				  const uint16_t *);
static int	iwx_get_n_hw_addrs(const struct iwx_softc *,
				   const uint16_t *);
static void	iwx_set_radio_cfg(const struct iwx_softc *,
				  struct iwx_nvm_data *, uint32_t);
static struct iwx_nvm_data *
	iwx_parse_nvm_sections(struct iwx_softc *, struct iwx_nvm_section *);
static int	iwx_nvm_init(struct iwx_softc *);
static int	iwx_pcie_load_section(struct iwx_softc *, uint8_t,
				      const struct iwx_fw_desc *);
static int	iwx_pcie_load_firmware_chunk(struct iwx_softc *, uint32_t,
					     bus_addr_t, uint32_t);
static int	iwx_pcie_load_cpu_sections_8000(struct iwx_softc *sc,
						const struct iwx_fw_img *,
						int, int *);
static int	iwx_pcie_load_cpu_sections(struct iwx_softc *,
					   const struct iwx_fw_img *,
					   int, int *);
static int	iwx_pcie_load_given_ucode_8000(struct iwx_softc *,
					       const struct iwx_fw_img *);
static int	iwx_pcie_load_given_ucode(struct iwx_softc *,
					  const struct iwx_fw_img *);
static int	iwx_start_fw(struct iwx_softc *, const struct iwx_fw_img *);
static int	iwx_send_tx_ant_cfg(struct iwx_softc *, uint8_t);
static int	iwx_send_phy_cfg_cmd(struct iwx_softc *);
// static int	iwm_load_ucode_wait_alive(struct iwm_softc *,
//                                               enum iwm_ucode_type);
// static int	iwm_run_init_ucode(struct iwm_softc *, int);
static int	iwx_config_ltr(struct iwx_softc *sc);
// static int	iwm_rx_addbuf(struct iwm_softc *, int, int);
// static void	iwm_rx_rx_phy_cmd(struct iwm_softc *,
//                                       struct iwm_rx_packet *);
// static int	iwm_get_noise(struct iwm_softc *,
// 		    const struct iwm_statistics_rx_non_phy *);
// static void	iwm_handle_rx_statistics(struct iwm_softc *,
// 		    struct iwm_rx_packet *);
// static bool	iwm_rx_mpdu(struct iwm_softc *, struct mbuf *,
// 		    uint32_t, bool);
// static int	iwm_rx_tx_cmd_single(struct iwm_softc *,
//                                          struct iwm_rx_packet *,
// 				         struct iwm_node *);
// static void	iwm_rx_tx_cmd(struct iwm_softc *, struct iwm_rx_packet *);
// static void	iwm_cmd_done(struct iwm_softc *, struct iwm_rx_packet *);
// #if 0
// static void	iwm_update_sched(struct iwm_softc *, int, int, uint8_t,
//                                  uint16_t);
// #endif
// static const struct iwm_rate *
// 	iwm_tx_fill_cmd(struct iwm_softc *, struct iwm_node *,
// 			struct mbuf *, struct iwm_tx_cmd *);
// static int	iwm_tx(struct iwm_softc *, struct mbuf *,
//                        struct ieee80211_node *, int);
// static int	iwm_raw_xmit(struct ieee80211_node *, struct mbuf *,
// 			     const struct ieee80211_bpf_params *);
// static int	iwm_update_quotas(struct iwm_softc *, struct iwm_vap *);
// static int	iwm_auth(struct ieee80211vap *, struct iwm_softc *);
// static struct ieee80211_node *
// 		iwm_node_alloc(struct ieee80211vap *,
// 		               const uint8_t[IEEE80211_ADDR_LEN]);
// static uint8_t	iwm_rate_from_ucode_rate(uint32_t);
// static int	iwm_rate2ridx(struct iwm_softc *, uint8_t);
// static void	iwm_setrates(struct iwm_softc *, struct iwm_node *, int);
// static int	iwm_media_change(struct ifnet *);
// static int	iwm_newstate(struct ieee80211vap *, enum ieee80211_state, int);
// static void	iwm_endscan_cb(void *, int);
// static int	iwm_send_bt_init_conf(struct iwm_softc *);
static boolean_t iwx_is_lar_supported(struct iwx_softc *);
static boolean_t iwx_is_wifi_mcc_supported(struct iwx_softc *);
static int	iwx_send_update_mcc_cmd(struct iwx_softc *, const char *);
// static void	iwm_tt_tx_backoff(struct iwm_softc *, uint32_t);
static int	iwx_init_hw(struct iwx_softc *);
// static void	iwm_init(struct iwm_softc *);
// static void	iwm_start(struct iwm_softc *);
static void	iwx_stop(struct iwx_softc *);
// static void	iwm_watchdog(void *);
// static void	iwm_parent(struct ieee80211com *);
// #ifdef IWM_DEBUG
// static const char *
// 		iwm_desc_lookup(uint32_t);
// static void	iwm_nic_error(struct iwm_softc *);
// static void	iwm_nic_umac_error(struct iwm_softc *);
// #endif
// static void	iwm_handle_rxb(struct iwm_softc *, struct mbuf *);
// static void	iwm_notif_intr(struct iwm_softc *);
// static void	iwm_intr(void *);
// static int	iwm_attach(device_t);
static int	iwx_is_valid_ether_addr(uint8_t *);
// static void	iwm_preinit(void *);
// static int	iwm_detach_local(struct iwm_softc *sc, int);
static void	iwx_init_task(void *);
// static void	iwm_radiotap_attach(struct iwm_softc *);
// static struct ieee80211vap *
// 		iwm_vap_create(struct ieee80211com *,
// 		               const char [IFNAMSIZ], int,
// 		               enum ieee80211_opmode, int,
// 		               const uint8_t [IEEE80211_ADDR_LEN],
// 		               const uint8_t [IEEE80211_ADDR_LEN]);
// static void	iwm_vap_delete(struct ieee80211vap *);
// static void	iwm_xmit_queue_drain(struct iwm_softc *);
// static void	iwm_scan_start(struct ieee80211com *);
// static void	iwm_scan_end(struct ieee80211com *);
// static void	iwm_update_mcast(struct ieee80211com *);
// static void	iwm_set_channel(struct ieee80211com *);
// static void	iwm_scan_curchan(struct ieee80211_scan_state *, unsigned long);
// static void	iwm_scan_mindwell(struct ieee80211_scan_state *);
// static int	iwm_detach(device_t);

static int	iwx_lar_disable = 0;
TUNABLE_INT("hw.iwx.lar.disable", &iwx_lar_disable);

static void
iwx_reset_rx_ring(struct iwx_softc *sc, struct iwx_rx_ring *ring)
{
	/* Reset the ring state */
	ring->cur = 0;

	/*
	 * The hw rx ring index in shared memory must also be cleared,
	 * otherwise the discrepancy can cause reprocessing chaos.
	 */
	if (sc->rxq.stat)
		memset(sc->rxq.stat, 0, sizeof(*sc->rxq.stat));
}

static void
iwx_reset_tx_ring(struct iwx_softc *sc, struct iwx_tx_ring *ring)
{
	int i;

	for (i = 0; i < IWX_TX_RING_COUNT; i++) {
		struct iwx_tx_data *data = &ring->data[i];

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

	if (ring->qid == IWX_CMD_QUEUE && sc->cmd_hold_nic_awake)
		iwx_pcie_clear_cmd_in_flight(sc);
}

/*
 * High-level hardware frobbing routines
 */

static void
iwx_enable_interrupts(struct iwx_softc *sc)
{
	sc->sc_intmask = IWX_CSR_INI_SET_MASK;
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
}

static void
iwx_restore_interrupts(struct iwx_softc *sc)
{
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
}

static void
iwx_disable_interrupts(struct iwx_softc *sc)
{
	/* disable interrupts */
	IWX_WRITE(sc, IWX_CSR_INT_MASK, 0);

	/* acknowledge all interrupts */
	IWX_WRITE(sc, IWX_CSR_INT, ~0);
	IWX_WRITE(sc, IWX_CSR_FH_INT_STATUS, ~0);
}

static void
iwx_ict_reset(struct iwx_softc *sc)
{
	iwx_disable_interrupts(sc);

	/* Reset ICT table. */
	memset(sc->ict_dma.vaddr, 0, IWX_ICT_SIZE);
	sc->ict_cur = 0;

	/* Set physical address of ICT table (4KB aligned). */
	IWX_WRITE(sc, IWX_CSR_DRAM_INT_TBL_REG,
	    IWX_CSR_DRAM_INT_TBL_ENABLE
	    | IWX_CSR_DRAM_INIT_TBL_WRITE_POINTER
	    | IWX_CSR_DRAM_INIT_TBL_WRAP_CHECK
	    | sc->ict_dma.paddr >> IWX_ICT_PADDR_SHIFT);

	/* Switch to ICT interrupt mode in driver. */
	sc->sc_flags |= IWX_FLAG_USE_ICT;

	/* Re-enable interrupts. */
	IWX_WRITE(sc, IWX_CSR_INT, ~0);
	iwx_enable_interrupts(sc);
}

/* iwlwifi pcie/trans.c */

/*
 * Since this .. hard-resets things, it's time to actually
 * mark the first vap (if any) as having no mac context.
 * It's annoying, but since the driver is potentially being
 * stop/start'ed whilst active (thanks openbsd port!) we
 * have to correctly track this.
 */
static void
iwx_stop_device(struct iwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	int chnl, qid;
	uint32_t mask = 0;

	/* tell the device to stop sending interrupts */
	iwx_disable_interrupts(sc);

	/*
	 * FreeBSD-local: mark the first vap as not-uploaded,
	 * so the next transition through auth/assoc
	 * will correctly populate the MAC context.
	 */
	if (vap) {
		struct iwx_vap *iv = IWX_VAP(vap);
		iv->phy_ctxt = NULL;
		iv->is_uploaded = 0;
	}
	sc->sc_firmware_state = 0;
	sc->sc_flags &= ~IWX_FLAG_TE_ACTIVE;

	/* device going down, Stop using ICT table */
	sc->sc_flags &= ~IWX_FLAG_USE_ICT;

	/* stop tx and rx.  tx and rx bits, as usual, are from if_iwn */

	if (iwx_nic_lock(sc)) {
		iwx_write_prph(sc, IWX_SCD_TXFACT, 0);

		/* Stop each Tx DMA channel */
		for (chnl = 0; chnl < IWX_FH_TCSR_CHNL_NUM; chnl++) {
			IWX_WRITE(sc,
			    IWX_FH_TCSR_CHNL_TX_CONFIG_REG(chnl), 0);
			mask |= IWX_FH_TSSR_TX_STATUS_REG_MSK_CHNL_IDLE(chnl);
		}

		/* Wait for DMA channels to be idle */
		if (!iwx_poll_bit(sc, IWX_FH_TSSR_TX_STATUS_REG, mask, mask,
		    5000)) {
			device_printf(sc->sc_dev,
			    "Failing on timeout while stopping DMA channel: [0x%08x]\n",
			    IWX_READ(sc, IWX_FH_TSSR_TX_STATUS_REG));
		}
		iwx_nic_unlock(sc);
	}
	iwx_pcie_rx_stop(sc);

	/* Stop RX ring. */
	iwx_reset_rx_ring(sc, &sc->rxq);

	/* Reset all TX rings. */
	for (qid = 0; qid < nitems(sc->txq); qid++)
		iwx_reset_tx_ring(sc, &sc->txq[qid]);

#if 0
	if (sc->cfg->device_family == IWM_DEVICE_FAMILY_7000) {
		/* Power-down device's busmaster DMA clocks */
		if (iwm_nic_lock(sc)) {
			iwm_write_prph(sc, IWM_APMG_CLK_DIS_REG,
			    IWM_APMG_CLK_VAL_DMA_CLK_RQT);
			iwm_nic_unlock(sc);
		}
		DELAY(5);
	}
#endif

	/* Make sure (redundant) we've released our request to stay awake */
	IWX_CLRBITS(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	/* Stop the device, and put it in low power state */
	iwx_apm_stop(sc);

	/* stop and reset the on-board processor */
	IWX_SETBITS(sc, IWX_CSR_RESET, IWX_CSR_RESET_REG_FLAG_SW_RESET);
	DELAY(5000);

	/*
	 * Upon stop, the APM issues an interrupt if HW RF kill is set.
	 */
	iwx_disable_interrupts(sc);

	/*
	 * Even if we stop the HW, we still want the RF kill
	 * interrupt
	 */
	iwx_enable_rfkill_int(sc);
	iwx_check_rfkill(sc);

	iwx_prepare_card_hw(sc);
}

/* iwlwifi: mvm/ops.c */
static void
iwx_nic_config(struct iwx_softc *sc)
{
	uint8_t radio_cfg_type, radio_cfg_step, radio_cfg_dash;
	uint32_t reg_val = 0;
	uint32_t phy_config = iwx_get_phy_config(sc);

	radio_cfg_type = (phy_config & IWX_FW_PHY_CFG_RADIO_TYPE) >>
	    IWX_FW_PHY_CFG_RADIO_TYPE_POS;
	radio_cfg_step = (phy_config & IWX_FW_PHY_CFG_RADIO_STEP) >>
	    IWX_FW_PHY_CFG_RADIO_STEP_POS;
	radio_cfg_dash = (phy_config & IWX_FW_PHY_CFG_RADIO_DASH) >>
	    IWX_FW_PHY_CFG_RADIO_DASH_POS;

	/* SKU control */
	reg_val |= IWX_CSR_HW_REV_STEP(sc->sc_hw_rev) <<
	    IWX_CSR_HW_IF_CONFIG_REG_POS_MAC_STEP;
	reg_val |= IWX_CSR_HW_REV_DASH(sc->sc_hw_rev) <<
	    IWX_CSR_HW_IF_CONFIG_REG_POS_MAC_DASH;

	/* radio configuration */
	reg_val |= radio_cfg_type << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_TYPE;
	reg_val |= radio_cfg_step << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_STEP;
	reg_val |= radio_cfg_dash << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_DASH;

	IWX_WRITE(sc, IWX_CSR_HW_IF_CONFIG_REG,
			IWX_CSR_HW_IF_CONFIG_REG_MSK_MAC_DASH |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_MAC_STEP |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_PHY_STEP |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_PHY_DASH |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_PHY_TYPE |
			IWX_CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI |
			IWX_CSR_HW_IF_CONFIG_REG_BIT_MAC_SI |
	    reg_val);

	IWX_DPRINTF(sc, IWX_DEBUG_RESET,
	    "Radio type=0x%x-0x%x-0x%x\n", radio_cfg_type,
	    radio_cfg_step, radio_cfg_dash);

#if 0
	/*
	 * W/A : NIC is stuck in a reset state after Early PCIe power off
	 * (PCIe power is lost before PERST# is asserted), causing ME FW
	 * to lose ownership and not being able to obtain it back.
	 */
	if (sc->cfg->device_family == IWM_DEVICE_FAMILY_7000) {
		iwm_set_bits_mask_prph(sc, IWM_APMG_PS_CTRL_REG,
		    IWM_APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS,
		    ~IWM_APMG_PS_CTRL_EARLY_PWR_OFF_RESET_DIS);
	}
#endif
}

static int
iwx_nic_rx_mq_init(struct iwx_softc *sc)
{
	int enabled;

	if (!iwx_nic_lock(sc))
		return EBUSY;

	/* Stop RX DMA. */
	iwx_write_prph(sc, IWX_RFH_RXF_DMA_CFG, 0);
	/* Disable RX used and free queue operation. */
	iwx_write_prph(sc, IWX_RFH_RXF_RXQ_ACTIVE, 0);

	iwx_write_prph64(sc, IWX_RFH_Q0_FRBDCB_BA_LSB,
	    sc->rxq.free_desc_dma.paddr);
	iwx_write_prph64(sc, IWX_RFH_Q0_URBDCB_BA_LSB,
	    sc->rxq.used_desc_dma.paddr);
	iwx_write_prph64(sc, IWX_RFH_Q0_URBD_STTS_WPTR_LSB,
	    sc->rxq.stat_dma.paddr);
	iwx_write_prph(sc, IWX_RFH_Q0_FRBDCB_WIDX, 0);
	iwx_write_prph(sc, IWX_RFH_Q0_FRBDCB_RIDX, 0);
	iwx_write_prph(sc, IWX_RFH_Q0_URBDCB_WIDX, 0);

	/* We configure only queue 0 for now. */
	enabled = ((1 << 0) << 16) | (1 << 0);

	/* Enable RX DMA, 4KB buffer size. */
	iwx_write_prph(sc, IWX_RFH_RXF_DMA_CFG,
	    IWX_RFH_DMA_EN_ENABLE_VAL |
	    IWX_RFH_RXF_DMA_RB_SIZE_4K |
	    IWX_RFH_RXF_DMA_MIN_RB_4_8 |
	    IWM_RFH_RXF_DMA_DROP_TOO_LARGE_MASK |
	    IWM_RFH_RXF_DMA_RBDCB_SIZE_512);

	/* Enable RX DMA snooping. */
	iwx_write_prph(sc, IWM_RFH_GEN_CFG,
	    IWM_RFH_GEN_CFG_RFH_DMA_SNOOP |
	    IWM_RFH_GEN_CFG_SERVICE_DMA_SNOOP |
	    (sc->cfg->integrated ? IWM_RFH_GEN_CFG_RB_CHUNK_SIZE_64 :
	    IWM_RFH_GEN_CFG_RB_CHUNK_SIZE_128));

	/* Enable the configured queue(s). */
	iwx_write_prph(sc, IWX_RFH_RXF_RXQ_ACTIVE, enabled);

	iwx_nic_unlock(sc);

	IWX_WRITE_1(sc, IWX_CSR_INT_COALESCING, IWX_HOST_INT_TIMEOUT_DEF);

	IWX_WRITE(sc, IWX_RFH_Q0_FRBDCB_WIDX_TRG, 8);

	return (0);
}

static int
iwx_nic_rx_legacy_init(struct iwx_softc *sc)
{

	/* Stop Rx DMA */
	iwx_pcie_rx_stop(sc);

	if (!iwx_nic_lock(sc))
		return EBUSY;

	/* reset and flush pointers */
	IWX_WRITE(sc, IWX_FH_MEM_RCSR_CHNL0_RBDCB_WPTR, 0);
	IWX_WRITE(sc, IWX_FH_MEM_RCSR_CHNL0_FLUSH_RB_REQ, 0);
	IWX_WRITE(sc, IWX_FH_RSCSR_CHNL0_RDPTR, 0);
	IWX_WRITE(sc, IWX_FH_RSCSR_CHNL0_RBDCB_WPTR_REG, 0);

	/* Set physical address of RX ring (256-byte aligned). */
	IWX_WRITE(sc,
	    IWX_FH_RSCSR_CHNL0_RBDCB_BASE_REG,
	    sc->rxq.free_desc_dma.paddr >> 8);

	/* Set physical address of RX status (16-byte aligned). */
	IWX_WRITE(sc,
	    IWX_FH_RSCSR_CHNL0_STTS_WPTR_REG, sc->rxq.stat_dma.paddr >> 4);

	/* Enable Rx DMA
	 * XXX 5000 HW isn't supported by the iwm(4) driver.
	 * IWM_FH_RCSR_CHNL0_RX_IGNORE_RXF_EMPTY is set because of HW bug in
	 *      the credit mechanism in 5000 HW RX FIFO
	 * Direct rx interrupts to hosts
	 * Rx buffer size 4 or 8k or 12k
	 * RB timeout 0x10
	 * 256 RBDs
	 */
	IWX_WRITE(sc, IWX_FH_MEM_RCSR_CHNL0_CONFIG_REG,
	    IWX_FH_RCSR_RX_CONFIG_CHNL_EN_ENABLE_VAL		|
	    IWX_FH_RCSR_CHNL0_RX_IGNORE_RXF_EMPTY		|  /* HW bug */
	    IWX_FH_RCSR_CHNL0_RX_CONFIG_IRQ_DEST_INT_HOST_VAL	|
	    IWX_FH_RCSR_RX_CONFIG_REG_VAL_RB_SIZE_4K		|
	    (IWX_RX_RB_TIMEOUT << IWX_FH_RCSR_RX_CONFIG_REG_IRQ_RBTH_POS) |
	    IWX_RX_QUEUE_SIZE_LOG << IWX_FH_RCSR_RX_CONFIG_RBDCB_SIZE_POS);

	IWX_WRITE_1(sc, IWX_CSR_INT_COALESCING, IWX_HOST_INT_TIMEOUT_DEF);

#if 0
	/* W/A for interrupt coalescing bug in 7260 and 3160 */
	if (sc->cfg->host_interrupt_operation_mode)
		IWM_SETBITS(sc, IWM_CSR_INT_COALESCING, IWM_HOST_INT_OPER_MODE);
#endif

	iwx_nic_unlock(sc);

	IWX_WRITE(sc, IWX_FH_RSCSR_CHNL0_WPTR, 8);

	return 0;
}

static int
iwx_nic_rx_init(struct iwx_softc *sc)
{
	if (sc->cfg->mqrx_supported)
		return iwx_nic_rx_mq_init(sc);
	else
		return iwx_nic_rx_legacy_init(sc);
}

static int
iwx_nic_tx_init(struct iwx_softc *sc)
{
	int qid;

	if (!iwx_nic_lock(sc))
		return EBUSY;

	/* Deactivate TX scheduler. */
	iwx_write_prph(sc, IWX_SCD_TXFACT, 0);

	/* Set physical address of "keep warm" page (16-byte aligned). */
	IWX_WRITE(sc, IWX_FH_KW_MEM_ADDR_REG, sc->kw_dma.paddr >> 4);

	/* Initialize TX rings. */
	for (qid = 0; qid < nitems(sc->txq); qid++) {
		struct iwx_tx_ring *txq = &sc->txq[qid];

		/* Set physical address of TX ring (256-byte aligned). */
		IWX_WRITE(sc, IWX_FH_MEM_CBBC_QUEUE(qid),
		    txq->desc_dma.paddr >> 8);
		IWX_DPRINTF(sc, IWX_DEBUG_XMIT,
		    "%s: loading ring %d descriptors (%p) at %lx\n",
		    __func__,
		    qid, txq->desc,
		    (unsigned long) (txq->desc_dma.paddr >> 8));
	}

	iwx_set_bits_prph(sc, IWX_SCD_GP_CTRL,
	    IWX_SCD_GP_CTRL_AUTO_ACTIVE_MODE |
	    IWX_SCD_GP_CTRL_ENABLE_31_QUEUES);

	iwx_nic_unlock(sc);

	return 0;
}

static int
iwx_nic_init(struct iwx_softc *sc)
{
	int error;

	iwx_apm_init(sc);
#if 0
	if (sc->cfg->device_family == IWM_DEVICE_FAMILY_7000)
		iwm_set_pwr(sc);
#endif

	iwx_nic_config(sc);

	if ((error = iwx_nic_rx_init(sc)) != 0)
		return error;

	/*
	 * Ditto for TX, from iwn
	 */
	if ((error = iwx_nic_tx_init(sc)) != 0)
		return error;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET,
	    "%s: shadow registers enabled\n", __func__);
	IWX_SETBITS(sc, IWX_CSR_MAC_SHADOW_REG_CTRL, 0x800fffff);

	return 0;
}

int
iwx_enable_txq(struct iwx_softc *sc, int sta_id, int qid, int fifo)
{
	int qmsk;

	qmsk = 1 << qid;

	if (!iwx_nic_lock(sc)) {
		device_printf(sc->sc_dev, "%s: cannot enable txq %d\n",
		    __func__, qid);
		return EBUSY;
	}

	IWX_WRITE(sc, IWX_HBUS_TARG_WRPTR, qid << 8 | 0);

	if (qid == IWX_CMD_QUEUE) {
		/* Disable the scheduler. */
		iwx_write_prph(sc, IWX_SCD_EN_CTRL, 0);

		/* Stop the TX queue prior to configuration. */
		iwx_write_prph(sc, IWX_SCD_QUEUE_STATUS_BITS(qid),
		    (0 << IWX_SCD_QUEUE_STTS_REG_POS_ACTIVE) |
		    (1 << IWX_SCD_QUEUE_STTS_REG_POS_SCD_ACT_EN));

		iwx_nic_unlock(sc);

		/* Disable aggregations for this queue. */
		iwx_clear_bits_prph(sc, IWX_SCD_AGGR_SEL, qmsk);

		if (!iwx_nic_lock(sc)) {
			device_printf(sc->sc_dev,
			    "%s: cannot enable txq %d\n", __func__, qid);
			return EBUSY;
		}
		iwx_write_prph(sc, IWX_SCD_QUEUE_RDPTR(qid), 0);
		iwx_nic_unlock(sc);

		iwx_write_mem32(sc,
		    sc->scd_base_addr + IWX_SCD_CONTEXT_QUEUE_OFFSET(qid), 0);
		/* Set scheduler window size and frame limit. */
		iwx_write_mem32(sc,
		    sc->scd_base_addr + IWX_SCD_CONTEXT_QUEUE_OFFSET(qid) +
		    sizeof(uint32_t),
		    ((IWX_FRAME_LIMIT << IWX_SCD_QUEUE_CTX_REG2_WIN_SIZE_POS) &
		    IWX_SCD_QUEUE_CTX_REG2_WIN_SIZE_MSK) |
		    ((IWX_FRAME_LIMIT << IWX_SCD_QUEUE_CTX_REG2_FRAME_LIMIT_POS) &
		    IWX_SCD_QUEUE_CTX_REG2_FRAME_LIMIT_MSK));

		if (!iwx_nic_lock(sc)) {
			device_printf(sc->sc_dev,
			    "%s: cannot enable txq %d\n", __func__, qid);
			return EBUSY;
		}
		iwx_write_prph(sc, IWX_SCD_QUEUE_STATUS_BITS(qid),
		    (1 << IWX_SCD_QUEUE_STTS_REG_POS_ACTIVE) |
		    (fifo << IWX_SCD_QUEUE_STTS_REG_POS_TXF) |
		    (1 << IWX_SCD_QUEUE_STTS_REG_POS_WSL) |
		    IWX_SCD_QUEUE_STTS_REG_MSK);

		/* Enable the scheduler for this queue. */
		iwx_write_prph(sc, IWX_SCD_EN_CTRL, qmsk);
	} else {
		struct iwx_scd_txq_cfg_cmd cmd;
		int error;

		iwx_nic_unlock(sc);

		memset(&cmd, 0, sizeof(cmd));
		cmd.scd_queue = qid;
		cmd.enable = 1;
		cmd.sta_id = sta_id;
		cmd.tx_fifo = fifo;
		cmd.aggregate = 0;
		cmd.window = IWX_FRAME_LIMIT;

		error = iwx_send_cmd_pdu(sc, IWX_SCD_QUEUE_CFG, IWX_CMD_SYNC,
		    sizeof(cmd), &cmd);
		if (error) {
			device_printf(sc->sc_dev,
			    "cannot enable txq %d\n", qid);
			return error;
		}

		if (!iwx_nic_lock(sc))
			return EBUSY;
	}

	iwx_nic_unlock(sc);

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: enabled txq %d FIFO %d\n",
	    __func__, qid, fifo);

	return 0;
}

static int
iwx_trans_pcie_fw_alive(struct iwx_softc *sc, uint32_t scd_base_addr)
{
	int error, chnl;

	int clear_dwords = (IWX_SCD_TRANS_TBL_MEM_UPPER_BOUND -
	    IWX_SCD_CONTEXT_MEM_LOWER_BOUND) / sizeof(uint32_t);

	if (!iwx_nic_lock(sc))
		return EBUSY;

	iwx_ict_reset(sc);

	sc->scd_base_addr = iwx_read_prph(sc, IWX_SCD_SRAM_BASE_ADDR);
	if (scd_base_addr != 0 &&
	    scd_base_addr != sc->scd_base_addr) {
		device_printf(sc->sc_dev,
		    "%s: sched addr mismatch: alive: 0x%x prph: 0x%x\n",
		    __func__, sc->scd_base_addr, scd_base_addr);
	}

	iwx_nic_unlock(sc);

	/* reset context data, TX status and translation data */
	error = iwx_write_mem(sc,
	    sc->scd_base_addr + IWX_SCD_CONTEXT_MEM_LOWER_BOUND,
	    NULL, clear_dwords);
	if (error)
		return EBUSY;

	if (!iwx_nic_lock(sc))
		return EBUSY;

	/* Set physical address of TX scheduler rings (1KB aligned). */
	iwx_write_prph(sc, IWX_SCD_DRAM_BASE_ADDR, sc->sched_dma.paddr >> 10);

	iwx_write_prph(sc, IWX_SCD_CHAINEXT_EN, 0);

	iwx_nic_unlock(sc);

	/* enable command channel */
	error = iwx_enable_txq(sc, 0 /* unused */, IWX_CMD_QUEUE, 7);
	if (error)
		return error;

	if (!iwx_nic_lock(sc))
		return EBUSY;

	iwx_write_prph(sc, IWX_SCD_TXFACT, 0xff);

	/* Enable DMA channels. */
	for (chnl = 0; chnl < IWX_FH_TCSR_CHNL_NUM; chnl++) {
		IWX_WRITE(sc, IWX_FH_TCSR_CHNL_TX_CONFIG_REG(chnl),
		    IWX_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE |
		    IWX_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_ENABLE);
	}

	IWX_SETBITS(sc, IWX_FH_TX_CHICKEN_BITS_REG,
	    IWX_FH_TX_CHICKEN_BITS_SCD_AUTO_RETRY_EN);

	iwx_nic_unlock(sc);

#if 0
	/* Enable L1-Active */
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000) {
		iwm_clear_bits_prph(sc, IWM_APMG_PCIDEV_STT_REG,
		    IWM_APMG_PCIDEV_STT_VAL_L1_ACT_DIS);
	}
#endif

	return error;
}

/*
 * NVM read access and content parsing.  We do not support
 * external NVM or writing NVM.
 * iwlwifi/mvm/nvm.c
 */

/* Default NVM size to read */
#define IWX_NVM_DEFAULT_CHUNK_SIZE	(2*1024)

#define IWX_NVM_READ_OPCODE 0

/* load nvm chunk response */
enum {
	IWM_READ_NVM_CHUNK_SUCCEED = 0,
	IWX_READ_NVM_CHUNK_NOT_VALID_ADDRESS = 1
};

static int
iwx_nvm_read_chunk(struct iwx_softc *sc, uint16_t section,
	uint16_t offset, uint16_t length, uint8_t *data, uint16_t *len)
{
	struct iwx_nvm_access_cmd nvm_access_cmd = {
		.offset = htole16(offset),
		.length = htole16(length),
		.type = htole16(section),
		.op_code = IWX_NVM_READ_OPCODE,
	};
	struct iwx_nvm_access_resp *nvm_resp;
	struct iwx_rx_packet *pkt;
	struct iwx_host_cmd cmd = {
		.id = IWX_NVM_ACCESS_CMD,
		.flags = IWX_CMD_WANT_SKB | IWX_CMD_SEND_IN_RFKILL,
		.data = { &nvm_access_cmd, },
	};
	int ret, bytes_read, offset_read;
	uint8_t *resp_data;

	cmd.len[0] = sizeof(struct iwx_nvm_access_cmd);

	ret = iwx_send_cmd(sc, &cmd);
	if (ret) {
		device_printf(sc->sc_dev,
		    "Could not send NVM_ACCESS command (error=%d)\n", ret);
		return ret;
	}

	pkt = cmd.resp_pkt;

	/* Extract NVM response */
	nvm_resp = (void *)pkt->data;
	ret = le16toh(nvm_resp->status);
	bytes_read = le16toh(nvm_resp->length);
	offset_read = le16toh(nvm_resp->offset);
	resp_data = nvm_resp->data;
	if (ret) {
		if ((offset != 0) &&
		    (ret == IWX_READ_NVM_CHUNK_NOT_VALID_ADDRESS)) {
			/*
			 * meaning of NOT_VALID_ADDRESS:
			 * driver try to read chunk from address that is
			 * multiple of 2K and got an error since addr is empty.
			 * meaning of (offset != 0): driver already
			 * read valid data from another chunk so this case
			 * is not an error.
			 */
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
				    "NVM access command failed on offset 0x%x since that section size is multiple 2K\n",
				    offset);
			*len = 0;
			ret = 0;
		} else {
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
				    "NVM access command failed with status %d\n", ret);
			ret = EIO;
		}
		goto exit;
	}

	if (offset_read != offset) {
		device_printf(sc->sc_dev,
		    "NVM ACCESS response with invalid offset %d\n",
		    offset_read);
		ret = EINVAL;
		goto exit;
	}

	if (bytes_read > length) {
		device_printf(sc->sc_dev,
		    "NVM ACCESS response with too much data "
		    "(%d bytes requested, %d bytes received)\n",
		    length, bytes_read);
		ret = EINVAL;
		goto exit;
	}

	/* Write data to NVM */
	memcpy(data + offset, resp_data, bytes_read);
	*len = bytes_read;

 exit:
	iwx_free_resp(sc, &cmd);
	return ret;
}


/*
 * Reads an NVM section completely.
 * NICs prior to 7000 family don't have a real NVM, but just read
 * section 0 which is the EEPROM. Because the EEPROM reading is unlimited
 * by uCode, we need to manually check in this case that we don't
 * overflow and try to read more than the EEPROM size.
 * For 7000 family NICs, we supply the maximal size we can read, and
 * the uCode fills the response with as much data as we can,
 * without overflowing, so no check is needed.
 */
static int
iwx_nvm_read_section(struct iwx_softc *sc,
	uint16_t section, uint8_t *data, uint16_t *len, uint32_t size_read)
{
	uint16_t seglen, length, offset = 0;
	int ret;

	/* Set nvm section read length */
	length = IWX_NVM_DEFAULT_CHUNK_SIZE;

	seglen = length;

	/* Read the NVM until exhausted (reading less than requested) */
	while (seglen == length) {
		/* Check no memory assumptions fail and cause an overflow */
		if ((size_read + offset + length) >
		    sc->cfg->eeprom_size) {
			device_printf(sc->sc_dev,
			    "EEPROM size is too small for NVM\n");
			return ENOBUFS;
		}

		ret = iwx_nvm_read_chunk(sc, section, offset, length, data, &seglen);
		if (ret) {
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
				    "Cannot read NVM from section %d offset %d, length %d\n",
				    section, offset, length);
			return ret;
		}
		offset += seglen;
	}

	IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
		    "NVM section %d read completed\n", section);
	*len = offset;
	return 0;
}

/*
 * BEGIN IWM_NVM_PARSE
 */

/* iwlwifi/iwl-nvm-parse.c */

/* NVM offsets (in words) definitions */
enum iwx_nvm_offsets {
	/* NVM HW-Section offset (in words) definitions */
	IWM_HW_ADDR = 0x15,

/* NVM SW-Section offset (in words) definitions */
	IWM_NVM_SW_SECTION = 0x1C0,
	IWM_NVM_VERSION = 0,
	IWM_RADIO_CFG = 1,
	IWX_SKU = 2,
	IWX_N_HW_ADDRS = 3,
//	IWM_NVM_CHANNELS = 0x1E0 - IWM_NVM_SW_SECTION,

/* NVM calibration section offset (in words) definitions */
	IWM_NVM_CALIB_SECTION = 0x2B8,
	IWM_XTAL_CALIB = 0x316 - IWM_NVM_CALIB_SECTION
};

enum iwx_8000_nvm_offsets {
	/* NVM HW-Section offset (in words) definitions */
	IWM_HW_ADDR0_WFPM_8000 = 0x12,
	IWM_HW_ADDR1_WFPM_8000 = 0x16,
	IWM_HW_ADDR0_PCIE_8000 = 0x8A,
	IWM_HW_ADDR1_PCIE_8000 = 0x8E,
	IWX_MAC_ADDRESS_OVERRIDE_8000 = 1,

	/* NVM SW-Section offset (in words) definitions */
//	IWM_NVM_SW_SECTION_8000 = 0x1C0,
	IWM_NVM_VERSION_8000 = 0,
	IWX_RADIO_CFG_8000 = 0,
//	IWM_SKU_8000 = 2,
//	IWX_N_HW_ADDRS_8000 = 3,

	/* NVM REGULATORY -Section offset (in words) definitions */
	IWX_NVM_CHANNELS = 0,
	IWX_NVM_LAR_OFFSET_8000_OLD = 0x4C7,
	IWX_NVM_LAR_OFFSET_8000 = 0x507,
	IWX_NVM_LAR_ENABLED_8000 = 0x7,

	/* NVM calibration section offset (in words) definitions */
	IWM_NVM_CALIB_SECTION_8000 = 0x2B8,
	IWM_XTAL_CALIB_8000 = 0x316 - IWM_NVM_CALIB_SECTION_8000
};

/* SKU Capabilities (actual values from NVM definition) */
enum nvm_sku_bits {
	IWX_NVM_SKU_CAP_BAND_24GHZ	= (1 << 0),
	IWX_NVM_SKU_CAP_BAND_52GHZ	= (1 << 1),
	IWM_NVM_SKU_CAP_11N_ENABLE	= (1 << 2),
	IWM_NVM_SKU_CAP_11AC_ENABLE	= (1 << 3),
};

/* radio config bits (actual values from NVM definition) */
#if 0
#define IWM_NVM_RF_CFG_DASH_MSK(x)   (x & 0x3)         /* bits 0-1   */
#define IWM_NVM_RF_CFG_STEP_MSK(x)   ((x >> 2)  & 0x3) /* bits 2-3   */
#define IWM_NVM_RF_CFG_TYPE_MSK(x)   ((x >> 4)  & 0x3) /* bits 4-5   */
#define IWM_NVM_RF_CFG_PNUM_MSK(x)   ((x >> 6)  & 0x3) /* bits 6-7   */
#define IWM_NVM_RF_CFG_TX_ANT_MSK(x) ((x >> 8)  & 0xF) /* bits 8-11  */
#define IWM_NVM_RF_CFG_RX_ANT_MSK(x) ((x >> 12) & 0xF) /* bits 12-15 */
#endif

//#define IWM_NVM_RF_CFG_FLAVOR_MSK_8000(x)	(x & 0xF)
//#define IWM_NVM_RF_CFG_DASH_MSK_8000(x)		((x >> 4) & 0xF)
//#define IWX_NVM_RF_CFG_STEP_MSK_8000(x)		((x >> 8) & 0xF)
//#define IWM_NVM_RF_CFG_TYPE_MSK_8000(x)		((x >> 12) & 0xFFF)
//#define IWX_NVM_RF_CFG_TX_ANT_MSK_8000(x)	((x >> 24) & 0xF)
//#define IWX_NVM_RF_CFG_RX_ANT_MSK_8000(x)	((x >> 28) & 0xF)
#define IWX_NVM_RF_CFG_FLAVOR_MSK(x) (x & 0xF)
#define IWX_NVM_RF_CFG_DASH_MSK(x)   ((x >> 4) & 0xF)
#define IWX_NVM_RF_CFG_STEP_MSK(x)   ((x >> 8) & 0xF)
#define IWX_NVM_RF_CFG_TYPE_MSK(x)   ((x >> 12) & 0xFFF)
#define IWX_NVM_RF_CFG_TX_ANT_MSK(x) ((x >> 24) & 0xF)
#define IWX_NVM_RF_CFG_RX_ANT_MSK(x) ((x >> 28) & 0xF)

/**
 * enum iwm_nvm_channel_flags - channel flags in NVM
 * @IWM_NVM_CHANNEL_VALID: channel is usable for this SKU/geo
 * @IWM_NVM_CHANNEL_IBSS: usable as an IBSS channel
 * @IWM_NVM_CHANNEL_ACTIVE: active scanning allowed
 * @IWM_NVM_CHANNEL_RADAR: radar detection required
 * XXX cannot find this (DFS) flag in iwm-nvm-parse.c
 * @IWM_NVM_CHANNEL_DFS: dynamic freq selection candidate
 * @IWM_NVM_CHANNEL_WIDE: 20 MHz channel okay (?)
 * @IWM_NVM_CHANNEL_40MHZ: 40 MHz channel okay (?)
 * @IWM_NVM_CHANNEL_80MHZ: 80 MHz channel okay (?)
 * @IWM_NVM_CHANNEL_160MHZ: 160 MHz channel okay (?)
 */
enum iwm_nvm_channel_flags {
	IWM_NVM_CHANNEL_VALID = (1 << 0),
	IWM_NVM_CHANNEL_IBSS = (1 << 1),
	IWM_NVM_CHANNEL_ACTIVE = (1 << 3),
	IWM_NVM_CHANNEL_RADAR = (1 << 4),
	IWM_NVM_CHANNEL_DFS = (1 << 7),
	IWM_NVM_CHANNEL_WIDE = (1 << 8),
	IWM_NVM_CHANNEL_40MHZ = (1 << 9),
	IWM_NVM_CHANNEL_80MHZ = (1 << 10),
	IWM_NVM_CHANNEL_160MHZ = (1 << 11),
};

/* iwlwifi: mvm/fw.c */
static int
iwx_send_phy_cfg_cmd(struct iwx_softc *sc)
{
	struct iwx_phy_cfg_cmd phy_cfg_cmd;
	enum iwx_ucode_type ucode_type = sc->cur_ucode;

	/* Set parameters */
	phy_cfg_cmd.phy_cfg = htole32(iwx_get_phy_config(sc));
	phy_cfg_cmd.calib_control.event_trigger =
	    sc->sc_default_calib[ucode_type].event_trigger;
	phy_cfg_cmd.calib_control.flow_trigger =
	    sc->sc_default_calib[ucode_type].flow_trigger;

	IWX_DPRINTF(sc, IWX_DEBUG_CMD | IWX_DEBUG_RESET,
	    "Sending Phy CFG command: 0x%x\n", phy_cfg_cmd.phy_cfg);
	return iwx_send_cmd_pdu(sc, IWX_PHY_CONFIGURATION_CMD, IWX_CMD_SYNC,
	    sizeof(phy_cfg_cmd), &phy_cfg_cmd);
}

static int
iwx_alive_fn(struct iwx_softc *sc, struct iwx_rx_packet *pkt, void *data)
{
	struct iwx_alive_data *alive_data = data;
	struct iwx_alive_resp_v3 *palive3;
	struct iwx_alive_resp *palive;
	struct iwx_umac_alive *umac;
	struct iwx_lmac_alive *lmac1;
	struct iwx_lmac_alive *lmac2 = NULL;
	uint16_t status;

	if (iwx_rx_packet_payload_len(pkt) == sizeof(*palive)) {
		palive = (void *)pkt->data;
		umac = &palive->umac_data;
		lmac1 = &palive->lmac_data[0];
		lmac2 = &palive->lmac_data[1];
		status = le16toh(palive->status);
	} else {
		palive3 = (void *)pkt->data;
		umac = &palive3->umac_data;
		lmac1 = &palive3->lmac_data;
		status = le16toh(palive3->status);
	}

	sc->error_event_table[0] = le32toh(lmac1->error_event_table_ptr);
	if (lmac2)
		sc->error_event_table[1] =
			le32toh(lmac2->error_event_table_ptr);
	sc->log_event_table = le32toh(lmac1->log_event_table_ptr);
	sc->umac_error_event_table = le32toh(umac->error_info_addr);
	alive_data->scd_base_addr = le32toh(lmac1->scd_base_ptr);
	alive_data->valid = status == IWX_ALIVE_STATUS_OK;
	if (sc->umac_error_event_table)
		sc->support_umac_log = TRUE;

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "Alive ucode status 0x%04x revision 0x%01X 0x%01X\n",
		    status, lmac1->ver_type, lmac1->ver_subtype);

	if (lmac2)
		IWX_DPRINTF(sc, IWX_DEBUG_FW, "Alive ucode CDB\n");

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "UMAC version: Major - 0x%x, Minor - 0x%x\n",
		    le32toh(umac->umac_major),
		    le32toh(umac->umac_minor));

	return TRUE;
}

static int
iwx_wait_phy_db_entry(struct iwx_softc *sc,
	struct iwx_rx_packet *pkt, void *data)
{
	struct iwx_phy_db *phy_db = data;

	if (pkt->hdr.code != IWX_CALIB_RES_NOTIF_PHY_DB) {
		if(pkt->hdr.code != IWX_INIT_COMPLETE_NOTIF) {
			device_printf(sc->sc_dev, "%s: Unexpected cmd: %d\n",
			    __func__, pkt->hdr.code);
		}
		return TRUE;
	}

	if (iwx_phy_db_set_section(phy_db, pkt)) {
		device_printf(sc->sc_dev,
		    "%s: iwx_phy_db_set_section failed\n", __func__);
	}

	return FALSE;
}

static int
iwx_load_ucode_wait_alive(struct iwx_softc *sc,
	enum iwx_ucode_type ucode_type)
{
	struct iwx_notification_wait alive_wait;
	struct iwx_alive_data alive_data;
	const struct iwx_fw_img *fw;
	enum iwx_ucode_type old_type = sc->cur_ucode;
	int error;
	static const uint16_t alive_cmd[] = { IWM_ALIVE };

	fw = &sc->sc_fw.img[ucode_type];
	sc->cur_ucode = ucode_type;
	sc->ucode_loaded = FALSE;

	memset(&alive_data, 0, sizeof(alive_data));
	iwx_init_notification_wait(sc->sc_notif_wait, &alive_wait,
				   alive_cmd, nitems(alive_cmd),
				   iwx_alive_fn, &alive_data);

	error = iwx_start_fw(sc, fw);
	if (error) {
		device_printf(sc->sc_dev, "iwx_start_fw: failed %d\n", error);
		sc->cur_ucode = old_type;
		iwx_remove_notification(sc->sc_notif_wait, &alive_wait);
		return error;
	}

	/*
	 * Some things may run in the background now, but we
	 * just wait for the ALIVE notification here.
	 */
	IWX_UNLOCK(sc);
	error = iwx_wait_notification(sc->sc_notif_wait, &alive_wait,
				      IWX_UCODE_ALIVE_TIMEOUT);
	IWX_LOCK(sc);
	if (error) {
#if 0
		if (sc->cfg->device_family >= IWM_DEVICE_FAMILY_8000) {
			uint32_t a = 0x5a5a5a5a, b = 0x5a5a5a5a;
			if (iwm_nic_lock(sc)) {
				a = iwm_read_prph(sc, IWM_SB_CPU_1_STATUS);
				b = iwm_read_prph(sc, IWM_SB_CPU_2_STATUS);
				iwm_nic_unlock(sc);
			}
			device_printf(sc->sc_dev,
			    "SecBoot CPU1 Status: 0x%x, CPU2 Status: 0x%x\n",
			    a, b);
		}
#endif
		sc->cur_ucode = old_type;
		return error;
	}

	if (!alive_data.valid) {
		device_printf(sc->sc_dev, "%s: Loaded ucode is not valid\n",
		    __func__);
		sc->cur_ucode = old_type;
		return EIO;
	}

	iwx_trans_pcie_fw_alive(sc, alive_data.scd_base_addr);

	/*
	 * configure and operate fw paging mechanism.
	 * driver configures the paging flow only once, CPU2 paging image
	 * included in the IWM_UCODE_INIT image.
	 */
	if (fw->paging_mem_size) {
		error = iwx_save_fw_paging(sc, fw);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: failed to save the FW paging image\n",
			    __func__);
			return error;
		}

		error = iwx_send_paging_cmd(sc, fw);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: failed to send the paging cmd\n", __func__);
			iwx_free_fw_paging(sc);
			return error;
		}
	}

	if (!error)
		sc->ucode_loaded = TRUE;
	return error;
}

/*
 * mvm misc bits
 */

/*
 * follows iwlwifi/fw.c
 */
static int
iwx_run_init_ucode(struct iwx_softc *sc, int justnvm)
{
	struct iwx_notification_wait calib_wait;
	static const uint16_t init_complete[] = {
		IWX_INIT_COMPLETE_NOTIF,
		IWX_CALIB_RES_NOTIF_PHY_DB
	};
	int ret;

	/* do not operate with rfkill switch turned on */
	if ((sc->sc_flags & IWX_FLAG_RFKILL) && !justnvm) {
		device_printf(sc->sc_dev,
		    "radio is disabled by hardware switch\n");
		return EPERM;
	}

	iwx_init_notification_wait(sc->sc_notif_wait,
				   &calib_wait,
				   init_complete,
				   nitems(init_complete),
				   iwx_wait_phy_db_entry,
				   sc->sc_phy_db);

	/* Will also start the device */
	ret = iwx_load_ucode_wait_alive(sc, IWX_UCODE_INIT);
	if (ret) {
		device_printf(sc->sc_dev, "Failed to start INIT ucode: %d\n",
		    ret);
		goto error;
	}

#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000) {
		ret = iwm_send_bt_init_conf(sc);
		if (ret) {
			device_printf(sc->sc_dev,
			    "failed to send bt coex configuration: %d\n", ret);
			goto error;
		}
	}
#endif

	if (justnvm) {
		/* Read nvm */
		ret = iwx_nvm_init(sc);
		if (ret) {
			device_printf(sc->sc_dev, "failed to read nvm\n");
			goto error;
		}
		IEEE80211_ADDR_COPY(sc->sc_ic.ic_macaddr, sc->nvm_data->hw_addr);
		goto error;
	}

	/* Send TX valid antennas before triggering calibrations */
	ret = iwx_send_tx_ant_cfg(sc, iwx_get_valid_tx_ant(sc));
	if (ret) {
		device_printf(sc->sc_dev,
		    "failed to send antennas before calibration: %d\n", ret);
		goto error;
	}

	/*
	 * Send phy configurations command to init uCode
	 * to start the 16.0 uCode init image internal calibrations.
	 */
	ret = iwx_send_phy_cfg_cmd(sc);
	if (ret) {
		device_printf(sc->sc_dev,
		    "%s: Failed to run INIT calibrations: %d\n",
		    __func__, ret);
		goto error;
	}

	/*
	 * Nothing to do but wait for the init complete notification
	 * from the firmware.
	 */
	IWX_UNLOCK(sc);
	ret = iwx_wait_notification(sc->sc_notif_wait, &calib_wait,
	    IWX_UCODE_CALIB_TIMEOUT);
	IWX_LOCK(sc);


	goto out;

error:
	iwx_remove_notification(sc->sc_notif_wait, &calib_wait);
out:
	return ret;
}

static int
iwx_config_ltr(struct iwx_softc *sc)
{
	struct iwx_ltr_config_cmd cmd = {
		.flags = htole32(IWX_LTR_CFG_FLAG_FEATURE_ENABLE),
	};

	if (!sc->sc_ltr_enabled)
		return 0;

	return iwx_send_cmd_pdu(sc, IWX_LTR_CONFIG, 0, sizeof(cmd), &cmd);
}

static int
iwx_set_hw_address(struct iwx_softc *sc, struct iwx_nvm_data *data,
		   const uint16_t *nvm_hw, const uint16_t *mac_override)
{
#ifdef notyet /* for FAMILY 9000 */
	if (cfg->mac_addr_from_csr) {
		iwm_set_hw_address_from_csr(sc, data);
	} else
#endif
#if 0
		if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000) {
		const uint8_t *hw_addr = (const uint8_t *)(nvm_hw + IWM_HW_ADDR);

		/* The byte order is little endian 16 bit, meaning 214365 */
		data->hw_addr[0] = hw_addr[1];
		data->hw_addr[1] = hw_addr[0];
		data->hw_addr[2] = hw_addr[3];
		data->hw_addr[3] = hw_addr[2];
		data->hw_addr[4] = hw_addr[5];
		data->hw_addr[5] = hw_addr[4];
	} else {
#endif
		iwx_set_hw_address_family_8000(sc, data, mac_override, nvm_hw);
//	}

	if (!iwx_is_valid_ether_addr(data->hw_addr)) {
		device_printf(sc->sc_dev, "no valid mac address was found\n");
		return EINVAL;
	}

	return 0;
}

static struct iwx_nvm_data *
iwx_parse_nvm_data(struct iwx_softc *sc,
		   const uint16_t *nvm_hw, const uint16_t *nvm_sw,
		   const uint16_t *nvm_calib, const uint16_t *mac_override,
		   const uint16_t *phy_sku, const uint16_t *regulatory)
{
	struct iwx_nvm_data *data;
	uint32_t sku, radio_cfg;
//	uint16_t lar_config;

#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000) {
		data = malloc(sizeof(*data) +
		    IWM_NUM_CHANNELS * sizeof(uint16_t),
		    M_DEVBUF, M_NOWAIT | M_ZERO);
	} else {
#endif
		data = malloc(sizeof(*data) +
		    IWX_NUM_CHANNELS * sizeof(uint16_t),
		    M_DEVBUF, M_NOWAIT | M_ZERO);
//	}
	if (!data)
		return NULL;

	data->nvm_version = iwx_get_nvm_version(sc, nvm_sw);

	radio_cfg = iwx_get_radio_cfg(sc, nvm_sw, phy_sku);
	iwx_set_radio_cfg(sc, data, radio_cfg);

	sku = iwx_get_sku(sc, nvm_sw, phy_sku);
	data->sku_cap_band_24GHz_enable = sku & IWX_NVM_SKU_CAP_BAND_24GHZ;
	data->sku_cap_band_52GHz_enable = sku & IWX_NVM_SKU_CAP_BAND_52GHZ;
	data->sku_cap_11n_enable = 0;

	data->n_hw_addrs = iwx_get_n_hw_addrs(sc, nvm_sw);

	/* todo: tune to match/support family 22000 */
#if 0
	if (sc->cfg->device_family >= IWX_DEVICE_FAMILY_8000) {
		/* TODO: use IWL_NVM_EXT */
		uint16_t lar_offset = data->nvm_version < 0xE39 ?
				       IWX_NVM_LAR_OFFSET_8000_OLD :
				       IWX_NVM_LAR_OFFSET_8000;

		lar_config = le16_to_cpup(regulatory + lar_offset);
		data->lar_enabled = !!(lar_config &
				       IWX_NVM_LAR_ENABLED_8000);
	}
#endif

	/* If no valid mac address was found - bail out */
	if (iwx_set_hw_address(sc, data, nvm_hw, mac_override)) {
		free(data, M_DEVBUF);
		return NULL;
	}

#if 0
	if (sc->cfg->device_family == IWM_DEVICE_FAMILY_7000) {
		memcpy(data->nvm_ch_flags, sc->cfg->nvm_type == IWM_NVM_SDP ?
		    &regulatory[0] : &nvm_sw[IWM_NVM_CHANNELS],
		    IWM_NUM_CHANNELS * sizeof(uint16_t));
	} else {
#endif
		memcpy(data->nvm_ch_flags, &regulatory[IWX_NVM_CHANNELS],
		    IWX_NUM_CHANNELS * sizeof(uint16_t));
//	}

	return data;
}

static void
iwx_set_hw_address_family_8000(struct iwx_softc *sc, struct iwx_nvm_data *data,
	const uint16_t *mac_override, const uint16_t *nvm_hw)
{
	const uint8_t *hw_addr;

	if (mac_override) {
		static const uint8_t reserved_mac[] = {
			0x02, 0xcc, 0xaa, 0xff, 0xee, 0x00
		};

		hw_addr = (const uint8_t *)(mac_override +
				 IWX_MAC_ADDRESS_OVERRIDE_8000);

		/*
		 * Store the MAC address from MAO section.
		 * No byte swapping is required in MAO section
		 */
		IEEE80211_ADDR_COPY(data->hw_addr, hw_addr);

		/*
		 * Force the use of the OTP MAC address in case of reserved MAC
		 * address in the NVM, or if address is given but invalid.
		 */
		if (!IEEE80211_ADDR_EQ(reserved_mac, hw_addr) &&
		    !IEEE80211_ADDR_EQ(ieee80211broadcastaddr, data->hw_addr) &&
		    iwx_is_valid_ether_addr(data->hw_addr) &&
		    !IEEE80211_IS_MULTICAST(data->hw_addr))
			return;

		IWX_DPRINTF(sc, IWX_DEBUG_RESET,
		    "%s: mac address from nvm override section invalid\n",
		    __func__);
	}

	if (nvm_hw) {
		/* read the mac address from WFMP registers */
		uint32_t mac_addr0 =
		    htole32(iwx_read_prph(sc, IWX_WFMP_MAC_ADDR_0));
		uint32_t mac_addr1 =
		    htole32(iwx_read_prph(sc, IWX_WFMP_MAC_ADDR_1));

		hw_addr = (const uint8_t *)&mac_addr0;
		data->hw_addr[0] = hw_addr[3];
		data->hw_addr[1] = hw_addr[2];
		data->hw_addr[2] = hw_addr[1];
		data->hw_addr[3] = hw_addr[0];

		hw_addr = (const uint8_t *)&mac_addr1;
		data->hw_addr[4] = hw_addr[1];
		data->hw_addr[5] = hw_addr[0];

		return;
	}

	device_printf(sc->sc_dev, "%s: mac address not found\n", __func__);
	memset(data->hw_addr, 0, sizeof(data->hw_addr));
}

static int
iwx_get_sku(const struct iwx_softc *sc, const uint16_t *nvm_sw,
	    const uint16_t *phy_sku)
{
#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000)
		return le16_to_cpup(nvm_sw + IWM_SKU);
#endif
	/* todo: tune to match/support family 22000 */
#if 0
	return le32_to_cpup((const uint32_t *)(phy_sku + IWM_SKU_8000));
#endif
	return le32_to_cpup((const uint32_t *)(phy_sku + IWX_SKU));
}

static int
iwx_get_nvm_version(const struct iwx_softc *sc, const uint16_t *nvm_sw)
{
#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000)
		return le16_to_cpup(nvm_sw + IWM_NVM_VERSION);
#endif
//	else
	/* todo: tune to match/support family 22000 */

		return le32_to_cpup((const uint32_t *)(nvm_sw +
						IWM_NVM_VERSION_8000));
}

static int
iwx_get_radio_cfg(const struct iwx_softc *sc, const uint16_t *nvm_sw,
		  const uint16_t *phy_sku)
{
#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000)
                return le16_to_cpup(nvm_sw + IWM_RADIO_CFG);
#endif
	/* todo: tune to match/support family 22000 */

        return le32_to_cpup((const uint32_t *)(phy_sku + IWX_RADIO_CFG_8000));
}

static int
iwx_get_n_hw_addrs(const struct iwx_softc *sc, const uint16_t *nvm_sw)
{
	int n_hw_addr;

#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000)
		return le16_to_cpup(nvm_sw + IWM_N_HW_ADDRS);
#endif
	/* todo: tune to match/support family 22000 */
	// n_hw_addr = le32_to_cpup((const uint32_t *)(nvm_sw + IWX_N_HW_ADDRS_8000));
	n_hw_addr = le32_to_cpup((const uint32_t *)(nvm_sw + IWX_N_HW_ADDRS));
        return n_hw_addr & IWX_N_HW_ADDR_MASK;
}

static void
iwx_set_radio_cfg(const struct iwx_softc *sc, struct iwx_nvm_data *data,
		  uint32_t radio_cfg)
{
#if 0
	if (sc->cfg->device_family < IWM_DEVICE_FAMILY_8000) {
		data->radio_cfg_type = IWM_NVM_RF_CFG_TYPE_MSK(radio_cfg);
		data->radio_cfg_step = IWM_NVM_RF_CFG_STEP_MSK(radio_cfg);
		data->radio_cfg_dash = IWM_NVM_RF_CFG_DASH_MSK(radio_cfg);
		data->radio_cfg_pnum = IWM_NVM_RF_CFG_PNUM_MSK(radio_cfg);
		return;
	}
#endif
	/* todo: tune to match/support family 22000 */
	/* set the radio configuration for family 8000 */
//	data->radio_cfg_type = IWX_NVM_RF_CFG_TYPE_MSK_8000(radio_cfg);
//	data->radio_cfg_step = IWX_NVM_RF_CFG_STEP_MSK_8000(radio_cfg);
//	data->radio_cfg_dash = IWX_NVM_RF_CFG_DASH_MSK_8000(radio_cfg);
//	data->radio_cfg_pnum = IWX_NVM_RF_CFG_FLAVOR_MSK_8000(radio_cfg);
//	data->valid_tx_ant = IWX_NVM_RF_CFG_TX_ANT_MSK_8000(radio_cfg);
//	data->valid_rx_ant = IWX_NVM_RF_CFG_RX_ANT_MSK_8000(radio_cfg);
	data->radio_cfg_type = IWX_NVM_RF_CFG_TYPE_MSK(radio_cfg);
	data->radio_cfg_step = IWX_NVM_RF_CFG_STEP_MSK(radio_cfg);
	data->radio_cfg_dash = IWX_NVM_RF_CFG_DASH_MSK(radio_cfg);
	data->radio_cfg_pnum = IWX_NVM_RF_CFG_FLAVOR_MSK(radio_cfg);
	data->valid_tx_ant = IWX_NVM_RF_CFG_TX_ANT_MSK(radio_cfg);
	data->valid_rx_ant = IWX_NVM_RF_CFG_RX_ANT_MSK(radio_cfg);
}

static struct iwx_nvm_data *
iwx_parse_nvm_sections(struct iwx_softc *sc, struct iwx_nvm_section *sections)
{
	const uint16_t *hw, *sw, *calib, *regulatory, *mac_override, *phy_sku;

#if 0
	/* todo: tune to match/support family 22000 */
	/* Checking for required sections */
	if (sc->cfg->device_family == IWM_DEVICE_FAMILY_7000) {
		if (!sections[IWM_NVM_SECTION_TYPE_SW].data ||
		    !sections[sc->cfg->nvm_hw_section_num].data) {
			device_printf(sc->sc_dev,
			    "Can't parse empty OTP/NVM sections\n");
			return NULL;
		}
	} else
		if (sc->cfg->device_family >= IWM_DEVICE_FAMILY_8000) {
		/* SW and REGULATORY sections are mandatory */
		if (!sections[IWM_NVM_SECTION_TYPE_SW].data ||
		    !sections[IWM_NVM_SECTION_TYPE_REGULATORY].data) {
			device_printf(sc->sc_dev,
			    "Can't parse empty OTP/NVM sections\n");
			return NULL;
		}
		/* MAC_OVERRIDE or at least HW section must exist */
		if (!sections[sc->cfg->nvm_hw_section_num].data &&
		    !sections[IWM_NVM_SECTION_TYPE_MAC_OVERRIDE].data) {
			device_printf(sc->sc_dev,
			    "Can't parse mac_address, empty sections\n");
			return NULL;
		}

		/* PHY_SKU section is mandatory in B0 */
		if (!sections[IWM_NVM_SECTION_TYPE_PHY_SKU].data) {
			device_printf(sc->sc_dev,
			    "Can't parse phy_sku in B0, empty sections\n");
			return NULL;
		}
	} else {
		panic("unknown device family %d\n", sc->cfg->device_family);
	}
#endif

	hw = (const uint16_t *) sections[sc->cfg->nvm_hw_section_num].data;
	sw = (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_SW].data;
	calib = (const uint16_t *)
	    sections[IWX_NVM_SECTION_TYPE_CALIBRATION].data;
#if 0
	regulatory = sc->cfg->nvm_type == IWX_NVM_SDP ?
	    (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_REGULATORY_SDP].data :
	    (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_REGULATORY].data;
#endif
	regulatory =
	    (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_REGULATORY].data;

	mac_override = (const uint16_t *)
	    sections[IWX_NVM_SECTION_TYPE_MAC_OVERRIDE].data;
	phy_sku = (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_PHY_SKU].data;

	return iwx_parse_nvm_data(sc, hw, sw, calib, mac_override,
	    phy_sku, regulatory);
}

static int
iwx_nvm_init(struct iwx_softc *sc)
{
	struct iwx_nvm_section nvm_sections[IWX_NVM_MAX_NUM_SECTIONS];
	int i, ret, section;
	uint32_t size_read = 0;
	uint8_t *nvm_buffer, *temp;
	uint16_t len;

	memset(nvm_sections, 0, sizeof(nvm_sections));

	if (sc->cfg->nvm_hw_section_num >= IWX_NVM_MAX_NUM_SECTIONS)
		return EINVAL;

	/* load NVM values from nic */
	/* Read From FW NVM */
	IWX_DPRINTF(sc, IWX_DEBUG_EEPROM, "Read from NVM\n");

	nvm_buffer = malloc(sc->cfg->eeprom_size, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (!nvm_buffer)
		return ENOMEM;
	for (section = 0; section < IWX_NVM_MAX_NUM_SECTIONS; section++) {
		/* we override the constness for initial read */
		ret = iwx_nvm_read_section(sc, section, nvm_buffer,
					   &len, size_read);
		if (ret)
			continue;
		size_read += len;
		temp = malloc(len, M_DEVBUF, M_NOWAIT);
		if (!temp) {
			ret = ENOMEM;
			break;
		}
		memcpy(temp, nvm_buffer, len);

		nvm_sections[section].data = temp;
		nvm_sections[section].length = len;
	}
	if (!size_read)
		device_printf(sc->sc_dev, "OTP is blank\n");
	free(nvm_buffer, M_DEVBUF);

	sc->nvm_data = iwx_parse_nvm_sections(sc, nvm_sections);
	if (!sc->nvm_data)
		return EINVAL;
	IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
		    "nvm version = %x\n", sc->nvm_data->nvm_version);

	for (i = 0; i < IWX_NVM_MAX_NUM_SECTIONS; i++) {
		if (nvm_sections[i].data != NULL)
			free(nvm_sections[i].data, M_DEVBUF);
	}

	return 0;
}

static int
iwx_pcie_load_section(struct iwx_softc *sc, uint8_t section_num,
	const struct iwx_fw_desc *section)
{
	struct iwx_dma_info *dma = &sc->fw_dma;
	uint8_t *v_addr;
	bus_addr_t p_addr;
	uint32_t offset, chunk_sz = MIN(IWX_FH_MEM_TB_MAX_LENGTH, section->len);
	int ret = 0;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET,
		    "%s: [%d] uCode section being loaded...\n",
		    __func__, section_num);

	v_addr = dma->vaddr;
	p_addr = dma->paddr;

	for (offset = 0; offset < section->len; offset += chunk_sz) {
		uint32_t copy_size, dst_addr;
		int extended_addr = FALSE;

		copy_size = MIN(chunk_sz, section->len - offset);
		dst_addr = section->offset + offset;

		if (dst_addr >= IWX_FW_MEM_EXTENDED_START &&
		    dst_addr <= IWX_FW_MEM_EXTENDED_END)
			extended_addr = TRUE;

		if (extended_addr)
			iwx_set_bits_prph(sc, IWX_LMPM_CHICK,
					  IWX_LMPM_CHICK_EXTENDED_ADDR_SPACE);

		memcpy(v_addr, (const uint8_t *)section->data + offset,
		    copy_size);
		bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);
		ret = iwx_pcie_load_firmware_chunk(sc, dst_addr, p_addr,
						   copy_size);

		if (extended_addr)
			iwx_clear_bits_prph(sc, IWX_LMPM_CHICK,
					    IWX_LMPM_CHICK_EXTENDED_ADDR_SPACE);

		if (ret) {
			device_printf(sc->sc_dev,
			    "%s: Could not load the [%d] uCode section\n",
			    __func__, section_num);
			break;
		}
	}

	return ret;
}

/*
 * ucode
 */
static int
iwx_pcie_load_firmware_chunk(struct iwx_softc *sc, uint32_t dst_addr,
			     bus_addr_t phy_addr, uint32_t byte_cnt)
{
	sc->sc_fw_chunk_done = 0;

	if (!iwx_nic_lock(sc))
		return EBUSY;

	IWX_WRITE(sc, IWX_FH_TCSR_CHNL_TX_CONFIG_REG(IWX_FH_SRVC_CHNL),
	    IWX_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_PAUSE);

	IWX_WRITE(sc, IWX_FH_SRVC_CHNL_SRAM_ADDR_REG(IWX_FH_SRVC_CHNL),
	    dst_addr);

	IWX_WRITE(sc, IWX_FH_TFDIB_CTRL0_REG(IWX_FH_SRVC_CHNL),
	    phy_addr & IWX_FH_MEM_TFDIB_DRAM_ADDR_LSB_MSK);

	IWX_WRITE(sc, IWX_FH_TFDIB_CTRL1_REG(IWX_FH_SRVC_CHNL),
	    (iwm_get_dma_hi_addr(phy_addr)
	     << IWX_FH_MEM_TFDIB_REG1_ADDR_BITSHIFT) | byte_cnt);

	IWX_WRITE(sc, IWX_FH_TCSR_CHNL_TX_BUF_STS_REG(IWX_FH_SRVC_CHNL),
	    1 << IWX_FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_NUM |
	    1 << IWX_FH_TCSR_CHNL_TX_BUF_STS_REG_POS_TB_IDX |
	    IWX_FH_TCSR_CHNL_TX_BUF_STS_REG_VAL_TFDB_VALID);

	IWX_WRITE(sc, IWX_FH_TCSR_CHNL_TX_CONFIG_REG(IWX_FH_SRVC_CHNL),
	    IWX_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CHNL_ENABLE    |
	    IWX_FH_TCSR_TX_CONFIG_REG_VAL_DMA_CREDIT_DISABLE |
	    IWX_FH_TCSR_TX_CONFIG_REG_VAL_CIRQ_HOST_ENDTFD);

	iwx_nic_unlock(sc);

	/* wait up to 5s for this segment to load */
	msleep(&sc->sc_fw, &sc->sc_mtx, 0, "iwxfw", hz * 5);

	if (!sc->sc_fw_chunk_done) {
		device_printf(sc->sc_dev,
		    "fw chunk addr 0x%x len %d failed to load\n",
		    dst_addr, byte_cnt);
		return ETIMEDOUT;
	}

	return 0;
}

static int
iwx_pcie_load_cpu_sections_8000(struct iwx_softc *sc,
	const struct iwx_fw_img *image, int cpu, int *first_ucode_section)
{
	int shift_param;
	int i, ret = 0, sec_num = 0x1;
	uint32_t val, last_read_idx = 0;

	if (cpu == 1) {
		shift_param = 0;
		*first_ucode_section = 0;
	} else {
		shift_param = 16;
		(*first_ucode_section)++;
	}

	for (i = *first_ucode_section; i < IWX_UCODE_SECTION_MAX; i++) {
		last_read_idx = i;

		/*
		 * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between
		 * CPU1 to CPU2.
		 * PAGING_SEPARATOR_SECTION delimiter - separate between
		 * CPU2 non paged to CPU2 paging sec.
		 */
		if (!image->sec[i].data ||
		    image->sec[i].offset == IWX_CPU1_CPU2_SEPARATOR_SECTION ||
		    image->sec[i].offset == IWX_PAGING_SEPARATOR_SECTION) {
			IWX_DPRINTF(sc, IWX_DEBUG_RESET,
				    "Break since Data not valid or Empty section, sec = %d\n",
				    i);
			break;
		}
		ret = iwx_pcie_load_section(sc, i, &image->sec[i]);
		if (ret)
			return ret;

		/* Notify the ucode of the loaded section number and status */
		if (iwx_nic_lock(sc)) {
			val = IWX_READ(sc, IWX_FH_UCODE_LOAD_STATUS);
			val = val | (sec_num << shift_param);
			IWX_WRITE(sc, IWX_FH_UCODE_LOAD_STATUS, val);
			sec_num = (sec_num << 1) | 0x1;
			iwx_nic_unlock(sc);
		}
	}

	*first_ucode_section = last_read_idx;

	iwx_enable_interrupts(sc);

	if (iwx_nic_lock(sc)) {
		if (cpu == 1)
			IWX_WRITE(sc, IWX_FH_UCODE_LOAD_STATUS, 0xFFFF);
		else
			IWX_WRITE(sc, IWX_FH_UCODE_LOAD_STATUS, 0xFFFFFFFF);
		iwx_nic_unlock(sc);
	}

	return 0;
}

/* todo:
 * it's probably saner to merge iwx_pcie_load_cpu_sections()
 * with iwx_pcie_load_cpu_sections_8000(), and drop the latter */
static int
iwx_pcie_load_cpu_sections(struct iwx_softc *sc,
	const struct iwx_fw_img *image, int cpu, int *first_ucode_section)
{
	int shift_param;
	int i, ret = 0;
	uint32_t last_read_idx = 0;

	if (cpu == 1) {
		shift_param = 0;
		*first_ucode_section = 0;
	} else {
		shift_param = 16;
		(*first_ucode_section)++;
	}

	for (i = *first_ucode_section; i < IWX_UCODE_SECTION_MAX; i++) {
		last_read_idx = i;

		/*
		 * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between
		 * CPU1 to CPU2.
		 * PAGING_SEPARATOR_SECTION delimiter - separate between
		 * CPU2 non paged to CPU2 paging sec.
		 */
		if (!image->sec[i].data ||
		    image->sec[i].offset == IWX_CPU1_CPU2_SEPARATOR_SECTION ||
		    image->sec[i].offset == IWX_PAGING_SEPARATOR_SECTION) {
			IWX_DPRINTF(sc, IWX_DEBUG_RESET,
				    "Break since Data not valid or Empty section, sec = %d\n",
				     i);
			break;
		}

		ret = iwx_pcie_load_section(sc, i, &image->sec[i]);
		if (ret)
			return ret;
	}

	*first_ucode_section = last_read_idx;

	return 0;

}

int
iwx_pcie_load_given_ucode_8000(struct iwx_softc *sc,
	const struct iwx_fw_img *image)
{
	int ret = 0;
	int first_ucode_section;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET, "working with %s CPU\n",
		    image->is_dual_cpus ? "Dual" : "Single");

	/* configure the ucode to be ready to get the secured image */
	/* release CPU reset */
	if (iwx_nic_lock(sc)) {
		iwx_write_prph(sc, IWX_RELEASE_CPU_RESET,
		    IWX_RELEASE_CPU_RESET_BIT);
		iwx_nic_unlock(sc);
	}

	/* load to FW the binary Secured sections of CPU1 */
	ret = iwx_pcie_load_cpu_sections_8000(sc, image, 1,
	    &first_ucode_section);
	if (ret)
		return ret;

	/* load to FW the binary sections of CPU2 */
	return iwx_pcie_load_cpu_sections_8000(sc, image, 2,
	    &first_ucode_section);
}

/* todo:
 * it's probably saner to merge iwx_pcie_load_given_ucode()
 * with iwx_pcie_load_given_ucode_8000(), and drop the latter */
static int
iwx_pcie_load_given_ucode(struct iwx_softc *sc, const struct iwx_fw_img *image)
{
	int ret = 0;
	int first_ucode_section;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET, "working with %s CPU\n",
		     image->is_dual_cpus ? "Dual" : "Single");

	/* load to FW the binary non secured sections of CPU1 */
	ret = iwx_pcie_load_cpu_sections(sc, image, 1, &first_ucode_section);
	if (ret)
		return ret;

	if (image->is_dual_cpus) {
		/* set CPU2 header address */
		if (iwx_nic_lock(sc)) {
			iwx_write_prph(sc,
				       IWX_LMPM_SECURE_UCODE_LOAD_CPU2_HDR_ADDR,
				       IWX_LMPM_SECURE_CPU2_HDR_MEM_SPACE);
			iwx_nic_unlock(sc);
		}

		/* load to FW the binary sections of CPU2 */
		ret = iwx_pcie_load_cpu_sections(sc, image, 2,
						 &first_ucode_section);
		if (ret)
			return ret;
	}

	iwx_enable_interrupts(sc);

	/* release CPU reset */
	IWX_WRITE(sc, IWX_CSR_RESET, 0);

	return 0;
}

/* XXX Get rid of this definition */
static inline void
iwx_enable_fw_load_int(struct iwx_softc *sc)
{
	IWX_DPRINTF(sc, IWX_DEBUG_INTR, "Enabling FW load interrupt\n");
	sc->sc_intmask = IWX_CSR_INT_BIT_FH_TX;
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
}

/* XXX Add proper rfkill support code */

static int
iwx_start_fw(struct iwx_softc *sc, const struct iwx_fw_img *fw)
{
	int ret;

	/* This may fail if AMT took ownership of the device */
	if (iwx_prepare_card_hw(sc)) {
		device_printf(sc->sc_dev,
		    "%s: Exit HW not ready\n", __func__);
		ret = EIO;
		goto out;
	}

	IWX_WRITE(sc, IWX_CSR_INT, 0xFFFFFFFF);

	iwx_disable_interrupts(sc);

	/* make sure rfkill handshake bits are cleared */
	IWX_WRITE(sc, IWX_CSR_UCODE_DRV_GP1_CLR, IWX_CSR_UCODE_SW_BIT_RFKILL);
	IWX_WRITE(sc, IWX_CSR_UCODE_DRV_GP1_CLR,
	    IWX_CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);

	/* clear (again), then enable host interrupts */
	IWX_WRITE(sc, IWX_CSR_INT, 0xFFFFFFFF);

	ret = iwx_nic_init(sc);
	if (ret) {
		device_printf(sc->sc_dev, "%s: Unable to init nic\n", __func__);
		goto out;
	}

	/*
	 * Now, we load the firmware and don't want to be interrupted, even
	 * by the RF-Kill interrupt (hence mask all the interrupt besides the
	 * FH_TX interrupt which is needed to load the firmware). If the
	 * RF-Kill switch is toggled, we will find out after having loaded
	 * the firmware and return the proper value to the caller.
	 */
	iwx_enable_fw_load_int(sc);

	/* really make sure rfkill handshake bits are cleared */
	/* maybe we should write a few times more?  just to make sure */
	IWX_WRITE(sc, IWX_CSR_UCODE_DRV_GP1_CLR, IWX_CSR_UCODE_SW_BIT_RFKILL);
	IWX_WRITE(sc, IWX_CSR_UCODE_DRV_GP1_CLR, IWX_CSR_UCODE_SW_BIT_RFKILL);

#if 0
	/* Load the given image to the HW */
	if (sc->cfg->device_family >= IWM_DEVICE_FAMILY_8000)
		ret = iwm_pcie_load_given_ucode_8000(sc, fw);
	else
#endif
	ret = iwx_pcie_load_given_ucode(sc, fw);

	/* XXX re-check RF-Kill state */

out:
	return ret;
}

static int
iwx_send_tx_ant_cfg(struct iwx_softc *sc, uint8_t valid_tx_ant)
{
	struct iwx_tx_ant_cfg_cmd tx_ant_cmd = {
		.valid = htole32(valid_tx_ant),
	};

	return iwx_send_cmd_pdu(sc, IWX_TX_ANT_CONFIGURATION_CMD,
	    IWX_CMD_SYNC, sizeof(tx_ant_cmd), &tx_ant_cmd);
}

static int
iwx_send_update_mcc_cmd(struct iwx_softc *sc, const char *alpha2)
{
	struct iwx_mcc_update_cmd mcc_cmd;
	struct iwx_host_cmd hcmd = {
		.id = IWX_MCC_UPDATE_CMD,
		.flags = (IWX_CMD_SYNC | IWX_CMD_WANT_SKB),
		.data = { &mcc_cmd },
	};
	int ret;
#ifdef IWX_DEBUG
	struct iwx_rx_packet *pkt;
	struct iwx_mcc_update_resp_v1 *mcc_resp_v1 = NULL;
	struct iwx_mcc_update_resp *mcc_resp;
	int n_channels;
	uint16_t mcc;
#endif
	int resp_v2 = iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_LAR_SUPPORT_V2);

	if (!iwx_is_lar_supported(sc)) {
		IWX_DPRINTF(sc, IWX_DEBUG_LAR, "%s: no LAR support\n",
		    __func__);
		return 0;
	}

	memset(&mcc_cmd, 0, sizeof(mcc_cmd));
	mcc_cmd.mcc = htole16(alpha2[0] << 8 | alpha2[1]);
	if (iwx_is_wifi_mcc_supported(sc))
		mcc_cmd.source_id = IWX_MCC_SOURCE_GET_CURRENT;
	else
		mcc_cmd.source_id = IWX_MCC_SOURCE_OLD_FW;

	if (resp_v2)
		hcmd.len[0] = sizeof(struct iwx_mcc_update_cmd);
	else
		hcmd.len[0] = sizeof(struct iwx_mcc_update_cmd_v1);

	IWX_DPRINTF(sc, IWX_DEBUG_LAR,
	    "send MCC update to FW with '%c%c' src = %d\n",
	    alpha2[0], alpha2[1], mcc_cmd.source_id);

	ret = iwx_send_cmd(sc, &hcmd);
	if (ret)
		return ret;

#ifdef IWX_DEBUG
	pkt = hcmd.resp_pkt;

	/* Extract MCC response */
	if (resp_v2) {
		mcc_resp = (void *)pkt->data;
		mcc = mcc_resp->mcc;
		n_channels =  le32toh(mcc_resp->n_channels);
	} else {
		mcc_resp_v1 = (void *)pkt->data;
		mcc = mcc_resp_v1->mcc;
		n_channels =  le32toh(mcc_resp_v1->n_channels);
	}

	/* W/A for a FW/NVM issue - returns 0x00 for the world domain */
	if (mcc == 0)
		mcc = 0x3030;  /* "00" - world */

	IWX_DPRINTF(sc, IWX_DEBUG_LAR,
	    "regulatory domain '%c%c' (%d channels available)\n",
	    mcc >> 8, mcc & 0xff, n_channels);
#endif
	iwx_free_resp(sc, &hcmd);

	return 0;
}

static boolean_t
iwx_is_lar_supported(struct iwx_softc *sc)
{
	boolean_t nvm_lar = sc->nvm_data->lar_enabled;
	boolean_t tlv_lar = iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_LAR_SUPPORT);

	if (iwx_lar_disable)
		return FALSE;

	/*
	 * Enable LAR only if it is supported by the FW (TLV) &&
	 * enabled in the NVM
	 */
#if 0
	if (sc->cfg->device_family >= IWM_DEVICE_FAMILY_8000)
#endif
	/* todo: compare against upstream whether or not this creates a bug */
	return nvm_lar && tlv_lar;
#if 0
	else
		return tlv_lar;
#endif
}

static boolean_t
iwx_is_wifi_mcc_supported(struct iwx_softc *sc)
{
	return iwx_fw_has_api(sc, IWX_UCODE_TLV_API_WIFI_MCC_UPDATE) ||
	    iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_LAR_MULTI_MCC);
}

static int
iwx_init_hw(struct iwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	int error, i, ac;

	sc->sf_state = IWX_SF_UNINIT;

	if ((error = iwx_start_hw(sc)) != 0) {
		printf("iwx_start_hw: failed %d\n", error);
		return error;
	}

	if ((error = iwx_run_init_ucode(sc, 0)) != 0) {
		printf("iwx_run_init_ucode: failed %d\n", error);
		return error;
	}

	/*
	 * should stop and start HW since that INIT
	 * image just loaded
	 */
	iwx_stop_device(sc);
	sc->sc_ps_disabled = FALSE;
	if ((error = iwx_start_hw(sc)) != 0) {
		device_printf(sc->sc_dev, "could not initialize hardware\n");
		return error;
	}

	/* omstart, this time with the regular firmware */
	error = iwx_load_ucode_wait_alive(sc, IWX_UCODE_REGULAR);
	if (error) {
		device_printf(sc->sc_dev, "could not load firmware\n");
		goto error;
	}

	error = iwx_sf_update(sc, NULL, FALSE);
	if (error)
		device_printf(sc->sc_dev, "Failed to initialize Smart Fifo\n");

#if 0
	if ((error = iwx_send_bt_init_conf(sc)) != 0) {
		device_printf(sc->sc_dev, "bt init conf failed\n");
		goto error;
	}
#endif

	error = iwx_send_tx_ant_cfg(sc, iwx_get_valid_tx_ant(sc));
	if (error != 0) {
		device_printf(sc->sc_dev, "antenna config failed\n");
		goto error;
	}

	/* Send phy db control command and then phy db calibration */
	if ((error = iwx_send_phy_db_data(sc->sc_phy_db)) != 0)
		goto error;

	if ((error = iwx_send_phy_cfg_cmd(sc)) != 0) {
		device_printf(sc->sc_dev, "phy_cfg_cmd failed\n");
		goto error;
	}

	/* Add auxiliary station for scanning */
	if ((error = iwx_add_aux_sta(sc)) != 0) {
		device_printf(sc->sc_dev, "add_aux_sta failed\n");
		goto error;
	}

	for (i = 0; i < IWX_NUM_PHY_CTX; i++) {
		/*
		 * The channel used here isn't relevant as it's
		 * going to be overwritten in the other flows.
		 * For now use the first channel we have.
		 */
		if ((error = iwx_phy_ctxt_add(sc,
		    &sc->sc_phyctxt[i], &ic->ic_channels[1], 1, 1)) != 0)
			goto error;
	}

	if (iwx_config_ltr(sc) != 0)
		device_printf(sc->sc_dev, "PCIe LTR configuration failed\n");

	error = iwx_power_update_device(sc);
	if (error)
		goto error;

	if ((error = iwx_send_update_mcc_cmd(sc, "ZZ")) != 0)
		goto error;

	if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_UMAC_SCAN)) {
		if ((error = iwx_config_umac_scan(sc)) != 0)
			goto error;
	}

	/* Enable Tx queues. */
	for (ac = 0; ac < WME_NUM_AC; ac++) {
		error = iwx_enable_txq(sc, IWX_STATION_ID, ac,
		    iwx_ac_to_tx_fifo[ac]);
		if (error)
			goto error;
	}

	if ((error = iwx_disable_beacon_filter(sc)) != 0) {
		device_printf(sc->sc_dev, "failed to disable beacon filter\n");
		goto error;
	}

	return 0;

 error:
	iwx_stop_device(sc);
	return error;
}

/*
 * ifnet interfaces
 */

static void
iwx_init(struct iwx_softc *sc)
{
	int error;

	if (sc->sc_flags & IWX_FLAG_HW_INITED) {
		return;
	}
	sc->sc_generation++;
	sc->sc_flags &= ~IWX_FLAG_STOPPED;

	if ((error = iwx_init_hw(sc)) != 0) {
		printf("iwx_init_hw failed %d\n", error);
		iwx_stop(sc);
		return;
	}

	/*
	 * Ok, firmware loaded and we are jogging
	 */
	sc->sc_flags |= IWX_FLAG_HW_INITED;
}

static void
iwx_stop(struct iwx_softc *sc)
{

	sc->sc_flags &= ~IWX_FLAG_HW_INITED;
	sc->sc_flags |= IWX_FLAG_STOPPED;
	sc->sc_generation++;
	sc->sc_tx_timer = 0;
	iwx_stop_device(sc);
	sc->sc_flags &= ~IWX_FLAG_SCAN_RUNNING;
}

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

/* PCI registers */
#define PCI_CFG_RETRY_TIMEOUT	0x041 /* shared with iwm */

// static int iwm_pci_attach()

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

static int
iwx_is_valid_ether_addr(uint8_t *addr)
{
	char zero_addr[IEEE80211_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || IEEE80211_ADDR_EQ(zero_addr, addr))
		return (FALSE);

	return (TRUE);
}

void
iwx_init_task(void *arg1)
{
	struct iwx_softc *sc = arg1;

	IWX_LOCK(sc);
	while (sc->sc_flags & IWX_FLAG_BUSY)
		msleep(&sc->sc_flags, &sc->sc_mtx, 0, "iwxpwr", 0);
	sc->sc_flags |= IWX_FLAG_BUSY;
	iwx_stop(sc);
	if (sc->sc_ic.ic_nrunning > 0)
		iwx_init(sc);
	sc->sc_flags &= ~IWX_FLAG_BUSY;
	wakeup(&sc->sc_flags);
	IWX_UNLOCK(sc);
}

static int
iwx_resume(device_t dev)
{
	struct iwx_softc *sc = device_get_softc(dev);
	int do_reinit = 0;

	/*
	 * We disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_write_config(dev, PCI_CFG_RETRY_TIMEOUT, 0x00, 1);

	if (!sc->sc_attached)
		return 0;

	iwx_init_task(device_get_softc(dev));

	IWX_LOCK(sc);
	if (sc->sc_flags & IWX_FLAG_SCANNING) {
		sc->sc_flags &= ~IWX_FLAG_SCANNING;
		do_reinit = 1;
	}
	IWX_UNLOCK(sc);

	if (do_reinit)
		ieee80211_resume_all(&sc->sc_ic);

	return 0;
}

static device_method_t iwx_pci_methods[] = {
        /* Device interface */
	DEVMETHOD(device_probe,         iwx_probe),
//        DEVMETHOD(device_attach,        iwx_attach),
//        DEVMETHOD(device_detach,        iwx_detach),
//        DEVMETHOD(device_suspend,       iwx_suspend),
//        DEVMETHOD(device_resume,        iwx_resume),

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
