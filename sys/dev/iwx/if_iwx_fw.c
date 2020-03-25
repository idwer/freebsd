/* copied from FreeBSD sys/dev/iwm/if_iwm_fw.c */

/*-
 * Based on BSD-licensed source modules in the Linux iwlwifi driver,
 * which were used as the reference documentation for this implementation.
 *
 * Driver version we are currently based off of is
 * Linux 4.7.3 (tag id d7f6728f57e3ecbb7ef34eb7d9f564d514775d75)
 *
 ***********************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
#include <dev/iwx/if_iwx_debug.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_fw.h>
#include <dev/iwx/if_iwx_config.h>
#include <dev/iwx/if_iwx_pcie_trans.h>

int	iwx_ctxt_info_alloc_dma(struct iwx_softc *,
	    const struct iwx_fw_desc *, struct iwx_dma_info *);
void	iwx_ctxt_info_free_paging(struct iwx_softc *);
int	iwx_get_num_sections(const struct iwx_fw_img *, int);
int	iwx_init_fw_sec(struct iwx_softc *, const struct iwx_fw_img *,
	    struct iwx_context_info_dram *);
int	iwx_alloc_fw_monitor_block(struct iwx_softc *, uint8_t,
	    uint8_t);
int	iwx_alloc_fw_monitor(struct iwx_softc *, uint8_t);
int	iwx_apply_debug_destination(struct iwx_softc *);
int	iwx_ctxt_info_init(struct iwx_softc *,
	    const struct iwx_fw_img *);
void	iwx_ctxt_info_free_fw_img(struct iwx_softc *);
void	iwx_ctxt_info_free(struct iwx_softc *);

int
iwx_ctxt_info_alloc_dma(struct iwx_softc *sc,
    const struct iwx_fw_desc *fw_sect, struct iwx_dma_info *dram)
{
	int err = iwx_dma_contig_alloc(sc->sc_dmat, dram, fw_sect->len, 0);
	if (err) {
		IWX_DPRINTF(sc, IWX_DEBUG_FW, "could not allocate context info DMA memory\n");
		return err;
	}

	memcpy(dram->vaddr, fw_sect->data, fw_sect->len);

	return 0;
}

void iwx_ctxt_info_free_paging(struct iwx_softc *sc)
{
	struct iwx_self_init_dram *dram = &sc->init_dram;
	int i;

	if (!dram->paging)
		return;

	/* free paging*/
	for (i = 0; i < dram->paging_cnt; i++)
		iwx_dma_contig_free(dram->paging);

//	free(dram->paging, M_DEVBUF, dram->paging_cnt * sizeof(*dram->paging));
	free(dram->paging, M_DEVBUF);
	dram->paging_cnt = 0;
	dram->paging = NULL;
}

int
// iwx_get_num_sections(const struct iwx_fw_sects *fws, int start)
iwx_get_num_sections(const struct iwx_fw_img *fws, int start)
{
	int i = 0;

	while (start < fws->fw_count &&
	       fws->fw_sect[start].offset != IWX_CPU1_CPU2_SEPARATOR_SECTION &&
	       fws->fw_sect[start].offset != IWX_PAGING_SEPARATOR_SECTION) {
		start++;
		i++;
	}

	return i;
}

int
// iwx_init_fw_sec(struct iwx_softc *sc, const struct iwx_fw_sects *fws,
iwx_init_fw_sec(struct iwx_softc *sc, const struct iwx_fw_img *fws,
    struct iwx_context_info_dram *ctxt_dram)
{
	struct iwx_self_init_dram *dram = &sc->init_dram;
	int i, ret, lmac_cnt, umac_cnt, paging_cnt;

	KASSERT(dram->paging == NULL, ("DRAM is empty"));

	lmac_cnt = iwx_get_num_sections(fws, 0);
	/* add 1 due to separator */
	umac_cnt = iwx_get_num_sections(fws, lmac_cnt + 1);
	/* add 2 due to separators */
	paging_cnt = iwx_get_num_sections(fws, lmac_cnt + umac_cnt + 2);

	dram->fw = mallocarray(umac_cnt + lmac_cnt, sizeof(*dram->fw),
	    M_DEVBUF,  M_ZERO | M_NOWAIT);
	if (!dram->fw)
		return ENOMEM;
	dram->paging = mallocarray(paging_cnt, sizeof(*dram->paging),
	    M_DEVBUF, M_ZERO | M_NOWAIT);
	if (!dram->paging)
		return ENOMEM;

	/* initialize lmac sections */
	for (i = 0; i < lmac_cnt; i++) {
		ret = iwx_ctxt_info_alloc_dma(sc, &fws->fw_sect[i],
						   &dram->fw[dram->fw_cnt]);
		if (ret)
			return ret;
		ctxt_dram->lmac_img[i] =
			htole64(dram->fw[dram->fw_cnt].paddr);
		IWX_DPRINTF(sc, IWX_DEBUG_FW, "%s: firmware LMAC section %d at 0x%llx size %lld\n", __func__, i,
		    (unsigned long long)dram->fw[dram->fw_cnt].paddr,
		    (unsigned long long)dram->fw[dram->fw_cnt].size);
		dram->fw_cnt++;
	}

	/* initialize umac sections */
	for (i = 0; i < umac_cnt; i++) {
		/* access FW with +1 to make up for lmac separator */
		ret = iwx_ctxt_info_alloc_dma(sc,
		    &fws->fw_sect[dram->fw_cnt + 1], &dram->fw[dram->fw_cnt]);
		if (ret)
			return ret;
		ctxt_dram->umac_img[i] =
			htole64(dram->fw[dram->fw_cnt].paddr);
		IWX_DPRINTF(sc, IWX_DEBUG_FW, "%s: firmware UMAC section %d at 0x%llx size %lld\n", __func__, i,
			(unsigned long long)dram->fw[dram->fw_cnt].paddr,
			(unsigned long long)dram->fw[dram->fw_cnt].size);
		dram->fw_cnt++;
	}

	/*
	 * Initialize paging.
	 * Paging memory isn't stored in dram->fw as the umac and lmac - it is
	 * stored separately.
	 * This is since the timing of its release is different -
	 * while fw memory can be released on alive, the paging memory can be
	 * freed only when the device goes down.
	 * Given that, the logic here in accessing the fw image is a bit
	 * different - fw_cnt isn't changing so loop counter is added to it.
	 */
	for (i = 0; i < paging_cnt; i++) {
		/* access FW with +2 to make up for lmac & umac separators */
		int fw_idx = dram->fw_cnt + i + 2;

		ret = iwx_ctxt_info_alloc_dma(sc,
		    &fws->fw_sect[fw_idx], &dram->paging[i]);
		if (ret)
			return ret;

		ctxt_dram->virtual_img[i] = htole64(dram->paging[i].paddr);
		IWX_DPRINTF(sc, IWX_DEBUG_FW, "%s: firmware paging section %d at 0x%llx size %lld\n", __func__, i,
		    (unsigned long long)dram->paging[i].paddr,
		    (unsigned long long)dram->paging[i].size);
		dram->paging_cnt++;
	}

	return 0;
}

int
iwx_alloc_fw_monitor_block(struct iwx_softc *sc, uint8_t max_power,
    uint8_t min_power)
{
	struct iwx_dma_info *fw_mon = &sc->fw_mon;
	uint32_t size = 0;
	uint8_t power;
	int err;

	if (fw_mon->size)
		return 0;

	for (power = max_power; power >= min_power; power--) {
		size = (1 << power);

		err = iwx_dma_contig_alloc(sc->sc_dmat, fw_mon, size, 0);
		if (err)
			continue;

		IWX_DPRINTF(sc, IWX_DEBUG_FW, "allocated 0x%08x bytes for firmware monitor.\n", size);
		break;
	}

	if (err) {
		fw_mon->size = 0;
		return err;
	}

	if (power != max_power)
		IWX_DPRINTF(sc, IWX_DEBUG_TRACE, "Sorry - debug buffer is only %luK while you requested %luK\n",
			(unsigned long)(1 << (power - 10)),
			(unsigned long)(1 << (max_power - 10)));

	return 0;
}

int
iwx_alloc_fw_monitor(struct iwx_softc *sc, uint8_t max_power)
{
	if (!max_power) {
		/* default max_power is maximum */
		max_power = 26;
	} else {
		max_power += 11;
	}

	if (max_power > 26) {
		 IWX_DPRINTF(sc, IWX_DEBUG_FW, "External buffer size for monitor is too big %d, "
		     "check the FW TLV\n", max_power);
		return 0;
	}

	if (sc->fw_mon.size)
		return 0;

	return iwx_alloc_fw_monitor_block(sc, max_power, 11);
}

int
iwx_apply_debug_destination(struct iwx_softc *sc)
{
	struct iwx_fw_dbg_dest_tlv_v1 *dest_v1;
	int i, err;
	uint8_t mon_mode, size_power, base_shift, end_shift;
	uint32_t base_reg, end_reg;

	dest_v1 = sc->sc_fw.dbg.dbg_dest_tlv_v1;
	mon_mode = dest_v1->monitor_mode;
	size_power = dest_v1->size_power;
	base_reg = le32toh(dest_v1->base_reg);
	end_reg = le32toh(dest_v1->end_reg);
	base_shift = dest_v1->base_shift;
	end_shift = dest_v1->end_shift;

	IWX_DPRINTF(sc, IWX_DEBUG_STATE, "applying debug destination %d\n", mon_mode);

	if (mon_mode == EXTERNAL_MODE) {
		err = iwx_alloc_fw_monitor(sc, size_power);
		if (err)
			return err;
	}

	if (!iwx_nic_lock(sc))
		return EBUSY;

	for (i = 0; i < sc->sc_fw.dbg.n_dest_reg; i++) {
		uint32_t addr, val;
		uint8_t op;

		addr = le32toh(dest_v1->reg_ops[i].addr);
		val = le32toh(dest_v1->reg_ops[i].val);
		op = dest_v1->reg_ops[i].op;

		IWX_DPRINTF(sc, IWX_DEBUG_STATE, "%s: op=%u addr=%u val=%u\n", __func__, op, addr, val);
		switch (op) {
		case CSR_ASSIGN:
			IWX_WRITE(sc, addr, val);
			break;
		case CSR_SETBIT:
			IWX_SETBITS(sc, addr, (1 << val));
			break;
		case CSR_CLEARBIT:
			IWX_CLRBITS(sc, addr, (1 << val));
			break;
		case PRPH_ASSIGN:
			iwx_write_prph(sc, addr, val);
			break;
		case PRPH_SETBIT:
			iwx_set_bits_prph(sc, addr, (1 << val));
			break;
		case PRPH_CLEARBIT:
			iwx_clear_bits_prph(sc, addr, (1 << val));
			break;
		case PRPH_BLOCKBIT:
			if (iwx_read_prph(sc, addr) & (1 << val))
				goto monitor;
			break;
		default:
			IWX_DPRINTF(sc, IWX_DEBUG_FW, "FW debug - unknown OP %d\n", op);
			break;
		}
	}

monitor:
	if (mon_mode == EXTERNAL_MODE && sc->fw_mon.size) {
		iwx_write_prph(sc, le32toh(base_reg),
		    sc->fw_mon.paddr >> base_shift);
		iwx_write_prph(sc, end_reg,
		    (sc->fw_mon.paddr + sc->fw_mon.size - 256)
		    >> end_shift);
	}

	iwx_nic_unlock(sc);
	return 0;
}

int
// iwx_ctxt_info_init(struct iwx_softc *sc, const struct iwx_fw_sects *fws)
iwx_ctxt_info_init(struct iwx_softc *sc, const struct iwx_fw_img *fws)
{
	struct iwx_context_info *ctxt_info;
	struct iwx_context_info_rbd_cfg *rx_cfg;
	uint32_t control_flags = 0, rb_size;
	int err;

	err = iwx_dma_contig_alloc(sc->sc_dmat, &sc->ctxt_info_dma,
	    sizeof(*ctxt_info), 0);
	if (err) {
		device_printf(sc->sc_dev, "could not allocate context info DMA memory\n");
		return err;
	}
	ctxt_info = sc->ctxt_info_dma.vaddr;

	ctxt_info->version.version = 0;
	ctxt_info->version.mac_id =
		htole16((uint16_t)IWX_READ(sc, IWX_CSR_HW_REV));
	/* size is in DWs */
	ctxt_info->version.size = htole16(sizeof(*ctxt_info) / 4);

	if (sc->cfg->device_family >= IWX_DEVICE_FAMILY_22560)
		rb_size = IWX_CTXT_INFO_RB_SIZE_2K;
	else
		rb_size = IWX_CTXT_INFO_RB_SIZE_4K;

	KASSERT(IWX_RX_QUEUE_CB_SIZE(IWX_MQ_RX_TABLE_SIZE) < 0xF,
			("table size too large"));
	control_flags = IWX_CTXT_INFO_TFD_FORMAT_LONG |
			(IWX_RX_QUEUE_CB_SIZE(IWX_MQ_RX_TABLE_SIZE) <<
			 IWX_CTXT_INFO_RB_CB_SIZE_POS) |
			(rb_size << IWX_CTXT_INFO_RB_SIZE_POS);
	ctxt_info->control.control_flags = htole32(control_flags);

	/* initialize RX default queue */
	rx_cfg = &ctxt_info->rbd_cfg;
	rx_cfg->free_rbd_addr = htole64(sc->rxq.free_desc_dma.paddr);
	rx_cfg->used_rbd_addr = htole64(sc->rxq.used_desc_dma.paddr);
	rx_cfg->status_wr_ptr = htole64(sc->rxq.stat_dma.paddr);

	/* initialize TX command queue */
	ctxt_info->hcmd_cfg.cmd_queue_addr =
		htole64(sc->txq[IWX_DQA_CMD_QUEUE].desc_dma.paddr);
	ctxt_info->hcmd_cfg.cmd_queue_size =
		IWX_TFD_QUEUE_CB_SIZE(IWX_CMD_QUEUE_SIZE);

	/* allocate ucode sections in dram and set addresses */
	err = iwx_init_fw_sec(sc, fws, &ctxt_info->dram);
	if (err) {
		iwx_ctxt_info_free(sc);
		return err;
	}

	/* Configure debug, if exists */
	if (sc->sc_fw.dbg.dbg_dest_tlv_v1) {
		err = iwx_apply_debug_destination(sc);
		if (err)
			return err;
	}

	/* kick FW self load */
	IWX_WRITE_8(sc, IWX_CSR_CTXT_INFO_BA, sc->ctxt_info_dma.paddr);
	if (!iwx_nic_lock(sc))
		return EBUSY;
	iwx_write_prph(sc, IWX_UREG_CPU_INIT_RUN, 1);
	iwx_nic_unlock(sc);

	/* Context info will be released upon alive or failure to get one */

	return 0;
}

void
iwx_ctxt_info_free_fw_img(struct iwx_softc *sc)
{
	struct iwx_self_init_dram *dram = &sc->init_dram;
	int i;

	if (!dram->fw) {
		KASSERT(dram->fw_cnt == 0, ("array fw_cnt is not empty"));
		return;
	}

	for (i = 0; i < dram->fw_cnt; i++)
		iwx_dma_contig_free(&dram->fw[i]);

//	free(dram->fw, M_DEVBUF, dram->fw_cnt * sizeof(dram->fw[0]));
	free(dram->fw, M_DEVBUF);
	dram->fw_cnt = 0;
	dram->fw = NULL;
}

void
iwx_ctxt_info_free(struct iwx_softc *sc)
{
	iwx_dma_contig_free(&sc->ctxt_info_dma);
	iwx_ctxt_info_free_fw_img(sc);
}

/* assume iwx_free_fw_paging() is identical to iwx_ctxt_info_free_paging() */
#ifdef not_in_iwx
void
iwx_free_fw_paging(struct iwx_softc *sc)
{
	int i;

	if (sc->fw_paging_db[0].fw_paging_block.vaddr == NULL)
		return;

	for (i = 0; i < IWX_NUM_OF_FW_PAGING_BLOCKS; i++) {
		iwx_dma_contig_free(&sc->fw_paging_db[i].fw_paging_block);
	}

	memset(sc->fw_paging_db, 0, sizeof(sc->fw_paging_db));
}
#endif

#ifdef not_in_iwx
static int
iwx_fill_paging_mem(struct iwx_softc *sc, const struct iwx_fw_img *image)
{
	int sec_idx, idx;
	uint32_t offset = 0;

	/*
	 * find where is the paging image start point:
	 * if CPU2 exist and it's in paging format, then the image looks like:
	 * CPU1 sections (2 or more)
	 * CPU1_CPU2_SEPARATOR_SECTION delimiter - separate between CPU1 to CPU2
	 * CPU2 sections (not paged)
	 * PAGING_SEPARATOR_SECTION delimiter - separate between CPU2
	 * non paged to CPU2 paging sec
	 * CPU2 paging CSS
	 * CPU2 paging image (including instruction and data)
	 */
	for (sec_idx = 0; sec_idx < IWX_UCODE_SECTION_MAX; sec_idx++) {
		if (image->sec[sec_idx].offset == IWX_PAGING_SEPARATOR_SECTION) {
			sec_idx++;
			break;
		}
	}

	/*
	 * If paging is enabled there should be at least 2 more sections left
	 * (one for CSS and one for Paging data)
	 */
	if (sec_idx >= nitems(image->sec) - 1) {
		device_printf(sc->sc_dev,
		    "Paging: Missing CSS and/or paging sections\n");
		iwx_free_fw_paging(sc);
		return EINVAL;
	}

	/* copy the CSS block to the dram */
	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "Paging: load paging CSS to FW, sec = %d\n",
		    sec_idx);

	memcpy(sc->fw_paging_db[0].fw_paging_block.vaddr,
	       image->sec[sec_idx].data,
	       sc->fw_paging_db[0].fw_paging_size);

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "Paging: copied %d CSS bytes to first block\n",
		    sc->fw_paging_db[0].fw_paging_size);

	sec_idx++;

	/*
	 * copy the paging blocks to the dram
	 * loop index start from 1 since that CSS block already copied to dram
	 * and CSS index is 0.
	 * loop stop at num_of_paging_blk since that last block is not full.
	 */
	for (idx = 1; idx < sc->num_of_paging_blk; idx++) {
		memcpy(sc->fw_paging_db[idx].fw_paging_block.vaddr,
		       (const char *)image->sec[sec_idx].data + offset,
		       sc->fw_paging_db[idx].fw_paging_size);

		IWX_DPRINTF(sc, IWX_DEBUG_FW,
			    "Paging: copied %d paging bytes to block %d\n",
			    sc->fw_paging_db[idx].fw_paging_size,
			    idx);

		offset += sc->fw_paging_db[idx].fw_paging_size;
	}

	/* copy the last paging block */
	if (sc->num_of_pages_in_last_blk > 0) {
		memcpy(sc->fw_paging_db[idx].fw_paging_block.vaddr,
		       (const char *)image->sec[sec_idx].data + offset,
		       IWX_FW_PAGING_SIZE * sc->num_of_pages_in_last_blk);

		IWX_DPRINTF(sc, IWX_DEBUG_FW,
			    "Paging: copied %d pages in the last block %d\n",
			    sc->num_of_pages_in_last_blk, idx);
	}

	return 0;
}
#endif

#ifdef not_in_iwx
static int
iwx_alloc_fw_paging_mem(struct iwx_softc *sc, const struct iwx_fw_img *image)
{
	int blk_idx = 0;
	int error, num_of_pages;

	if (sc->fw_paging_db[0].fw_paging_block.vaddr != NULL) {
		int i;
		/* Device got reset, and we setup firmware paging again */
		for (i = 0; i < sc->num_of_paging_blk + 1; i++) {
			bus_dmamap_sync(sc->sc_dmat,
			    sc->fw_paging_db[i].fw_paging_block.map,
			    BUS_DMASYNC_POSTWRITE | BUS_DMASYNC_POSTREAD);
		}
		return 0;
	}

	/* ensure IWM_BLOCK_2_EXP_SIZE is power of 2 of IWM_PAGING_BLOCK_SIZE */
        _Static_assert((1 << IWX_BLOCK_2_EXP_SIZE) == IWX_PAGING_BLOCK_SIZE,
	    "IWX_BLOCK_2_EXP_SIZE must be power of 2 of IWX_PAGING_BLOCK_SIZE");

	num_of_pages = image->paging_mem_size / IWX_FW_PAGING_SIZE;
	sc->num_of_paging_blk = ((num_of_pages - 1) /
				    IWX_NUM_OF_PAGE_PER_GROUP) + 1;

	sc->num_of_pages_in_last_blk =
		num_of_pages -
		IWX_NUM_OF_PAGE_PER_GROUP * (sc->num_of_paging_blk - 1);

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "Paging: allocating mem for %d paging blocks, each block holds 8 pages, last block holds %d pages\n",
		    sc->num_of_paging_blk,
		    sc->num_of_pages_in_last_blk);

	/* allocate block of 4Kbytes for paging CSS */
	error = iwx_dma_contig_alloc(sc->sc_dmat,
	    &sc->fw_paging_db[blk_idx].fw_paging_block, IWX_FW_PAGING_SIZE,
	    4096);
	if (error) {
		/* free all the previous pages since we failed */
		iwx_free_fw_paging(sc);
		return ENOMEM;
	}

	sc->fw_paging_db[blk_idx].fw_paging_size = IWX_FW_PAGING_SIZE;

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "Paging: allocated 4K(CSS) bytes for firmware paging.\n");

	/*
	 * allocate blocks in dram.
	 * since that CSS allocated in fw_paging_db[0] loop start from index 1
	 */
	for (blk_idx = 1; blk_idx < sc->num_of_paging_blk + 1; blk_idx++) {
		/* allocate block of IWM_PAGING_BLOCK_SIZE (32K) */
		/* XXX Use iwm_dma_contig_alloc for allocating */
		error = iwx_dma_contig_alloc(sc->sc_dmat,
		     &sc->fw_paging_db[blk_idx].fw_paging_block,
		    IWX_PAGING_BLOCK_SIZE, 4096);
		if (error) {
			/* free all the previous pages since we failed */
			iwx_free_fw_paging(sc);
			return ENOMEM;
		}

		sc->fw_paging_db[blk_idx].fw_paging_size = IWX_PAGING_BLOCK_SIZE;

		IWX_DPRINTF(sc, IWX_DEBUG_FW,
			    "Paging: allocated 32K bytes for firmware paging.\n");
	}

	return 0;
}
#endif

#ifdef not_in_iwx
int
iwx_save_fw_paging(struct iwx_softc *sc, const struct iwx_fw_img *fw)
{
	int ret;

	ret = iwx_alloc_fw_paging_mem(sc, fw);
	if (ret)
		return ret;

	return iwx_fill_paging_mem(sc, fw);
}
#endif

/* send paging cmd to FW in case CPU2 has paging image */
#ifdef not_in_iwx
int
iwx_send_paging_cmd(struct iwx_softc *sc, const struct iwx_fw_img *fw)
{
	int blk_idx;
	uint32_t dev_phy_addr;
	struct iwx_fw_paging_cmd fw_paging_cmd = {
		.flags =
			htole32(IWX_PAGING_CMD_IS_SECURED |
				IWX_PAGING_CMD_IS_ENABLED |
				(sc->num_of_pages_in_last_blk <<
				IWX_PAGING_CMD_NUM_OF_PAGES_IN_LAST_GRP_POS)),
		.block_size = htole32(IWX_BLOCK_2_EXP_SIZE),
		.block_num = htole32(sc->num_of_paging_blk),
	};

	/* loop for for all paging blocks + CSS block */
	for (blk_idx = 0; blk_idx < sc->num_of_paging_blk + 1; blk_idx++) {
		dev_phy_addr = htole32(
		    sc->fw_paging_db[blk_idx].fw_paging_block.paddr >>
		    IWX_PAGE_2_EXP_SIZE);
		fw_paging_cmd.device_phy_addr[blk_idx] = dev_phy_addr;
		bus_dmamap_sync(sc->sc_dmat,
		    sc->fw_paging_db[blk_idx].fw_paging_block.map,
		    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	}

	return iwx_send_cmd_pdu(sc, iwx_cmd_id(IWX_FW_PAGING_BLOCK_CMD,
						   IWX_ALWAYS_LONG_GROUP, 0),
				    0, sizeof(fw_paging_cmd), &fw_paging_cmd);
}
#endif
