/* copied from FreeBSD sys/dev/iwm/if_iwm_util.c */

/*	$OpenBSD: if_iwm.c,v 1.39 2015/03/23 00:35:19 jsg Exp $	*/

/*
 * Copyright (c) 2014 genua mbh <info@genua.de>
 * Copyright (c) 2014 Fixup Software Ltd.
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

/*-
 * Based on BSD-licensed source modules in the Linux iwlwifi driver,
 * which were used as the reference documentation for this implementation.
 *
 * Driver version we are currently based off of is
 * Linux 3.14.3 (tag id a2df521e42b1d9a23f620ac79dbfe8655a8391dd)
 *
 ***********************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2013 Intel Corporation. All rights reserved.
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
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2013 Intel Corporation. All rights reserved.
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

/*-
 * Copyright (c) 2007-2010 Damien Bergamini <damien.bergamini@free.fr>
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
#include <dev/iwx/if_iwx_binding.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_pcie_trans.h>

/*
 * Send a command to the firmware.  We try to implement the Linux
 * driver interface for the routine.
 * mostly from if_iwn (iwn_cmd()).
 *
 * For now, we always copy the first part and map the second one (if it exists).
 */
int
iwx_send_cmd(struct iwx_softc *sc, struct iwx_host_cmd *hcmd)
{
	struct iwx_tx_ring *ring = &sc->txq[IWX_DQA_CMD_QUEUE];
	struct iwx_tfh_tfd *desc;
	struct iwx_tx_data *txdata = NULL;
	struct iwx_device_cmd *cmd;
	struct mbuf *m;
	bus_dma_segment_t seg;
	bus_addr_t paddr;
	uint64_t addr;
	int error = 0, i, paylen, off;
	int code;
	int async, wantresp;
	int group_id;
	int nsegs;
	size_t hdrlen, datasz;
	uint8_t *data;

	code = hcmd->id;
	async = hcmd->flags & IWX_CMD_ASYNC;
	wantresp = hcmd->flags & IWX_CMD_WANT_SKB;
	data = NULL;

	for (i = 0, paylen = 0; i < nitems(hcmd->len); i++) {
		paylen += hcmd->len[i];
	}

	/* if the command wants an answer, busy sc_cmd_resp */
	if (wantresp) {
		KASSERT(!async, ("invalid async parameter"));
		while (sc->sc_wantresp != -1)
			msleep(&sc->sc_wantresp, &sc->sc_mtx, 0, "iwxcmdsl", 0);
		sc->sc_wantresp = ring->qid << 16 | ring->cur;
		IWX_DPRINTF(sc, IWX_DEBUG_CMD,
		    "wantresp is %x\n", sc->sc_wantresp);
	}

	/*
	 * Is the hardware still available?  (after e.g. above wait).
	 */
	if (sc->sc_flags & IWX_FLAG_STOPPED) {
		error = ENXIO;
		goto out;
	}

	desc = &ring->desc[ring->cur];
	txdata = &ring->data[ring->cur];

	group_id = iwx_cmd_groupid(code);
	if (group_id != 0) {
		hdrlen = sizeof(cmd->hdr_wide);
		datasz = sizeof(cmd->data_wide);
	} else {
		hdrlen = sizeof(cmd->hdr);
		datasz = sizeof(cmd->data);
	}

	if (paylen > datasz) {
		IWX_DPRINTF(sc, IWX_DEBUG_CMD,
		    "large command paylen=%u len0=%u\n",
			paylen, hcmd->len[0]);
		/* Command is too large */
		size_t totlen = hdrlen + paylen;
		if (paylen > IWX_MAX_CMD_PAYLOAD_SIZE) {
			device_printf(sc->sc_dev,
			    "firmware command too long (%zd bytes)\n",
			    totlen);
			error = EINVAL;
			goto out;
		}
		m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, IWX_RBUF_SIZE);
		if (m == NULL) {
			error = ENOBUFS;
			goto out;
		}

		m->m_len = m->m_pkthdr.len = m->m_ext.ext_size;
		error = bus_dmamap_load_mbuf_sg(ring->data_dmat,
		    txdata->map, m, &seg, &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: can't map mbuf, error %d\n", __func__, error);
			m_freem(m);
			goto out;
		}
		txdata->m = m; /* mbuf will be freed in iwx_cmd_done() */
		cmd = mtod(m, struct iwx_device_cmd *);
		paddr = seg.ds_addr;
	} else {
		cmd = &ring->cmd[ring->cur];
		paddr = txdata->cmd_paddr;
	}

	if (group_id != 0) {
		cmd->hdr_wide.opcode = iwx_cmd_opcode(code);
		cmd->hdr_wide.group_id = group_id;
		cmd->hdr_wide.qid = ring->qid;
		cmd->hdr_wide.idx = ring->cur;
		cmd->hdr_wide.length = htole16(paylen);
		cmd->hdr_wide.version = iwx_cmd_version(code);
		data = cmd->data_wide;
	} else {
		cmd->hdr.code = iwx_cmd_opcode(code);
		cmd->hdr.flags = 0;
		cmd->hdr.qid = ring->qid;
		cmd->hdr.idx = ring->cur;
		data = cmd->data;
	}

	for (i = 0, off = 0; i < nitems(hcmd->data); i++) {
		if (hcmd->len[i] == 0)
			continue;
		memcpy(data + off, hcmd->data[i], hcmd->len[i]);
		off += hcmd->len[i];
	}
	KASSERT(off == paylen, ("off %d != paylen %d", off, paylen));

	desc->tbs[0].tb_len = htole16(hdrlen + paylen);
	addr = htole64((uint64_t)paddr);
	memcpy(&desc->tbs[0].addr, &addr, sizeof(addr));
	desc->num_tbs = 1;

	IWX_DPRINTF(sc, IWX_DEBUG_CMD,
	    "iwx_send_cmd 0x%x size=%lu %s\n",
	    code,
	    (unsigned long) (hcmd->len[0] + hcmd->len[1] + hdrlen),
	    async ? " (async)" : "");

	if (paylen > datasz) {
		bus_dmamap_sync(ring->data_dmat, txdata->map,
		    BUS_DMASYNC_PREWRITE);
	} else {
		bus_dmamap_sync(ring->cmd_dma.tag, ring->cmd_dma.map,
		    BUS_DMASYNC_PREWRITE);
	}
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

	error = iwx_pcie_set_cmd_in_flight(sc);
	if (error)
		goto out;
	ring->queued++;

	IWX_DPRINTF(sc, IWX_DEBUG_CMD,
	    "sending command 0x%x qid %d, idx %d\n",
	    code, ring->qid, ring->cur);

	/* Kick command ring. */
	ring->cur = (ring->cur + 1) % IWX_CMD_QUEUE_SIZE;
	IWX_WRITE(sc, IWX_HBUS_TARG_WRPTR, ring->qid << 16 | ring->cur);

	if (!async) {
		/* m..m-mmyy-mmyyyy-mym-ym m-my generation */
		int generation = sc->sc_generation;
		error = msleep(desc, &sc->sc_mtx, PCATCH, "iwxcmd", hz);
		if (error == 0) {
			/* if hardware is no longer up, return error */
			if (generation != sc->sc_generation) {
				error = ENXIO;
			} else {
				hcmd->resp_pkt = (void *)sc->sc_cmd_resp;
			}
		}
	}
 out:
	if (wantresp && error != 0) {
		iwx_free_resp(sc, hcmd);
	}

	return error;
}

/* iwlwifi: mvm/utils.c */
#if 0
int
iwx_send_cmd_pdu(struct iwx_softc *sc, uint32_t id,
	uint32_t flags, uint16_t len, const void *data)
{
	struct iwx_host_cmd cmd = {
		.id = id,
		.len = { len, },
		.data = { data, },
		.flags = flags,
	};

	return iwx_send_cmd(sc, &cmd);
}

/* iwlwifi: mvm/utils.c */
int
iwx_send_cmd_status(struct iwx_softc *sc,
	struct iwx_host_cmd *cmd, uint32_t *status)
{
	struct iwx_rx_packet *pkt;
	struct iwx_cmd_response *resp;
	int error, resp_len;

	KASSERT((cmd->flags & IWX_CMD_WANT_SKB) == 0,
	    ("invalid command"));
	cmd->flags |= IWX_CMD_SYNC | IWX_CMD_WANT_SKB;

	if ((error = iwx_send_cmd(sc, cmd)) != 0)
		return error;
	pkt = cmd->resp_pkt;

	/* Can happen if RFKILL is asserted */
	if (!pkt) {
		error = 0;
		goto out_free_resp;
	}

	if (pkt->hdr.flags & IWX_CMD_FAILED_MSK) {
		error = EIO;
		goto out_free_resp;
	}

	resp_len = iwx_rx_packet_payload_len(pkt);
	if (resp_len != sizeof(*resp)) {
		error = EIO;
		goto out_free_resp;
	}

	resp = (void *)pkt->data;
	*status = le32toh(resp->status);
 out_free_resp:
	iwx_free_resp(sc, cmd);
	return error;
}

/* iwlwifi/mvm/utils.c */
int
iwx_send_cmd_pdu_status(struct iwx_softc *sc, uint32_t id,
	uint16_t len, const void *data, uint32_t *status)
{
	struct iwx_host_cmd cmd = {
		.id = id,
		.len = { len, },
		.data = { data, },
	};

	return iwx_send_cmd_status(sc, &cmd, status);
}
#endif

void
iwx_free_resp(struct iwx_softc *sc, struct iwx_host_cmd *hcmd)
{
	KASSERT(sc->sc_wantresp != -1, ("already freed"));
	KASSERT((hcmd->flags & (IWX_CMD_WANT_SKB|IWX_CMD_SYNC))
	    == (IWX_CMD_WANT_SKB|IWX_CMD_SYNC), ("invalid flags"));
	sc->sc_wantresp = -1;
	wakeup(&sc->sc_wantresp);
}

static void
iwx_dma_map_addr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
        if (error != 0)
                return;
	KASSERT(nsegs == 1, ("too many DMA segments, %d should be 1", nsegs));
	*(bus_addr_t *)arg = segs[0].ds_addr;
}

int
iwx_dma_contig_alloc(bus_dma_tag_t tag, struct iwx_dma_info *dma,
    bus_size_t size, bus_size_t alignment)
{
	int error;

	dma->tag = NULL;
	dma->map = NULL;
	dma->size = size;
	dma->vaddr = NULL;

	error = bus_dma_tag_create(tag, alignment,
            0, BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, size,
            1, size, 0, NULL, NULL, &dma->tag);
        if (error != 0)
                goto fail;

        error = bus_dmamem_alloc(dma->tag, (void **)&dma->vaddr,
            BUS_DMA_NOWAIT | BUS_DMA_ZERO | BUS_DMA_COHERENT, &dma->map);
        if (error != 0)
                goto fail;

        error = bus_dmamap_load(dma->tag, dma->map, dma->vaddr, size,
            iwx_dma_map_addr, &dma->paddr, BUS_DMA_NOWAIT);
        if (error != 0) {
		bus_dmamem_free(dma->tag, dma->vaddr, dma->map);
		dma->vaddr = NULL;
		goto fail;
	}

	bus_dmamap_sync(dma->tag, dma->map, BUS_DMASYNC_PREWRITE);

	return 0;

fail:
	iwx_dma_contig_free(dma);

	return error;
}

void
iwx_dma_contig_free(struct iwx_dma_info *dma)
{
	if (dma->vaddr != NULL) {
		bus_dmamap_sync(dma->tag, dma->map,
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(dma->tag, dma->map);
		bus_dmamem_free(dma->tag, dma->vaddr, dma->map);
		dma->vaddr = NULL;
	}
	if (dma->tag != NULL) {
		bus_dma_tag_destroy(dma->tag);
		dma->tag = NULL;
	}
}

/**
 * iwm_send_lq_cmd() - Send link quality command
 * @init: This command is sent as part of station initialization right
 *        after station has been added.
 *
 * The link quality command is sent as the last step of station creation.
 * This is the special case in which init is set and we call a callback in
 * this case to clear the state indicating that station creation is in
 * progress.
 */
int
iwx_send_lq_cmd(struct iwx_softc *sc, struct iwx_lq_cmd *lq, boolean_t init)
{
	struct iwx_host_cmd cmd = {
		.id = IWX_LQ_CMD,
		.len = { sizeof(struct iwx_lq_cmd), },
		.flags = init ? 0 : IWX_CMD_ASYNC,
		.data = { lq, },
	};

	if (lq->sta_id == IWX_STATION_COUNT)
		return EINVAL;

	return iwx_send_cmd(sc, &cmd);
}

int
iwx_clear_statistics(struct iwx_softc *sc)
{
	struct iwx_statistics_cmd scmd = {
		.flags = htole32(IWX_STATISTICS_FLG_CLEAR)
	};
	struct iwx_host_cmd cmd = {
		.id = IWX_STATISTICS_CMD,
		.len[0] = sizeof(scmd),
		.data[0] = &scmd,
//		.flags = IWX_CMD_WANT_RESP,
		.flags = IWX_CMD_WANT_SKB,
#ifdef not_in_iwx
		.resp_pkt_len = sizeof(struct iwx_notif_statistics),
#endif
	};
	int err;

	err = iwx_send_cmd(sc, &cmd);
	if (err)
		return err;

	iwx_free_resp(sc, &cmd);
	return 0;
}

boolean_t
iwx_rx_diversity_allowed(struct iwx_softc *sc)
{
	if (num_of_ant(iwx_get_valid_rx_ant(sc)) == 1)
		return FALSE;

	/*
	 * XXX Also return FALSE when SMPS (Spatial Multiplexing Powersave)
	 *     is used on any vap (in the future).
	 */

	return TRUE;
}
