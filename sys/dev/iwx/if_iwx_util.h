/* copied from FreeBSD sys/dev/iwm/if_iwm_util.h */

/*	$OpenBSD: if_iwm.c,v 1.39 2015/03/23 00:35:19 jsg Exp $	*/
/*	$FreeBSD$ */

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
#ifndef	__IF_IWM_UTIL_H__
#define	__IF_IWM_UTIL_H__

extern	int iwx_send_cmd(struct iwx_softc *sc, struct iwx_host_cmd *hcmd);
extern	int iwx_send_cmd_pdu(struct iwx_softc *sc, uint32_t id,
	    uint32_t flags, uint16_t len, const void *data);

extern	int iwx_send_cmd_status(struct iwx_softc *sc,
	    struct iwx_host_cmd *cmd, uint32_t *status);
extern	int iwx_send_cmd_pdu_status(struct iwx_softc *sc, uint32_t id,
	uint16_t len, const void *data, uint32_t *status);
extern	void iwx_free_resp(struct iwx_softc *sc, struct iwx_host_cmd *hcmd);

extern	int iwx_dma_contig_alloc(bus_dma_tag_t tag, struct iwx_dma_info *dma,
				 bus_size_t size, bus_size_t alignment);
extern	void iwx_dma_contig_free(struct iwx_dma_info *);

extern	int iwx_send_lq_cmd(struct iwx_softc *sc, struct iwx_lq_cmd *lq,
				boolean_t init);

extern	int iwx_clear_statistics(struct iwx_softc *sc);
extern	boolean_t iwx_rx_diversity_allowed(struct iwx_softc *sc);

extern	uint8_t iwx_ridx2rate(struct ieee80211_rateset *rs, int ridx);
/* this is a verbatim copy/paste from openbsd */
extern int iwx_enable_txq(struct iwx_softc *sc, int sta_id, int qid, int tid,
    int num_slots);
extern	int iwx_flush_tx_path(struct iwx_softc *sc, uint32_t flags);

static inline uint8_t
iwx_get_valid_tx_ant(struct iwx_softc *sc)
{
	return sc->nvm_data && sc->nvm_data->valid_tx_ant ?
	       sc->sc_fw.valid_tx_ant & sc->nvm_data->valid_tx_ant :
	       sc->sc_fw.valid_tx_ant;
}

static inline uint8_t
iwx_get_valid_rx_ant(struct iwx_softc *sc)
{
	return sc->nvm_data && sc->nvm_data->valid_rx_ant ?
	       sc->sc_fw.valid_rx_ant & sc->nvm_data->valid_rx_ant :
	       sc->sc_fw.valid_rx_ant;
}

static inline uint32_t
iwx_get_phy_config(struct iwx_softc *sc)
{
	uint32_t phy_config = ~(IWX_FW_PHY_CFG_TX_CHAIN |
				IWX_FW_PHY_CFG_RX_CHAIN);
	uint32_t valid_rx_ant = iwx_get_valid_rx_ant(sc);
	uint32_t valid_tx_ant = iwx_get_valid_tx_ant(sc);

	phy_config |= valid_tx_ant << IWX_FW_PHY_CFG_TX_CHAIN_POS |
		      valid_rx_ant << IWX_FW_PHY_CFG_RX_CHAIN_POS;

	return sc->sc_fw.phy_config & phy_config;
}

#endif	/* __IF_IWM_UTIL_H__ */
