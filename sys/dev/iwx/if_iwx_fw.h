/* copied from FreeBSD sys/dev/iwm/if_iwm_fw.h */

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
 * Copyright(c) 2008 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016        Intel Deutschland GmbH
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
 * Copyright(c) 2005 - 2014 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016        Intel Deutschland GmbH
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
/*
 * $FreeBSD$
 */
#ifndef	__IF_IWM_FW_H__
#define	__IF_IWM_FW_H__

/*
 * Block paging calculations
 */
#define IWX_PAGE_2_EXP_SIZE 12 /* 4K == 2^12 */
#define IWX_FW_PAGING_SIZE (1 << IWX_PAGE_2_EXP_SIZE) /* page size is 4KB */
#define IWX_PAGE_PER_GROUP_2_EXP_SIZE 3
/* 8 pages per group */
#define IWX_NUM_OF_PAGE_PER_GROUP (1 << IWX_PAGE_PER_GROUP_2_EXP_SIZE)
/* don't change, support only 32KB size */
#define IWX_PAGING_BLOCK_SIZE (IWX_NUM_OF_PAGE_PER_GROUP * IWX_FW_PAGING_SIZE)
/* 32K == 2^15 */
#define IWX_BLOCK_2_EXP_SIZE (IWX_PAGE_2_EXP_SIZE + IWX_PAGE_PER_GROUP_2_EXP_SIZE)

/*
 * Image paging calculations
 */
#define IWX_BLOCK_PER_IMAGE_2_EXP_SIZE 5
/* 2^5 == 32 blocks per image */
#define IWX_NUM_OF_BLOCK_PER_IMAGE (1 << IWX_BLOCK_PER_IMAGE_2_EXP_SIZE)
/* maximum image size 1024KB */
#define IWX_MAX_PAGING_IMAGE_SIZE (IWX_NUM_OF_BLOCK_PER_IMAGE * IWX_PAGING_BLOCK_SIZE)

/* Virtual address signature */
#define IWM_PAGING_ADDR_SIG 0xAA000000

#define IWX_PAGING_CMD_IS_SECURED (1 << 9)
#define IWX_PAGING_CMD_IS_ENABLED (1 << 8)
#define IWX_PAGING_CMD_NUM_OF_PAGES_IN_LAST_GRP_POS 0
#define IWM_PAGING_TLV_SECURE_MASK 1

extern	void iwx_free_fw_paging(struct iwx_softc *);
extern	int iwx_save_fw_paging(struct iwx_softc *, const struct iwx_fw_img *);
extern	int iwx_send_paging_cmd(struct iwx_softc *, const struct iwx_fw_img *);

#endif	/* __IF_IWM_FW_H__ */
