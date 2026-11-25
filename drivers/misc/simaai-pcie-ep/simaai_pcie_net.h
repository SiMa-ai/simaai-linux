// SPDX-License-Identifier: GPL-2.0
/**
 * Host PCIe EP driver for SiMa.ai Davinci SoC
 *
 * Copyright (C) 2021-2022 SiMa.ai
 * Author:
 */

#ifndef _SIMA_PCIE_NET_H_
#define _SIMA_PCIE_NET_H_

#include <linux/bitfield.h>
#include <linux/compiler_types.h>
#include <linux/kernel.h>
#include <linux/types.h>

/* si_pkt_desc.csum_flags */
#define SI_PKT_F_CSUM_PARTIAL	BIT(0)

struct si_pkt_desc {
	uint32_t len;
	uint32_t maxlen;
	void *cpu_addr;
	dma_addr_t dma_addr;
	union {
		/* On the SoC, the packet it received into this slot */
		struct sk_buff *skb;
		/*
		 * What the sender tells the receiver, once the feature for
		 * the direction is agreed (SI_NET_F_PARTIAL_CSUM from the
		 * SoC, SI_NET_F_HOST_PARTIAL_CSUM from the host). With
		 * SI_PKT_F_CSUM_PARTIAL set, the checksum of the packet
		 * from csum_start to its end belongs csum_offset past
		 * csum_start, both counted from the ethernet header. All
		 * zero otherwise, which is what the field always held.
		 */
		struct {
			uint16_t csum_start;
			uint16_t csum_offset;
			uint32_t csum_flags;
		};
	};
};

struct si_net_hwdata {
	u32 tx_desc_low;
	u32 tx_desc_high;
	u32 rx_desc_low;
	u32 rx_desc_high;
	u32 tx_head; /* As seen from the host */
	u32 tx_tail;
	u32 rx_head;
	u32 rx_tail;
	u32 mtu;	/* Buffer size will be this + sizeof(struct ethhdr) */
	u32 n_txbuf;
	u32 n_rxbuf;
};

#endif
