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

struct si_pkt_desc {
	uint32_t len;
	uint32_t maxlen;
	void *cpu_addr;
	dma_addr_t dma_addr;
	struct sk_buff *skb;
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
