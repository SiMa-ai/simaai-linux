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

#define SIMA_NET_MTU	(65536)
/*
 * This needs to be MTU + 16 bytes for ethernet header
 */
#define TX_BUFSZ	(SIMA_NET_MTU + sizeof(struct ethhdr))
#define RX_BUFSZ	(SIMA_NET_MTU + sizeof(struct ethhdr))

#define N_TXBUF		4096
#define N_RXBUF		4096

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
};

#endif
