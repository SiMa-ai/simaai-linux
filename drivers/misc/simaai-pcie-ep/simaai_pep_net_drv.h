// SPDX-License-Identifier: GPL-2.0
/**
 * Host PCIe EP driver for SiMa.ai Davinci SoC
 *
 * Copyright (C) 2024 SiMa.ai
 * Author:
 */

#ifndef _SIMA_PEP_NET_DRV_H_
#define _SIMA_PEP_NET_DRV_H_

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/netdevice.h>

#include "simaai_pcie_net.h"

struct si_pep_dev;

struct si_pep_net_dev {
	spinlock_t net_tx_lock;
	struct net_device *ndev;
	struct napi_struct napi;
	/* Locally cached descriptors */
	struct si_pkt_desc *rx_descs;
	struct si_pkt_desc *tx_descs;
	dma_addr_t rx_descs_p;
	dma_addr_t tx_descs_p;
	struct si_net_hwdata *hwdata;
	dma_addr_t hwdata_p;
	/* Remote descriptor bases */
	dma_addr_t host_tx_descs_p;
	dma_addr_t host_rx_descs_p;
	dma_addr_t host_hwdata_p;
	atomic_t rxphead;
	atomic_t rx_en;
	u32 created;
	__be32 net_addr;
	__be32 net_mask;
};

struct si_pep_net_pdata {
	struct si_pep_net_dev *net_dev;
};

void si_pep_net_cleanup(struct si_pep_dev *);
int si_pep_net_init(struct si_pep_dev *);
int si_pep_net_copy_packets(struct si_pep_dev *, u32, u32);
int si_pep_check_packets(struct si_pep_dev *, u32, u32);

#endif

