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
#include <linux/irqreturn.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>

#include "simaai_pcie_net.h"

struct si_pep_dev;
struct si_pep_dma_ll_delem;

/* The pieces a packet can come in: its linear part and each page fragment */
#define SI_PEP_NET_TX_SEGS	(MAX_SKB_FRAGS + 1)

/* Packets one receive DMA transfer brings at most */
#define SI_PEP_NET_RX_BATCH	16

/* A packet the receive transfer in flight is copying into its skb */
struct si_pep_net_rxjob {
	struct sk_buff *skb;
	dma_addr_t maddr;
	u32 len;
	/* What the host's descriptor said about its checksum */
	u32 csum_flags;
	u16 csum_start;
	u16 csum_offset;
};

/* A packet the transmit path holds: mapped, and not yet with the host */
struct si_pep_net_txjob {
	struct sk_buff *skb;
	dma_addr_t maddr[SI_PEP_NET_TX_SEGS];
	u32 mlen[SI_PEP_NET_TX_SEGS];
	u8 n;		/* Pieces mapped */
	bool head;	/* The first of them is the linear part */
};

struct si_pep_net_dev {
	spinlock_t net_tx_lock;
	struct net_device *ndev;
	struct napi_struct napi;
	/* Locally cached descriptors */
	void *desc_vaddr;
	struct si_pkt_desc *rx_descs;
	struct si_pkt_desc *tx_descs;
	dma_addr_t rx_descs_p;
	dma_addr_t tx_descs_p;
	struct si_net_hwdata hwdata;
	/* Remote descriptor bases */
	dma_addr_t host_tx_descs_p;
	dma_addr_t host_rx_descs_p;
	dma_addr_t host_hwdata_p;
	dma_addr_t desc_paddr;
	size_t desc_size;
	/* DMA linked list packets and their descriptors are sent with */
	struct si_pep_dma_ll_delem *tx_ll;
	dma_addr_t tx_ll_p;
	/*
	 * Packets held for transmit, oldest at txq_tail. The first
	 * tx_inflight of them are in the transfer that is running and the
	 * rest wait for it to finish. Under net_tx_lock, as is the rest of
	 * the transmit state below.
	 */
	struct si_pep_net_txjob *txq;
	u32 txq_head;
	u32 txq_tail;
	u32 tx_inflight;
	u32 tx_tries;		/* Times the running transfer was started */
	u32 tx_prod;		/* Host ring slot the next packet goes in */
	unsigned long tx_start;	/* When the running transfer was started */
	struct timer_list tx_timer;
	bool tx_irq_on;		/* Completion interrupt is enabled */
	u32 tx_irq_missed;	/* Transfers in a row it did not come for */
	unsigned long tx_irq_latched;	/* When it was given up on */
	/*
	 * What the transmit path has to say and cannot while it holds
	 * net_tx_lock: a console on this interface sends through that lock.
	 * SI_PEP_NET_REP_* bits, logged by si_pep_net_report() once the lock
	 * is released, with the numbers they are about.
	 */
	unsigned long reports;
	u32 report_ints;
	int report_err;
	/*
	 * The host's ring was full, so the queue is stopped and the timer
	 * looks at the ring until the host has taken enough off it. Under
	 * net_tx_lock like the rest of the transmit state.
	 */
	bool tx_ring_stopped;
	u32 tx_ring_poll_ns;	/* Wait until the next look */
	struct hrtimer tx_ring_timer;
	/*
	 * Receive, where the poll fetches the packets. All of it is the
	 * poll's alone, but for the two flags the interrupt and the timer
	 * leave for it.
	 */
	struct si_pep_dma_ll_delem *rx_ll;
	dma_addr_t rx_ll_p;
	struct sk_buff_head rx_ready;	/* Copied in, not yet handed up */
	struct si_pep_net_rxjob rxq[SI_PEP_NET_RX_BATCH];
	u32 rx_next;		/* Host ring slot to take next */
	u32 rx_inflight;	/* Slots the running transfer covers */
	u32 rx_njobs;		/* Packets of those it is copying */
	bool rx_wait;		/* It is waited for, not left to interrupt */
	u32 rx_tries;		/* Times the running transfer was started */
	unsigned long rx_start;	/* When the running transfer was started */
	struct timer_list rx_timer;
	bool rx_irq_on;		/* Completion interrupt is enabled */
	bool rx_irq_seen;	/* It came for the running transfer */
	bool rx_timer_fired;	/* The timer found the transfer overdue */
	u32 rx_irq_missed;	/* Transfers in a row it did not come for */
	unsigned long rx_irq_latched;	/* When it was given up on */
	/* What the DMA that carries SI_NET_MSI copies to the host */
	u32 *msi_word;
	dma_addr_t msi_word_p;
	u32 features;	/* SI_NET_F_* agreed with the host */
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
int si_pep_net_init(struct si_pep_dev *, dma_addr_t, u32);
int si_pep_net_copy_packets(struct si_pep_dev *, u32, u32);
int si_pep_check_packets(struct si_pep_dev *, u32, u32);
irqreturn_t si_pep_net_tx_irq(int, void *);
irqreturn_t si_pep_net_rx_irq(int, void *);
void si_pep_net_rx_schedule(struct si_pep_dev *);

#endif

