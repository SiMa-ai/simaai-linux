// SPDX-License-Identifier: GPL-2.0
/**
 * Platform PCIe EP driver for SiMa.ai Davinci SoC
 *
 * Copyright (C) 2024 SiMa.ai
 * Author:
 */

#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/circ_buf.h>
#include <linux/string.h>
#include <linux/align.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include "simaai_pep_hw.h"
#include "simaai_pep_drv.h"

/* Packets the transmit path holds at most, queued or being transferred */
#define SI_PEP_NET_TXQ_LEN	64
#define SI_PEP_NET_TXQ_MASK	(SI_PEP_NET_TXQ_LEN - 1)
/* Packets one DMA transfer carries at most */
#define SI_PEP_NET_TX_BATCH	16
/*
 * Linked list elements for that many: a descriptor and the pieces for each,
 * and the link that ends the list.
 */
#define SI_PEP_NET_TX_LL_ENTS \
	(SI_PEP_NET_TX_BATCH * (SI_PEP_NET_TX_SEGS + 1) + 1)
/* Tries an aborted transfer gets, as the plain transfer gives it */
#define SI_PEP_NET_TX_TRIES	3
/* How long a transfer may run, as for every other transfer in the driver */
#define SI_PEP_NET_TX_TIMEOUT	(2 * HZ)
/* How often the timer looks at a transfer that is in flight, in jiffies */
#define SI_PEP_NET_TX_TICK	2
/* How long a stopped transfer gets to leave the running state, in us */
#define SI_PEP_NET_TX_STOP_US	1000
/*
 * Finished transfers in a row the timer may find before the interrupt is
 * given up on
 */
#define SI_PEP_NET_TX_IRQ_MISSES	3
/*
 * How long an interrupt that has been given up on stays given up on. A
 * transfer is then left to it again, and one more miss gives it up for
 * another round.
 */
#define SI_PEP_NET_IRQ_RETRY	(10 * HZ)
/*
 * The host does not say when it has taken packets off its ring, so a ring
 * that is full is looked at from a timer. The first look comes soon, since a
 * host that is keeping up frees slots within microseconds. Each look that
 * finds too little doubles the wait, up to a limit, so a host that has
 * stopped costs a thousand looks a second at most. In ns.
 */
#define SI_PEP_NET_RING_POLL_MIN_NS	10000
#define SI_PEP_NET_RING_POLL_MAX_NS	1000000
/*
 * A soft timer, so its handler runs in softirq context like the transmit
 * timer's. The completion interrupt takes net_tx_lock without disabling
 * interrupts, which would deadlock against a handler that took it from hard
 * interrupt context if that interrupt were ever threaded.
 */
#define SI_PEP_NET_RING_TIMER_MODE	HRTIMER_MODE_REL_SOFT
/*
 * What the transmit channel's interrupt setup is left at between transfers:
 * nothing enabled and everything masked. Masked matters, because a list
 * element that asks for a watermark event raises the interrupt for it
 * unless the mask says otherwise, and the transfers that are waited for
 * leave this register alone.
 */
#define SI_PEP_NET_TX_INT_IDLE	DMA_INT_SETUP_INTS_MASK

/* Linked list elements to receive a batch: one for each packet, and the link */
#define SI_PEP_NET_RX_LL_ENTS	(SI_PEP_NET_RX_BATCH + 1)
/*
 * Packets that may be waiting to be handed up before any more are taken
 * from the host. A napi poll's worth.
 */
#define SI_PEP_NET_RX_READY_MAX	64
/*
 * A receive transfer of no more than this many bytes is waited for: it is
 * over sooner than its interrupt could be taken and the poll run again.
 */
#define SI_PEP_NET_RX_WAIT_BYTES	4096

static bool net_async_tx = true;
module_param(net_async_tx, bool, 0644);
MODULE_PARM_DESC(net_async_tx,
	"Send veth packets without waiting in the sender for the DMA transfer");

static bool net_async_rx = true;
module_param(net_async_rx, bool, 0644);
MODULE_PARM_DESC(net_async_rx,
	"Hand up received veth packets while the DMA transfer of the next ones runs");

static int si_pep_net_rx_poll(struct napi_struct *napi, int budget);

u8 si_pep_net_dev_addr[MAX_ADDR_LEN] = { 0 };

static int si_pep_net_get_txdescs(struct si_pep_net_dev *net)
{
	struct si_pep_dev *pep = container_of(net, struct si_pep_dev, net_dev);

	if (net->host_hwdata_p == 0) /* Host hasn't configured us */
		return -ENOENT;

	if (net->host_rx_descs_p == 0)
		return -ENOENT;

	si_pep_copy_from_host(pep, net->tx_descs_p, net->host_rx_descs_p,
			sizeof(struct si_pkt_desc) * net->hwdata.n_rxbuf,
			ETH_DESC_DMA_CH);

	return 0;
}

int si_pep_net_copy_packets(struct si_pep_dev *pep, u32 start, u32 npkts)
{
	u32 len;
	u8 *skb_data;
	int i;
	int ret;
	int done = 0;
	struct si_pep_net_dev *net = &pep->net_dev;
	struct net_device *ndev = net->ndev;
	struct si_pkt_desc *pdesc;
	dma_addr_t dmaaddr;
	u32 csum_flags;
	u16 csum_start;
	u16 csum_offset;

	si_dbg_net(pep, "Copying %u packets from %u", npkts, start);

	i = start;
	while (npkts) {
		pdesc = &net->rx_descs[i];
		len = pdesc->len;
		/*
		 * What the host says about the checksum shares its storage
		 * with the skb pointer kept here, so take it first.
		 */
		csum_start = pdesc->csum_start;
		csum_offset = pdesc->csum_offset;
		csum_flags = pdesc->csum_flags;
		/*
		 * Clear any stale pointer first: a slot that fails the
		 * checks below must not retain a (now freed) skb from a
		 * previous trip around the ring, or the poll routine would
		 * hand that dangling pointer to napi_gro_receive().
		 */
		pdesc->skb = NULL;
		if (len > (net->hwdata.mtu + sizeof(struct ethhdr)))
			goto next;

		pdesc->skb = dev_alloc_skb(len);
		if (!pdesc->skb) {
			pdesc->skb = NULL;
			si_err(pep, "Failed to allocate Skb");
			goto next;
		}

		skb_data = skb_put(pdesc->skb, len);
		dmaaddr = dma_map_single(pep->dev, skb_data, len, DMA_FROM_DEVICE);
		ret = si_pep_copy_from_host(pep, dmaaddr, pdesc->dma_addr, len,
				ETH_DATA_DMA_CH);
		dma_unmap_single(pep->dev, dmaaddr, len, DMA_FROM_DEVICE);
		if (ret) {
			si_err(pep, "Failed to copy from host");
			dev_kfree_skb(pdesc->skb);
			pdesc->skb = NULL;
			goto next;
		}

		pdesc->skb->dev=ndev;
		/*
		 * The link does not corrupt, so nothing needs verifying.
		 * But a packet the host left without its L4 checksum has to
		 * be marked as such rather than as verified, or it would go
		 * out that way if it is forwarded through another interface.
		 */
		if (!(net->features & SI_NET_F_HOST_PARTIAL_CSUM) ||
		    !(csum_flags & SI_PKT_F_CSUM_PARTIAL) ||
		    !skb_partial_csum_set(pdesc->skb, csum_start, csum_offset))
			pdesc->skb->ip_summed = CHECKSUM_UNNECESSARY;
		pdesc->skb->protocol = eth_type_trans(pdesc->skb, ndev);
		done++;
next:
		npkts--;
		i++;
	}

	return done;
}

int si_pep_check_packets(struct si_pep_dev *pep, u32 head, u32 tail)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 phead = atomic_read(&net->rxphead);
	
	si_dbg_net(pep, "Head: %u, Tail: %u, PHead: %u", head, tail, phead);

	if (unlikely(head > net->hwdata.n_txbuf)) {
		pep->drv_ops->net_enable_rxint(pep);
		return -EINVAL;
	}

	if (unlikely(phead > net->hwdata.n_txbuf)) {
		atomic_set(&net->rxphead, 0);
		pep->drv_ops->net_enable_rxint(pep);
		return -EINVAL;
	}

	if (unlikely(tail >= net->hwdata.n_txbuf)) {
		net_set_rxtail(pep, 0);
		pep->drv_ops->net_enable_rxint(pep);
		return -EINVAL;
	}

	if (head == 0 || head == phead || head == tail) {
		pep->drv_ops->net_enable_rxint(pep);
		return -ENOENT;
	}

	if (phead >= net->hwdata.n_txbuf)
		phead = 0;

	if (phead > head) {
		si_pep_copy_from_host(pep,
				net->rx_descs_p +
				(phead * sizeof(struct si_pkt_desc)),
				net->host_tx_descs_p +
				(phead * sizeof(struct si_pkt_desc)),
				sizeof(struct si_pkt_desc) *
				(net->hwdata.n_txbuf - phead),
				ETH_DESC_DMA_CH);
		si_pep_net_copy_packets(pep, phead,
				(net->hwdata.n_txbuf - phead));
		si_pep_copy_from_host(pep,
				net->rx_descs_p, net->host_tx_descs_p,
				sizeof(struct si_pkt_desc) * head,
				ETH_DESC_DMA_CH);
		si_pep_net_copy_packets(pep, 0, head);
	} else {
		si_pep_copy_from_host(pep,
				net->rx_descs_p +
				(phead * sizeof(struct si_pkt_desc)),
				net->host_tx_descs_p +
				(phead * sizeof(struct si_pkt_desc)),
				sizeof(struct si_pkt_desc) * (head - phead),
				ETH_DESC_DMA_CH);
		si_pep_net_copy_packets(pep, phead, head - phead);
	}

	smp_wmb();
	atomic_set(&net->rxphead, head);
	napi_schedule(&net->napi);

	return 0;
}

static int si_pep_net_copy_packet(struct si_pep_dev *pep, u32 idx)
{
	u32 len;
	u8 *skb_data;
	int ret;
	struct si_pep_net_dev *net = &pep->net_dev;
	struct net_device *ndev = net->ndev;
	struct si_pkt_desc *pdesc;
	dma_addr_t dmaaddr;

	si_dbg_net(pep, "Copying packet %u", idx);

	pdesc = &net->rx_descs[idx];
	len = pdesc->len;
	pdesc->skb = NULL;
	if (len > (net->hwdata.mtu + sizeof(struct ethhdr)))
		return -EINVAL;

	pdesc->skb = napi_alloc_skb(&net->napi, len);
	if (pdesc->skb == NULL) {
		si_err(pep, "Failed to allocate Skb");
		return -ENOMEM;
	}

	skb_data = skb_put(pdesc->skb, len);
	dmaaddr = dma_map_single(pep->dev, skb_data, len, DMA_FROM_DEVICE);
	ret = si_pep_copy_from_host(pep, dmaaddr, pdesc->dma_addr, len,
			ETH_DATA_DMA_CH);
	dma_unmap_single(pep->dev, dmaaddr, len, DMA_FROM_DEVICE);
	if (ret) {
		si_err(pep, "Failed to copy from host");
		dev_kfree_skb(pdesc->skb);
		pdesc->skb = NULL;
		return ret;
	}

	pdesc->skb->dev = ndev;
	pdesc->skb->ip_summed = CHECKSUM_UNNECESSARY;
	pdesc->skb->protocol = eth_type_trans(pdesc->skb, ndev);

	return 0;
}

static int si_pep_net_poll(struct napi_struct *napi, int budget)
{
	struct si_pep_net_dev *net = container_of(napi, struct si_pep_net_dev,
			napi);
	struct si_pep_dev *pep = container_of(net, struct si_pep_dev, net_dev);
	struct net_device *ndev = net->ndev;
	struct si_pkt_desc *pdesc;
	int pkts_done = 0;
	int pkts_avail = 0;
	u32 rxhead;
	u32 rxtail;

	/*
	 * Where a doorbell says there are packets, this poll fetches them
	 * itself. What follows is for the platform that has a thread look
	 * for them and copy them in before the poll is scheduled.
	 */
	if (pep->modalix)
		return si_pep_net_rx_poll(napi, budget);

	rxhead = atomic_read(&net->rxphead);
	smp_rmb();
	/*
	 * head uses the "last index + 1" convention and can equal n_txbuf;
	 * normalise it to a ring position so the wrapping tail can reach it.
	 * Otherwise pkts_avail never falls to 0 once tail wraps and the poll
	 * re-delivers the whole ring forever (NAPI never completes). The host
	 * TX producer reserves a slot, so head == n_txbuf with tail == 0 only
	 * means the ring is empty. (Mirror of the rxphead read in the host's
	 * si_mla_net_poll(); note rxphead itself is left intact because
	 * si_pep_check_packets() uses it as the produced-up-to mark.)
	 */
	if (rxhead >= net->hwdata.n_txbuf)
		rxhead = 0;

	rxtail = net_get_rxtail(pep);
	if (rxtail >= net->hwdata.n_txbuf) {
		si_err(pep, "Invalid rx tail");
		rxtail = 0;
		net_set_rxtail(pep, rxtail);
		goto err_out;
	}

	if (rxtail > rxhead) {
		pkts_avail = (net->hwdata.n_txbuf - rxtail) + rxhead;
	} else {
		pkts_avail = rxhead - rxtail;
	}

	if (pkts_avail == 0) {
		si_dbg_net(pep, "Packets unavailable. Head:%u, Tail:%u", rxhead,
				rxtail);
		goto err_out;
	}

	si_dbg_net(pep, "Head: %u, Tail: %u, Packets: %d", rxhead, rxtail,
			pkts_avail);

	while (pkts_avail && (pkts_done < budget)) {
		si_dbg_net(pep, "Processing packet %d", rxtail);
		pdesc = &net->rx_descs[rxtail];
		si_dbg_net(pep, "Descriptor 0x%px, Len: %u, skb: %px",
				pdesc, pdesc->len, pdesc->skb);

		if (pdesc->skb == NULL) {
			ndev->stats.rx_dropped++;
			pkts_avail--;
		} else if (WARN_ONCE(!virt_addr_valid(pdesc->skb) ||
				pdesc->skb->dev == NULL,
				"Corrupt RX skb: slot=%u skb=%px dev=%px len=%u "
				"head=%u tail=%u avail=%d done=%d\n",
				rxtail, pdesc->skb,
				virt_addr_valid(pdesc->skb) ?
					pdesc->skb->dev : NULL,
				pdesc->len, rxhead, rxtail, pkts_avail,
				pkts_done)) {
			ndev->stats.rx_errors++;
			pkts_avail--;
		} else {
			napi_gro_receive(napi, pdesc->skb);
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += pdesc->len;
			pkts_done++;
			pkts_avail--;
		}

		pdesc->skb = NULL;
		rxtail++;
		if (rxtail == net->hwdata.n_txbuf)
			rxtail = 0;
	}

	net_set_rxtail(pep, rxtail);

	if (pkts_done < budget) {
err_out:
		napi_complete_done(napi, pkts_done);
		pep->drv_ops->net_enable_rxint(pep);
	}

	si_dbg_net(pep, "Processed %d packets", pkts_done);
	return pkts_done;
}

static void si_pep_net_ll_set(struct si_pep_dma_ll_delem *delem,
		dma_addr_t src, dma_addr_t dst, u32 size)
{
	delem->flags = CYCLE_BIT;
	delem->xfer_size = size;
	delem->sar_low = lower_32_bits(src);
	delem->sar_high = upper_32_bits(src);
	delem->dar_low = lower_32_bits(dst);
	delem->dar_high = upper_32_bits(dst);
}

/*
 * Map a packet for DMA as it comes from the stack: the linear part, then
 * each page fragment. A TCP payload arrives as fragments now that the
 * interface takes scatter-gather, and is never copied together here.
 */
static int si_pep_net_map_skb(struct si_pep_dev *pep, struct sk_buff *skb,
		struct si_pep_net_txjob *job)
{
	struct device *dev = pep->dev;
	const skb_frag_t *frag;
	unsigned int len;
	int n = 0;
	int i;

	/* Not something the stack sends without NETIF_F_FRAGLIST */
	if (unlikely(skb_has_frag_list(skb)) && skb_linearize(skb))
		return -ENOMEM;

	job->skb = skb;
	job->head = false;

	len = skb_headlen(skb);
	if (len) {
		job->maddr[n] = dma_map_single(dev, skb->data, len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(dev, job->maddr[n]))
			return -ENOMEM;
		job->mlen[n++] = len;
		job->head = true;
	}

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		frag = &skb_shinfo(skb)->frags[i];
		len = skb_frag_size(frag);
		if (len == 0)
			continue;

		job->maddr[n] = skb_frag_dma_map(dev, frag, 0, len,
				DMA_TO_DEVICE);
		if (dma_mapping_error(dev, job->maddr[n]))
			goto unmap;
		job->mlen[n++] = len;
	}

	job->n = n;
	if (unlikely(n == 0))
		return -EINVAL;

	return 0;

unmap:
	i = 0;
	if (job->head) {
		dma_unmap_single(dev, job->maddr[0], job->mlen[0],
				DMA_TO_DEVICE);
		i = 1;
	}
	for (; i < n; i++)
		dma_unmap_page(dev, job->maddr[i], job->mlen[i],
				DMA_TO_DEVICE);

	return -ENOMEM;
}

static void si_pep_net_unmap_job(struct si_pep_dev *pep,
		struct si_pep_net_txjob *job)
{
	struct device *dev = pep->dev;
	int i = 0;

	if (job->head) {
		dma_unmap_single(dev, job->maddr[0], job->mlen[0],
				DMA_TO_DEVICE);
		i = 1;
	}
	for (; i < job->n; i++)
		dma_unmap_page(dev, job->maddr[i], job->mlen[i],
				DMA_TO_DEVICE);
}

/* Describe the packet in the descriptor of the transmit slot it goes in */
static void si_pep_net_fill_desc(struct si_pep_net_dev *net, u32 idx,
		struct sk_buff *skb)
{
	struct si_pkt_desc *pdesc = &net->tx_descs[idx];

	pdesc->len = skb->len;
	/*
	 * Only a host that agreed to SI_NET_F_PARTIAL_CSUM gets the interface
	 * the checksum offload that leaves a packet this way.
	 */
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		pdesc->csum_start = skb_checksum_start_offset(skb);
		pdesc->csum_offset = skb->csum_offset;
		pdesc->csum_flags = SI_PKT_F_CSUM_PARTIAL;
	} else {
		pdesc->csum_start = 0;
		pdesc->csum_offset = 0;
		pdesc->csum_flags = 0;
	}
}

/*
 * Add a packet for transmit slot idx to the linked list, from element lli:
 * the slot's descriptor into the host's descriptor ring, then the packet,
 * piece after piece, into the one buffer that descriptor names. Returns the
 * next free element.
 *
 * The descriptor goes with the packet, so telling the host how long it is
 * does not take a transfer of its own.
 */
static int si_pep_net_ll_add(struct si_pep_net_dev *net, int lli,
		struct si_pep_net_txjob *job, u32 idx)
{
	struct si_pkt_desc *pdesc = &net->tx_descs[idx];
	dma_addr_t dst = pdesc->dma_addr;
	size_t desc_off = idx * sizeof(*pdesc);
	unsigned int off = 0;
	int i;

	si_pep_net_ll_set(&net->tx_ll[lli++], net->tx_descs_p + desc_off,
			net->host_rx_descs_p + desc_off, sizeof(*pdesc));
	for (i = 0; i < job->n; i++) {
		si_pep_net_ll_set(&net->tx_ll[lli++], job->maddr[i],
				dst + off, job->mlen[i]);
		off += job->mlen[i];
	}

	return lli;
}

/* Finish the linked list at ll, which has lli data elements */
static void si_pep_net_ll_end(struct si_pep_dma_ll_delem *ll, dma_addr_t ll_p,
		int lli)
{
	struct si_pep_dma_ll_lelem *lelem;

	ll[lli - 1].flags = CYCLE_BIT | LWIE_BIT;

	lelem = (struct si_pep_dma_ll_lelem *)&ll[lli];
	memset(lelem, 0, sizeof(*lelem));
	lelem->flags = LLP_ELEMENT | TCB_BIT;
	lelem->ll_ptr_low = lower_32_bits(ll_p);
	lelem->ll_ptr_high = upper_32_bits(ll_p);
}

/*
 * Start the transfer the linked list at ll_p describes, in direction dir,
 * and return. The channel is set up as si_pep_copy_do_ll() sets it up, by the
 * same function, and then given an interrupt setup of its own: if irq is set,
 * it interrupts when the transfer stops or aborts. The last element asks for
 * a watermark event, as every list in the driver does, and that one is masked
 * because the stop that follows it is the one that matters.
 */
static void si_pep_net_ll_start(struct si_pep_dev *pep, int dir,
		dma_addr_t ll_p, bool irq)
{
	const int ch = ETH_DATA_DMA_CH;

	si_pep_dma_ll_program(pep, ch, dir, ll_p);
	DMA_REG_WRITE(pep, ch, dir, int_setup, irq ? (DMA_INT_SETUP_LSIE |
			DMA_INT_SETUP_LAIE | DMA_INT_SETUP_WATERMARK_MASK) :
			SI_PEP_NET_TX_INT_IDLE);
	DMA_REG_WRITE(pep, ch, dir, doorbell, DMA_DOORBELL_DB_START);
}

/*
 * What the transmit path reports. It cannot do it where it finds out: it
 * holds net_tx_lock there, a console on this interface sends through that
 * lock, and a message printed under it can wait for itself for good. So the
 * finding is noted, with the number it is about, and si_pep_net_report()
 * says it once the caller has let go of the lock.
 */
enum {
	SI_PEP_NET_REP_STOP,		/* The DMA did not stop */
	SI_PEP_NET_REP_ABORT,		/* A transfer was aborted for good */
	SI_PEP_NET_REP_TIMEOUT,		/* A transfer timed out */
	SI_PEP_NET_REP_NO_IRQ,		/* The completion interrupt is missing */
	SI_PEP_NET_REP_NOTIFY,		/* The host could not be signalled */
	SI_PEP_NET_REP_LOST,		/* There is no connection to the host */
	SI_PEP_NET_REP_NO_RXBUFS,	/* The host has no receive buffers */
	SI_PEP_NET_REP_BAD_DESC,	/* A descriptor of the host is invalid */
};

static void si_pep_net_note(struct si_pep_net_dev *net, int rep)
{
	/* The number the report is about is stored before it is flagged */
	smp_mb__before_atomic();
	set_bit(rep, &net->reports);
}

static u32 si_pep_net_tx_ints(struct si_pep_dev *pep)
{
	return DMA_REG_READ(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_status);
}

/* With no lock held that the transmit path takes */
static void si_pep_net_report(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	unsigned long reports;

	if (likely(!READ_ONCE(net->reports)))
		return;

	reports = xchg(&net->reports, 0);
	if (!net_ratelimit())
		return;

	if (test_bit(SI_PEP_NET_REP_STOP, &reports))
		si_err(pep, "Net DMA did not stop");
	if (test_bit(SI_PEP_NET_REP_ABORT, &reports))
		si_err(pep, "Net DMA aborted: int status 0x%x",
		       net->report_ints);
	if (test_bit(SI_PEP_NET_REP_TIMEOUT, &reports))
		si_err(pep, "Net DMA timed out: int status 0x%x",
		       net->report_ints);
	if (test_bit(SI_PEP_NET_REP_NO_IRQ, &reports))
		si_err(pep, "Net DMA interrupt missing. Waiting for transfers");
	if (test_bit(SI_PEP_NET_REP_NOTIFY, &reports))
		si_err(pep, "Failed to signal the host: %d", net->report_err);
	if (test_bit(SI_PEP_NET_REP_LOST, &reports))
		si_err(pep, "Connection lost");
	if (test_bit(SI_PEP_NET_REP_NO_RXBUFS, &reports))
		si_err(pep, "Host RX buffers unavailable");
	if (test_bit(SI_PEP_NET_REP_BAD_DESC, &reports))
		si_err(pep, "Invalid packet descriptors");
}

/*
 * Tell the host there is a packet, if it wants to be told.
 *
 * The head must already be published. The host turns its interrupt on and
 * then looks at the head; this publishes the head and then looks at the
 * interrupt enable. Whichever way the two interleave, one side sees what the
 * other wrote, so a packet is never left waiting on a host that has gone to
 * sleep. While the host is busy polling it keeps the enable off and this
 * costs one register read.
 *
 * An MSI can only leave here on the back of a DMA transfer, so the head is
 * copied into the host's view of it, which nothing else writes.
 */
static void si_pep_net_notify_host(struct si_pep_dev *pep, u32 head)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	int ret;

	if (!(net->features & SI_NET_F_RXMSI))
		return;

	/* An MSI skips the state check in the copy, so make it here */
	if (pep->state != SOC_STATE_RUN)
		return;

	if (!net_get_host_rxint(pep))
		return;

	*net->msi_word = head;
	ret = si_pep_copy_to_host_quiet(pep, net->host_hwdata_p +
			offsetof(struct si_net_hwdata, rx_head),
			net->msi_word_p, sizeof(*net->msi_word),
			ETH_DESC_DMA_CH, SI_NET_MSI);
	if (ret) {
		net->report_err = ret;
		si_pep_net_note(net, SI_PEP_NET_REP_NOTIFY);
	}
}

/*
 * The host now has count more packets, in the slots from tx_prod on. Move
 * the head past them. The head is the last slot used plus one, so it reads
 * as the ring size, not zero, when that slot is the last in the ring.
 */
static void si_pep_net_tx_publish(struct si_pep_dev *pep, u32 count)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 n_rxbuf = net->hwdata.n_rxbuf;
	u32 head = net->tx_prod + count;

	if (head > n_rxbuf)
		head -= n_rxbuf;

	net_set_txhead(pep, head);
	net->tx_prod = (head == n_rxbuf) ? 0 : head;
	si_pep_net_notify_host(pep, head);
}

/*
 * Send one packet and wait for it, which is how every packet went before
 * the transfer could be left to finish on its own. It still does where
 * there is no completion interrupt to finish it with.
 */
static int si_pep_net_tx_sync(struct si_pep_dev *pep,
		struct si_pep_net_txjob *job)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	struct si_pep_dma_ll_ctx ll = { 0 };
	int lli;
	int ret;
	int i;

	si_pep_net_fill_desc(net, net->tx_prod, job->skb);
	lli = si_pep_net_ll_add(net, 0, job, net->tx_prod);
	si_pep_net_ll_end(net->tx_ll, net->tx_ll_p, lli);

	ll.dir = DIR_WRITE;
	ll.dma_ll = net->tx_ll;
	ll.p_dma_ll = net->tx_ll_p;
	ll.nents = lli + 1;
	ll.quiet = true;
	for (i = 0; i < SI_PEP_NET_TX_TRIES; i++) {
		ret = si_pep_copy_do_ll(pep, &ll, ETH_DATA_DMA_CH, -1);
		if (ret != -EIO)
			break;
	}
	if (ret)
		return ret;

	si_pep_net_tx_publish(pep, 1);

	return 0;
}

/*
 * Whether a packet may be left to its transfer. It takes the channel's
 * completion interrupt to pick the transfer up again, so not where there
 * is none, and not once it has shown that it does not come.
 */
static bool si_pep_net_tx_async(struct si_pep_net_dev *net)
{
	if (!net_async_tx || !net->tx_irq_on)
		return false;

	if (net->tx_irq_missed >= SI_PEP_NET_TX_IRQ_MISSES) {
		/*
		 * Only an interrupt takes the count back down, and none comes
		 * for a transfer that is not left to it. So the interrupt is
		 * given another chance after a while: one more miss gives it
		 * up again.
		 */
		if (!time_after(jiffies, net->tx_irq_latched +
					SI_PEP_NET_IRQ_RETRY))
			return false;

		net->tx_irq_missed = SI_PEP_NET_TX_IRQ_MISSES - 1;
	}

	return true;
}

/* Start the transfer the transmit linked list describes and return */
static void si_pep_net_tx_kick(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;

	si_pep_net_ll_start(pep, DIR_WRITE, net->tx_ll_p, true);

	net->tx_start = jiffies;
	if (!timer_pending(&net->tx_timer))
		mod_timer(&net->tx_timer, jiffies + SI_PEP_NET_TX_TICK);
}

/*
 * Drop every packet the transmit path holds, stopping the transfer that is
 * carrying some of them if there is one. None of them is published.
 */
static void si_pep_net_tx_flush(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	struct si_pep_net_txjob *job;
	const int ch = ETH_DATA_DMA_CH;
	const int dir = DIR_WRITE;

	if (net->tx_inflight) {
		u32 status;

		DMA_REG_WRITE(pep, ch, dir, doorbell, DMA_DOORBELL_DB_STOP);
		/* The packets are freed below, so the engine must be done with them */
		if (readl_poll_timeout_atomic(DMA_REG_OFFSET(pep, ch, dir, status),
				status, (status & DMA_STATUS_STATUS_MASK) !=
				DMA_STATUS_RUNNING, 1, SI_PEP_NET_TX_STOP_US))
			si_pep_net_note(net, SI_PEP_NET_REP_STOP);
		DMA_REG_WRITE(pep, ch, dir, en, 0);
		DMA_REG_WRITE(pep, ch, dir, int_setup,
				SI_PEP_NET_TX_INT_IDLE);
		DMA_REG_WRITE(pep, ch, dir, int_clear,
				DMA_INT_CLEAR_INTS_MASK);
		net->tx_inflight = 0;
	}

	while (net->txq_tail != net->txq_head) {
		job = &net->txq[net->txq_tail++ & SI_PEP_NET_TXQ_MASK];
		si_pep_net_unmap_job(pep, job);
		DEV_STATS_INC(net->ndev, tx_dropped);
		dev_kfree_skb_any(job->skb);
	}
}

/*
 * The channel is idle. Put as many of the queued packets as one transfer
 * takes into a linked list and start it.
 *
 * A packet gets its slot in the host's ring here and not when it is queued,
 * so that a transfer that fails gives its slots back: the packets behind it
 * are sent into the same ones, and the host never sees a slot that nothing
 * was written to.
 */
static void si_pep_net_tx_next(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	struct si_pep_net_txjob *job;
	u32 count = net->txq_head - net->txq_tail;
	u32 idx = net->tx_prod;
	int lli = 0;
	u32 i;

	if (count == 0)
		return;

	/* As with a transfer that is waited for, nothing goes to a host
	 * that is not there to take it.
	 */
	if (pep->state != SOC_STATE_RUN) {
		si_pep_net_tx_flush(pep);
		return;
	}

	if (count > SI_PEP_NET_TX_BATCH)
		count = SI_PEP_NET_TX_BATCH;

	for (i = 0; i < count; i++) {
		job = &net->txq[(net->txq_tail + i) & SI_PEP_NET_TXQ_MASK];
		si_pep_net_fill_desc(net, idx, job->skb);
		lli = si_pep_net_ll_add(net, lli, job, idx);
		if (++idx == net->hwdata.n_rxbuf)
			idx = 0;
	}
	si_pep_net_ll_end(net->tx_ll, net->tx_ll_p, lli);

	net->tx_inflight = count;
	net->tx_tries = 0;
	si_pep_net_tx_kick(pep);
}

/*
 * The transfer in flight is over. Hand its packets to the host if it went
 * through, release them either way, and start on what queued up behind it.
 */
static void si_pep_net_tx_done(struct si_pep_dev *pep, bool ok)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	struct net_device *ndev = net->ndev;
	struct si_pep_net_txjob *job;
	u32 count = net->tx_inflight;
	u32 i;

	DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_WRITE, en, 0);

	/* Nothing is published to, or signalled at, a host that is not there */
	if (ok && pep->state != SOC_STATE_RUN)
		ok = false;
	if (ok)
		si_pep_net_tx_publish(pep, count);

	for (i = 0; i < count; i++) {
		job = &net->txq[net->txq_tail++ & SI_PEP_NET_TXQ_MASK];
		si_pep_net_unmap_job(pep, job);
		if (ok) {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += job->skb->len;
			dev_consume_skb_any(job->skb);
		} else {
			ndev->stats.tx_errors++;
			dev_kfree_skb_any(job->skb);
		}
	}
	net->tx_inflight = 0;

	si_pep_net_tx_next(pep);
	/* Nothing to wait for: leave the channel unable to interrupt */
	if (!net->tx_inflight)
		DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_setup,
				SI_PEP_NET_TX_INT_IDLE);

	/* A queue stopped for want of ring room is the ring timer's to start */
	if (netif_queue_stopped(ndev) && !net->tx_ring_stopped &&
	    net->txq_head - net->txq_tail <= SI_PEP_NET_TXQ_LEN / 2)
		netif_wake_queue(ndev);
}

/*
 * Look at the transfer in flight and finish it if it has stopped. Returns
 * true if it was finished. An aborted one is started again the number of
 * times the plain transfer would try.
 *
 * This is all the completion interrupt does, and the sender and the timer
 * do it too, so a transfer is picked up by whichever gets there first.
 */
static bool si_pep_net_tx_reap(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 status;

	if (!net->tx_inflight)
		return false;

	status = DMA_REG_READ(pep, ETH_DATA_DMA_CH, DIR_WRITE, status) &
		DMA_STATUS_STATUS_MASK;
	if (status == DMA_STATUS_RUNNING)
		return false;

	if (status == DMA_STATUS_ABORTED) {
		if (++net->tx_tries < SI_PEP_NET_TX_TRIES) {
			si_pep_net_tx_kick(pep);
			return false;
		}

		net->report_ints = si_pep_net_tx_ints(pep);
		si_pep_net_note(net, SI_PEP_NET_REP_ABORT);
	}

	si_pep_net_tx_done(pep, status != DMA_STATUS_ABORTED);

	return true;
}

/*
 * The transmit DMA channel stopped or aborted.
 *
 * Enabled only while there is an interface, so everything it touches is
 * there. The sender or the timer may have finished the transfer already,
 * in which case there is nothing left to do but quieten the line.
 *
 * An interrupt with nothing behind it, no event and no transfer to finish,
 * is reported as not handled. Should the line ever stick, that is what lets
 * the kernel shut it off instead of spinning here, and the timer then
 * carries on finishing transfers.
 */
irqreturn_t si_pep_net_tx_irq(int irq, void *arg)
{
	struct si_pep_dev *pep = arg;
	struct si_pep_net_dev *net = &pep->net_dev;
	bool reaped;
	u32 ints;

	ints = DMA_REG_READ(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_status) &
		DMA_INT_ST_INTS_MASK;
	DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_clear, ints);

	spin_lock(&net->net_tx_lock);
	reaped = si_pep_net_tx_reap(pep);
	if (!net->tx_inflight)
		DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_setup,
				SI_PEP_NET_TX_INT_IDLE);
	/* It came for a transfer, so it is not the interrupt that is late */
	if (reaped || (ints & (DMA_INT_ST_STOP | DMA_INT_ST_ABORT)))
		net->tx_irq_missed = 0;
	spin_unlock(&net->net_tx_lock);
	si_pep_net_report(pep);

	return (reaped || ints) ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * Runs while a transfer is in flight, for the two things the completion
 * interrupt cannot be relied on for.
 *
 * A transfer to a host that has gone away never stops, and gets the time
 * every other transfer gets before it is stopped and its packets dropped.
 *
 * And a transfer that has long been over when this finds it means the
 * interrupt did not come. Once could be an interrupt held off for a while;
 * after a few in a row, packets are sent the old way, which is slower and
 * does not depend on it.
 */
static void si_pep_net_tx_timer(struct timer_list *t)
{
	struct si_pep_net_dev *net = container_of(t, struct si_pep_net_dev,
			tx_timer);
	struct si_pep_dev *pep = container_of(net, struct si_pep_dev,
			net_dev);
	unsigned long flags;
	bool late;

	spin_lock_irqsave(&net->net_tx_lock, flags);
	if (net->tx_inflight) {
		late = time_after(jiffies, net->tx_start + 1);
		if (si_pep_net_tx_reap(pep)) {
			if (late && ++net->tx_irq_missed ==
					SI_PEP_NET_TX_IRQ_MISSES) {
				net->tx_irq_latched = jiffies;
				si_pep_net_note(net, SI_PEP_NET_REP_NO_IRQ);
			}
		} else if (time_after(jiffies,
				net->tx_start + SI_PEP_NET_TX_TIMEOUT)) {
			net->report_ints = si_pep_net_tx_ints(pep);
			si_pep_net_note(net, SI_PEP_NET_REP_TIMEOUT);
			si_pep_net_tx_flush(pep);
			netif_wake_queue(net->ndev);
		}
	}

	if (net->tx_inflight)
		mod_timer(&net->tx_timer, jiffies + SI_PEP_NET_TX_TICK);
	spin_unlock_irqrestore(&net->net_tx_lock, flags);
	si_pep_net_report(pep);
}

/*
 * Give the packets still held a moment to go, then drop what is left. For
 * when the interface goes down, after the stack has stopped sending.
 */
static void si_pep_net_tx_drain(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	unsigned long flags;
	bool idle;
	int i;

	for (i = 0; i < 100; i++) {
		spin_lock_irqsave(&net->net_tx_lock, flags);
		si_pep_net_tx_reap(pep);
		idle = (net->txq_head == net->txq_tail);
		spin_unlock_irqrestore(&net->net_tx_lock, flags);
		if (idle)
			break;

		usleep_range(500, 1000);
	}

	spin_lock_irqsave(&net->net_tx_lock, flags);
	si_pep_net_tx_flush(pep);
	/* Nobody is left to start the queue, and the ring timer must not */
	net->tx_ring_stopped = false;
	spin_unlock_irqrestore(&net->net_tx_lock, flags);
	si_pep_net_report(pep);

	timer_delete_sync(&net->tx_timer);
	hrtimer_cancel(&net->tx_ring_timer);
}

/*
 * How many more packets the host's ring can take, with queued packets
 * already waiting for theirs. The queued packets have no slot yet but each
 * will take one, and one slot is kept back so that a full ring can be told
 * from an empty one.
 */
static int si_pep_net_tx_slack(u32 n_rxbuf, u32 prod, int txtail, u32 queued)
{
	int avail = CIRC_SPACE(prod, txtail, n_rxbuf);

	return avail - 1 - (int)queued;
}

static bool si_pep_net_tx_room(u32 n_rxbuf, u32 prod, int txtail, u32 queued)
{
	return si_pep_net_tx_slack(n_rxbuf, prod, txtail, queued) > 0;
}

/*
 * Whether the ring looks full, without the lock, so that a packet it cannot
 * take is turned away before the slow part of sending one, which is mapping
 * it. Nothing depends on the answer being exact: the check under the lock is
 * the real one. Room only grows. The host moves its tail on, and a transfer
 * that finishes moves tx_prod and the queue by the same amount, so a stale
 * answer can only turn away a packet that would just have fitted, and that
 * one is sent again when the queue is started. A ring that is not set up is
 * left to the checks under the lock, which say what is wrong.
 */
static bool si_pep_net_tx_full(struct si_pep_net_dev *net, int txtail)
{
	/* No lock, on purpose, as above: a stale hint is safe */
	return data_race(net->host_hwdata_p && net->host_rx_descs_p &&
			!si_pep_net_tx_room(net->hwdata.n_rxbuf, net->tx_prod,
				txtail, net->txq_head - net->txq_tail));
}

/*
 * Free slots the ring needs before a queue that was stopped for want of
 * them is started again: a batch, or a quarter of a small ring. Starting at
 * the first free slot would stop and start the queue for every packet the
 * host takes, and waiting for more would leave a small ring too little to
 * run on while the host drains it.
 */
static int si_pep_net_ring_wake_slots(u32 n_rxbuf)
{
	return max_t(int, 1, min_t(u32, SI_PEP_NET_TX_BATCH, n_rxbuf / 4));
}

/*
 * The ring has no room for the packet in hand. Stop the queue, so that the
 * stack holds its packets instead of sending this one again at once for as
 * long as the host is slow, and have the timer look at the ring until the
 * host has taken enough off it. A look is all there is to go on: the host
 * moves its tail with a plain register write and rings nothing.
 *
 * The caller gives the packet back to the stack, which sends it again once
 * the queue is started. Each time the ring fills counts once, in
 * tx_fifo_errors; nothing is dropped. Called with net_tx_lock held.
 */
static void si_pep_net_tx_ring_stop(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;

	netif_stop_queue(net->ndev);
	if (!net->tx_ring_stopped) {
		net->tx_ring_stopped = true;
		net->tx_ring_poll_ns = SI_PEP_NET_RING_POLL_MIN_NS;
		DEV_STATS_INC(net->ndev, tx_fifo_errors);
		si_dbg_net(pep, "No free buffers");
	}

	/* Not if it is waiting already. While its handler runs, it is not */
	if (!hrtimer_is_queued(&net->tx_ring_timer))
		hrtimer_start(&net->tx_ring_timer,
			      ns_to_ktime(net->tx_ring_poll_ns),
			      SI_PEP_NET_RING_TIMER_MODE);
}

/*
 * Look at the ring that stopped the queue. Start the queue if the host has
 * taken enough off it, and look again later if not, after longer each time.
 *
 * It starts itself again with hrtimer_start() and always returns
 * HRTIMER_NORESTART, which is what lets si_pep_net_tx_ring_stop() start it
 * at any moment, even while this runs. The flag and the timer are changed
 * together under the lock, so a stopped queue always has a timer coming.
 */
static enum hrtimer_restart si_pep_net_tx_ring_timer(struct hrtimer *t)
{
	struct si_pep_net_dev *net = container_of(t, struct si_pep_net_dev,
			tx_ring_timer);
	struct si_pep_dev *pep = container_of(net, struct si_pep_dev,
			net_dev);
	u32 n_rxbuf = net->hwdata.n_rxbuf;
	unsigned long flags;
	int slack;

	spin_lock_irqsave(&net->net_tx_lock, flags);
	if (!net->tx_ring_stopped)
		goto out;

	slack = si_pep_net_tx_slack(n_rxbuf, net->tx_prod, net_get_txtail(pep),
				    net->txq_head - net->txq_tail);
	if (slack >= si_pep_net_ring_wake_slots(n_rxbuf)) {
		net->tx_ring_stopped = false;
		netif_wake_queue(net->ndev);
		goto out;
	}

	net->tx_ring_poll_ns = min_t(u32, 2 * net->tx_ring_poll_ns,
				     SI_PEP_NET_RING_POLL_MAX_NS);
	hrtimer_start(t, ns_to_ktime(net->tx_ring_poll_ns),
		      SI_PEP_NET_RING_TIMER_MODE);
out:
	spin_unlock_irqrestore(&net->net_tx_lock, flags);

	return HRTIMER_NORESTART;
}

/*
 * A packet is mapped, queued and, if the DMA channel is idle, started on
 * its way; the sender does not wait for it. The completion interrupt
 * hands it to the host and starts the next transfer with everything that
 * queued up in the meantime, so the channel is kept busy while the sender
 * is busy producing, where the two used to take turns.
 */
static netdev_tx_t si_pep_net_xmit(struct sk_buff *skb,
				   struct net_device *ndev)
{
	struct si_pep_net_pdata *pdata = netdev_priv(ndev);
	struct si_pep_net_dev *net_dev = pdata->net_dev;
	struct si_pep_dev *pep = container_of(net_dev, struct si_pep_dev,
			net_dev);
	struct si_pep_net_txjob job;
	struct si_pkt_desc *pdesc;
	unsigned long flags;
	u32 n_rxbuf = net_dev->hwdata.n_rxbuf;
	u32 queued;
	u32 slot;
	int txtail;
	int ret;

	netif_trans_update(ndev);

	/*
	 * Turn a packet the ring has no room for away before it is mapped, and
	 * stop the queue. The stack would send the packet again at once, and
	 * with the ring full the caches would be cleaned for every try and the
	 * work thrown away. The tail is read once, here, and used by the check
	 * under the lock too: the host only moves it on, so that check can only
	 * be short of room by it. Stopping the queue takes the lock, so a ring
	 * that looks full is looked at once more there.
	 */
	txtail = net_get_txtail(pep);
	if (si_pep_net_tx_full(net_dev, txtail)) {
		spin_lock_irqsave(&net_dev->net_tx_lock, flags);
		txtail = net_get_txtail(pep);
		queued = net_dev->txq_head - net_dev->txq_tail;
		if (!si_pep_net_tx_room(n_rxbuf, net_dev->tx_prod, txtail,
					queued)) {
			si_pep_net_tx_ring_stop(pep);
			spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);
			return NETDEV_TX_BUSY;
		}
		spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);
	}

	/* Cleaning the caches for the packet is the slow part. Do it before
	 * taking the lock the completion interrupt needs.
	 */
	if (si_pep_net_map_skb(pep, skb, &job)) {
		DEV_STATS_INC(ndev, tx_dropped);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	spin_lock_irqsave(&net_dev->net_tx_lock, flags);

	if (net_dev->host_hwdata_p == 0) {
		si_pep_net_note(net_dev, SI_PEP_NET_REP_LOST);
		netif_stop_queue(ndev);
		goto error;
	}

	if (net_dev->host_rx_descs_p == 0) {
		si_pep_net_note(net_dev, SI_PEP_NET_REP_NO_RXBUFS);
		netif_stop_queue(ndev);
		goto error;
	}

	si_pep_net_tx_reap(pep);

	queued = net_dev->txq_head - net_dev->txq_tail;
	if (queued >= SI_PEP_NET_TXQ_LEN) {
		netif_stop_queue(ndev);
		goto error;
	}

	if (!si_pep_net_tx_room(n_rxbuf, net_dev->tx_prod, txtail, queued)) {
		si_pep_net_tx_ring_stop(pep);
		goto error;
	}

	slot = net_dev->tx_prod + queued;
	if (slot >= n_rxbuf)
		slot -= n_rxbuf;
	pdesc = &net_dev->tx_descs[slot];
	if (pdesc->maxlen == 0 || pdesc->dma_addr == 0) {
		/*
		 * Not given back: the queue keeps running, and the stack would
		 * send the same packet again at once for as long as the host
		 * leaves the slot like that.
		 */
		si_pep_net_note(net_dev, SI_PEP_NET_REP_BAD_DESC);
		spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);
		si_pep_net_unmap_job(pep, &job);
		DEV_STATS_INC(ndev, tx_errors);
		DEV_STATS_INC(ndev, tx_dropped);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (skb->len > pdesc->maxlen) {
		/* Longer than the host's buffer: would be copied past its end */
		spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);
		si_pep_net_unmap_job(pep, &job);
		DEV_STATS_INC(ndev, tx_dropped);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (queued == 0 && !si_pep_net_tx_async(net_dev)) {
		ret = si_pep_net_tx_sync(pep, &job);
		spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);
		si_pep_net_unmap_job(pep, &job);
		if (ret) {
			/* The host buffer holds nothing whole. Not published */
			DEV_STATS_INC(ndev, tx_dropped);
			dev_kfree_skb_any(skb);
		} else {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += skb->len;
			dev_consume_skb_any(skb);
		}
		return NETDEV_TX_OK;
	}

	net_dev->txq[net_dev->txq_head++ & SI_PEP_NET_TXQ_MASK] = job;
	if (!net_dev->tx_inflight)
		si_pep_net_tx_next(pep);

	/* Woken again once the transfers have made room */
	if (net_dev->txq_head - net_dev->txq_tail >= SI_PEP_NET_TXQ_LEN)
		netif_stop_queue(ndev);

	spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);

	return NETDEV_TX_OK;
error:
	spin_unlock_irqrestore(&net_dev->net_tx_lock, flags);
	si_pep_net_unmap_job(pep, &job);
	/* Given back to the stack to send again, so nothing is dropped */
	return NETDEV_TX_BUSY;
}

static netdev_tx_t si_pep_net_tx(struct sk_buff *skb, struct net_device *ndev)
{
	struct si_pep_net_pdata *pdata = netdev_priv(ndev);
	struct si_pep_dev *pep = container_of(pdata->net_dev, struct si_pep_dev,
			net_dev);
	netdev_tx_t ret;

	ret = si_pep_net_xmit(skb, ndev);
	si_pep_net_report(pep);

	return ret;
}

/*
 * Receive, where a doorbell says the host has queued packets.
 *
 * The poll does all of it, in three steps that overlap. It takes the
 * descriptors of the packets the host has published, a batch at a time, and
 * starts one DMA linked list that copies every packet of the batch into an
 * skb of its own. It does not wait for that: it hands up the packets the
 * previous transfer brought, and the transfer's completion interrupt brings
 * it back to do the same again. So the engine copies one batch while the
 * stack takes the one before, where every packet used to be waited for in
 * turn before any was handed up.
 *
 * All of the state is the poll's alone. The interrupts and the timer only
 * schedule it.
 */

/*
 * Whether a batch of this many bytes is left to its transfer. It takes the
 * channel's completion interrupt to bring the poll back, so not where there
 * is none, and not once it has shown that it does not come. And a transfer
 * of a packet or two of acknowledgements is not worth an interrupt.
 *
 * Asked once for a batch, when it is fetched. Starting the transfer and
 * deciding whether to wait for it must go by the same answer, and the
 * parameter can change under them.
 */
static bool si_pep_net_rx_async(struct si_pep_net_dev *net, u32 bytes)
{
	if (!net_async_rx || !net->rx_irq_on ||
	    bytes <= SI_PEP_NET_RX_WAIT_BYTES)
		return false;

	if (net->rx_irq_missed >= SI_PEP_NET_TX_IRQ_MISSES) {
		/* As for the transmit side: only an interrupt can clear it */
		if (!time_after(jiffies, net->rx_irq_latched +
					SI_PEP_NET_IRQ_RETRY))
			return false;

		net->rx_irq_missed = SI_PEP_NET_TX_IRQ_MISSES - 1;
	}

	return true;
}

/*
 * How many packets the host has published from rx_next on, in one run that
 * stops at the end of the ring.
 *
 * The head is the last slot the host used plus one, so it reads as the ring
 * size when that was the last slot. Zero is what it holds before anything
 * is sent and nothing can be taken from it, with one exception: a Windows
 * host older than its fix publishes zero after it has used the last slot.
 * That packet used to wait there until the next one was sent.
 */
static u32 si_pep_net_rx_avail(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 n = net->hwdata.n_txbuf;
	u32 head = net_get_rxhead(pep);
	u32 next = net->rx_next;

	if (head > n)
		return 0;

	if (head == 0)
		return (n > 1 && next == n - 1) ? 1 : 0;

	if (head == n)
		head = 0;

	if (head == next)
		return 0;

	return (head > next) ? head - next : n - next;
}

/* The host can have the next count slots back */
static void si_pep_net_rx_advance(struct si_pep_dev *pep, u32 count)
{
	struct si_pep_net_dev *net = &pep->net_dev;

	net->rx_next += count;
	if (net->rx_next >= net->hwdata.n_txbuf)
		net->rx_next = 0;

	net_set_rxtail(pep, net->rx_next);
}

static void si_pep_net_rx_kick(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;

	WRITE_ONCE(net->rx_irq_seen, false);
	WRITE_ONCE(net->rx_timer_fired, false);
	si_pep_net_ll_start(pep, DIR_READ, net->rx_ll_p, !net->rx_wait);

	net->rx_start = jiffies;
	if (!net->rx_wait && !timer_pending(&net->rx_timer))
		mod_timer(&net->rx_timer, jiffies + SI_PEP_NET_TX_TICK);
}

/*
 * The transfer in flight is over. Its packets are ready for the stack if it
 * went through and are dropped if it did not; either way the host gets the
 * slots back.
 */
static void si_pep_net_rx_finish(struct si_pep_dev *pep, bool ok)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	struct net_device *ndev = net->ndev;
	struct si_pep_net_rxjob *job;
	struct sk_buff *skb;
	u32 i;

	DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_READ, en, 0);
	DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_READ, int_setup,
			SI_PEP_NET_TX_INT_IDLE);

	for (i = 0; i < net->rx_njobs; i++) {
		job = &net->rxq[i];
		skb = job->skb;
		dma_unmap_single(pep->dev, job->maddr, job->len,
				DMA_FROM_DEVICE);
		if (!ok) {
			ndev->stats.rx_errors++;
			dev_kfree_skb_any(skb);
			continue;
		}

		skb->dev = ndev;
		/*
		 * The link does not corrupt, so nothing needs verifying.
		 * But a packet the host left without its L4 checksum has to
		 * be marked as such rather than as verified, or it would go
		 * out that way if it is forwarded through another interface.
		 */
		if (!(net->features & SI_NET_F_HOST_PARTIAL_CSUM) ||
		    !(job->csum_flags & SI_PKT_F_CSUM_PARTIAL) ||
		    !skb_partial_csum_set(skb, job->csum_start,
					  job->csum_offset))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb->protocol = eth_type_trans(skb, ndev);
		__skb_queue_tail(&net->rx_ready, skb);
	}

	si_pep_net_rx_advance(pep, net->rx_inflight);
	net->rx_inflight = 0;
	net->rx_njobs = 0;
}

/*
 * Look at the transfer in flight and finish it if it has stopped. Returns
 * true if there is none in flight any more. An aborted one is started again
 * the number of times the plain transfer would try, and one that has run
 * for as long as any transfer in the driver may is stopped.
 */
static bool si_pep_net_rx_reap(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	const int ch = ETH_DATA_DMA_CH;
	const int dir = DIR_READ;
	u32 status;

	if (!net->rx_inflight)
		return true;

	status = DMA_REG_READ(pep, ch, dir, status) & DMA_STATUS_STATUS_MASK;
	if (status == DMA_STATUS_RUNNING) {
		if (!time_after(jiffies, net->rx_start + SI_PEP_NET_TX_TIMEOUT))
			return false;

		si_err(pep, "Net receive DMA timed out: int status 0x%x",
				DMA_REG_READ(pep, ch, dir, int_status));
		DMA_REG_WRITE(pep, ch, dir, doorbell, DMA_DOORBELL_DB_STOP);
		DMA_REG_WRITE(pep, ch, dir, int_clear,
				DMA_INT_CLEAR_INTS_MASK);
		si_pep_net_rx_finish(pep, false);
		return true;
	}

	if (status == DMA_STATUS_ABORTED) {
		if (++net->rx_tries < SI_PEP_NET_TX_TRIES) {
			si_pep_net_rx_kick(pep);
			return false;
		}

		if (net_ratelimit())
			si_err(pep, "Net receive DMA aborted: int status 0x%x",
					DMA_REG_READ(pep, ch, dir,
						int_status));
	}

	/*
	 * The timer only finds a transfer overdue when its interrupt has not
	 * brought the poll back. Once could be an interrupt held off for a
	 * while; after a few in a row, every batch is waited for instead.
	 */
	if (READ_ONCE(net->rx_irq_seen)) {
		net->rx_irq_missed = 0;
	} else if (READ_ONCE(net->rx_timer_fired) &&
		   ++net->rx_irq_missed == SI_PEP_NET_TX_IRQ_MISSES) {
		net->rx_irq_latched = jiffies;
		si_err(pep,
		       "Net RX DMA interrupt missing. Waiting for transfers");
	}

	si_pep_net_rx_finish(pep, status != DMA_STATUS_ABORTED);

	return true;
}

/*
 * Take the next batch of packets the host has published: fetch their
 * descriptors, give each an skb, and start the transfer that fills them.
 * Returns false if there was nothing to take.
 *
 * A slot whose packet cannot be received, for want of an skb or because its
 * descriptor makes no sense, is given back to the host with the rest of
 * the batch and the packet is dropped.
 */
static bool si_pep_net_rx_fetch(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	struct net_device *ndev = net->ndev;
	struct device *dev = pep->dev;
	struct si_pep_net_rxjob *job;
	struct si_pkt_desc *pdesc;
	struct sk_buff *skb;
	size_t maxlen = net->hwdata.mtu + sizeof(struct ethhdr);
	size_t off = net->rx_next * sizeof(*pdesc);
	u32 count = si_pep_net_rx_avail(pep);
	u32 njobs = 0;
	u32 bytes = 0;
	dma_addr_t src;
	u32 len;
	u32 i;

	if (count == 0)
		return false;

	if (count > SI_PEP_NET_RX_BATCH)
		count = SI_PEP_NET_RX_BATCH;

	if (si_pep_copy_from_host(pep, net->rx_descs_p + off,
				net->host_tx_descs_p + off,
				count * sizeof(*pdesc), ETH_DESC_DMA_CH)) {
		/* Nothing is taken from a host that is not there */
		if (pep->state != SOC_STATE_RUN)
			return false;

		/* Without their lengths the packets cannot be told apart */
		ndev->stats.rx_errors += count;
		si_pep_net_rx_advance(pep, count);
		return true;
	}

	for (i = 0; i < count; i++) {
		pdesc = &net->rx_descs[net->rx_next + i];
		len = pdesc->len;
		src = pdesc->dma_addr;
		if (len < sizeof(struct ethhdr) || len > maxlen || src == 0) {
			ndev->stats.rx_length_errors++;
			continue;
		}

		skb = napi_alloc_skb(&net->napi, len);
		if (skb == NULL) {
			ndev->stats.rx_dropped++;
			continue;
		}

		job = &net->rxq[njobs];
		job->maddr = dma_map_single(dev, skb_put(skb, len), len,
				DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, job->maddr)) {
			ndev->stats.rx_dropped++;
			dev_kfree_skb_any(skb);
			continue;
		}

		job->skb = skb;
		job->len = len;
		job->csum_start = pdesc->csum_start;
		job->csum_offset = pdesc->csum_offset;
		job->csum_flags = pdesc->csum_flags;
		si_pep_net_ll_set(&net->rx_ll[njobs], src, job->maddr, len);
		bytes += len;
		njobs++;
	}

	if (njobs == 0) {
		si_pep_net_rx_advance(pep, count);
		return true;
	}

	si_pep_net_ll_end(net->rx_ll, net->rx_ll_p, njobs);
	net->rx_inflight = count;
	net->rx_njobs = njobs;
	net->rx_wait = !si_pep_net_rx_async(net, bytes);
	net->rx_tries = 0;
	si_pep_net_rx_kick(pep);

	return true;
}

/*
 * Move the receive side on: finish the transfer that is over and start the
 * next one. Where a transfer cannot be left to its interrupt it is waited
 * for, and the next started, until there is a poll's worth to hand up.
 */
static void si_pep_net_rx_pump(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	unsigned long start = jiffies;

	for (;;) {
		if (!si_pep_net_rx_reap(pep))
			return;

		if (skb_queue_len(&net->rx_ready) >= SI_PEP_NET_RX_READY_MAX)
			return;

		/*
		 * A transfer that had to be timed out took seconds, and the
		 * next one may well do the same. Do not string them together
		 * in one poll: go back, and let the poll be scheduled again.
		 */
		if (jiffies != start)
			return;

		if (!si_pep_net_rx_fetch(pep))
			return;

		if (net->rx_inflight && !net->rx_wait)
			return;

		while (!si_pep_net_rx_reap(pep))
			cpu_relax();
	}
}

static int si_pep_net_rx_poll(struct napi_struct *napi, int budget)
{
	struct si_pep_net_dev *net = container_of(napi, struct si_pep_net_dev,
			napi);
	struct si_pep_dev *pep = container_of(net, struct si_pep_dev, net_dev);
	struct net_device *ndev = net->ndev;
	struct sk_buff *skb;
	int work = 0;

	si_pep_net_rx_pump(pep);

	while (work < budget && (skb = __skb_dequeue(&net->rx_ready)) != NULL) {
		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += skb->len + sizeof(struct ethhdr);
		napi_gro_receive(napi, skb);
		work++;
	}

	if (work == budget)
		return budget;

	if (!napi_complete_done(napi, work))
		return work;

	/* Its completion interrupt, or the timer, brings the poll back */
	if (net->rx_inflight)
		return work;

	/*
	 * Turn the doorbell interrupt back on, then look at the head once
	 * more. The host writes the head and then reads the interrupt mask,
	 * and does not ring while it is masked. A packet it published while
	 * the poll was running therefore raised nothing, and nothing would
	 * until it sent another; that last look is what finds it. One
	 * published after it rings the doorbell.
	 */
	pep->drv_ops->net_enable_rxint(pep);
	if (pep->state == SOC_STATE_RUN && si_pep_net_rx_avail(pep) &&
	    napi_schedule_prep(napi)) {
		pep->drv_ops->net_disable_rxint(pep);
		__napi_schedule(napi);
	}

	return work;
}

/*
 * The host rang the doorbell for a packet. Its interrupt is masked by now.
 *
 * The doorbell is only unmasked while the interface is up, but the host
 * rings by reading the mask and writing it back, which can undo a mask set
 * in between. So this can be called when there is no interface, and its
 * napi context is then not one that may be scheduled. The doorbell stays
 * masked in that case, and opening the interface looks for the packet.
 */
void si_pep_net_rx_schedule(struct si_pep_dev *pep)
{
	if (READ_ONCE(pep->net_rx_up))
		napi_schedule(&pep->net_dev.napi);
}

/*
 * The receive DMA channel stopped or aborted. Enabled only while there is
 * an interface. An interrupt with no event behind it is reported as not
 * handled, so that the kernel can shut the line off should it ever stick.
 */
irqreturn_t si_pep_net_rx_irq(int irq, void *arg)
{
	struct si_pep_dev *pep = arg;
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 ints;

	ints = DMA_REG_READ(pep, ETH_DATA_DMA_CH, DIR_READ, int_status) &
		DMA_INT_ST_INTS_MASK;
	DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_READ, int_clear, ints);

	if (ints & (DMA_INT_ST_STOP | DMA_INT_ST_ABORT))
		WRITE_ONCE(net->rx_irq_seen, true);
	napi_schedule(&net->napi);

	return ints ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * Runs while a receive transfer is in flight and schedules the poll, which
 * is what notices a transfer that never stops, or one whose completion
 * interrupt did not come.
 */
static void si_pep_net_rx_timer(struct timer_list *t)
{
	struct si_pep_net_dev *net = container_of(t, struct si_pep_net_dev,
			rx_timer);

	if (!READ_ONCE(net->rx_inflight))
		return;

	if (time_after(jiffies, READ_ONCE(net->rx_start) + 1))
		WRITE_ONCE(net->rx_timer_fired, true);
	napi_schedule(&net->napi);
	mod_timer(&net->rx_timer, jiffies + SI_PEP_NET_TX_TICK);
}

/*
 * The interface is going down and the poll will not run again. Let the
 * transfer in flight finish, since the engine has to be idle, and drop
 * what it and the poll were holding. The packets of that transfer are
 * still the host's and are taken again when the interface comes back.
 */
static void si_pep_net_rx_stop(struct si_pep_dev *pep)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	const int ch = ETH_DATA_DMA_CH;
	const int dir = DIR_READ;
	u32 i;

	if (net->rx_inflight) {
		for (i = 0; i < 100; i++) {
			if ((DMA_REG_READ(pep, ch, dir, status) &
			     DMA_STATUS_STATUS_MASK) != DMA_STATUS_RUNNING)
				break;
			usleep_range(500, 1000);
		}
		if (i == 100)
			DMA_REG_WRITE(pep, ch, dir, doorbell,
					DMA_DOORBELL_DB_STOP);
		DMA_REG_WRITE(pep, ch, dir, en, 0);
		DMA_REG_WRITE(pep, ch, dir, int_setup, SI_PEP_NET_TX_INT_IDLE);
		DMA_REG_WRITE(pep, ch, dir, int_clear,
				DMA_INT_CLEAR_INTS_MASK);

		for (i = 0; i < net->rx_njobs; i++) {
			dma_unmap_single(pep->dev, net->rxq[i].maddr,
					net->rxq[i].len, DMA_FROM_DEVICE);
			dev_kfree_skb_any(net->rxq[i].skb);
		}
		WRITE_ONCE(net->rx_inflight, 0);
		net->rx_njobs = 0;
	}

	__skb_queue_purge(&net->rx_ready);
	timer_delete_sync(&net->rx_timer);
}

static int si_pep_net_open(struct net_device *ndev)
{
	struct si_pep_net_pdata *pdata = netdev_priv(ndev);
	struct si_pep_net_dev *net_dev = pdata->net_dev;
	struct si_pep_dev *pep = container_of(net_dev, struct si_pep_dev,
			net_dev);

	si_pep_net_get_txdescs(net_dev);
	if (net_dev->host_hwdata_p)
		netif_start_queue(ndev);

	napi_enable(&net_dev->napi);
	pep->drv_ops->net_open(pep);
	WRITE_ONCE(pep->net_rx_up, true);
	pep->drv_ops->net_enable_rxint(pep);
	/*
	 * A packet the host published while the interface was down rang
	 * nothing. The poll takes it, and ends by looking once more.
	 */
	if (pep->modalix) {
		local_bh_disable();
		si_pep_net_rx_schedule(pep);
		local_bh_enable();
	}

	return 0;
}

int si_pep_net_release(struct net_device *ndev)
{
	struct si_pep_net_pdata *pdata = netdev_priv(ndev);
	struct si_pep_net_dev *net_dev = pdata->net_dev;
	struct si_pep_dev *pep = container_of(net_dev, struct si_pep_dev,
			net_dev);

	pep->drv_ops->net_disable_rxint(pep);
	pep->drv_ops->net_release(pep);
	napi_disable(&net_dev->napi);
	WRITE_ONCE(pep->net_rx_up, false);
	/* A poll that was finishing may have turned the doorbell back on */
	pep->drv_ops->net_disable_rxint(pep);
	if (pep->modalix)
		si_pep_net_rx_stop(pep);
	netif_stop_queue(ndev);
	si_pep_net_tx_drain(pep);
	/* A transfer that finished while draining may have woken it */
	netif_stop_queue(ndev);

	return 0;
}

static const struct net_device_ops si_pep_net_dev_ops = {
	.ndo_open            = si_pep_net_open,
	.ndo_stop            = si_pep_net_release,
	.ndo_start_xmit      = si_pep_net_tx,
};

void si_pep_net_cleanup(struct si_pep_dev *pep)
{
	struct device *dev = pep->dev;
	struct si_pep_net_dev *net_dev = &pep->net_dev;

	pep->drv_ops->net_release(pep);

	if (net_dev->ndev) {
		/* Takes the interface down, which empties the transmit path */
		unregister_netdev(net_dev->ndev);
		if (net_dev->tx_irq_on) {
			disable_irq(pep->net_tx_irq);
			net_dev->tx_irq_on = false;
		}
		if (net_dev->rx_irq_on) {
			disable_irq(pep->net_rx_irq);
			net_dev->rx_irq_on = false;
		}
		timer_delete_sync(&net_dev->tx_timer);
		hrtimer_cancel(&net_dev->tx_ring_timer);
		timer_delete_sync(&net_dev->rx_timer);
		free_netdev(net_dev->ndev);
		net_dev->ndev = NULL;
	}

	kfree(net_dev->txq);
	net_dev->txq = NULL;

	si_dbg_net(pep, "Freeing descriptors at %px, 0x%llx, Size: %lu",
			net_dev->desc_vaddr, net_dev->desc_paddr,
			net_dev->desc_size);

	if (net_dev->desc_vaddr != NULL) {
		dma_free_coherent(dev, net_dev->desc_size, net_dev->desc_vaddr,
				net_dev->desc_paddr);
		net_dev->desc_vaddr = NULL;
		net_dev->desc_paddr = 0;
		net_dev->desc_size = 0;
	}

	memset(&net_dev->hwdata, 0, sizeof(struct si_net_hwdata));
	net_dev->host_hwdata_p = 0;
	net_set_rxtail(pep, 0);
	net_set_txhead(pep, 0);
}

/*
 * What of the host's offer this end takes up.
 *
 * Only Modalix has a register for the host's interrupt enable, and without
 * one there is no telling whether an MSI is wanted.
 */
static u32 si_pep_net_features(struct si_pep_dev *pep, u32 offered)
{
	u32 features = SI_NET_F_PARTIAL_CSUM | SI_NET_F_HOST_PARTIAL_CSUM;

	if (pep->modalix && si_pep_msi_has_vector(pep, SI_NET_MSI))
		features |= SI_NET_F_RXMSI;

	return offered & features;
}

int si_pep_net_init(struct si_pep_dev *pep, dma_addr_t haddr, u32 features)
{
	int ret;
	struct device *dev = pep->dev;
	struct si_pep_net_pdata *pdata;
	struct si_pep_net_dev *net_dev = &pep->net_dev;
	struct si_net_hwdata *hwdata;
	dma_addr_t hwdata_p;
	dma_addr_t descs_base;
	size_t txdesc_size;
	size_t rxdesc_size;
	size_t offset;
	size_t ll_offset;
	size_t ll_size;
	size_t rx_ll_size;

	memset(net_dev, 0, sizeof(struct si_pep_net_dev));
	net_dev->host_hwdata_p = haddr;
	net_dev->features = si_pep_net_features(pep, features);
	spin_lock_init(&net_dev->net_tx_lock);
	timer_setup(&net_dev->tx_timer, si_pep_net_tx_timer, 0);
	hrtimer_setup(&net_dev->tx_ring_timer, si_pep_net_tx_ring_timer,
		      CLOCK_MONOTONIC, SI_PEP_NET_RING_TIMER_MODE);
	timer_setup(&net_dev->rx_timer, si_pep_net_rx_timer, 0);
	__skb_queue_head_init(&net_dev->rx_ready);
	pep->drv_ops->net_disable_rxint(pep);
	si_dbg_net(pep, "Initializing network interface");
	hwdata = kzalloc(sizeof(struct si_net_hwdata), GFP_KERNEL);
	if (hwdata == NULL) {
		si_err(pep, "Failed to allocate hardware data");
		return -ENOMEM;
	}

	hwdata_p = dma_map_single(dev, hwdata, sizeof(struct si_net_hwdata),
			DMA_FROM_DEVICE);
	si_dbg_net(pep, "Copy hardware data from 0x%llx",
			net_dev->host_hwdata_p);
	si_pep_copy_from_host(pep, hwdata_p, net_dev->host_hwdata_p,
			sizeof(struct si_net_hwdata), ETH_DESC_DMA_CH);
	dma_unmap_single(dev, hwdata_p, sizeof(struct si_net_hwdata),
			DMA_FROM_DEVICE);
	memcpy(&net_dev->hwdata, hwdata, sizeof(struct si_net_hwdata));
	kfree(hwdata);
	hwdata = &net_dev->hwdata;
	si_dbg_net(pep, "Init network interface. MTU: %u, nTXB: %u, nRXB: %u",
			hwdata->mtu, hwdata->n_txbuf, hwdata->n_rxbuf);

	/* Initialize addresses */
	descs_base = hwdata->tx_desc_high;
	descs_base = (descs_base << 32) | hwdata->tx_desc_low;
	if (descs_base == 0) {
		si_err(pep, "Host TX descriptors not found");
		return -ENOENT;
	}
	net_dev->host_tx_descs_p = descs_base;
	si_dbg_net(pep, "Host TX descriptors are at 0x%llx",
			net_dev->host_tx_descs_p);

	descs_base = hwdata->rx_desc_high;
	descs_base = (descs_base << 32) | hwdata->rx_desc_low;
	if (descs_base == 0) {
		si_err(pep, "Host TX descriptors not found");
		return -ENOENT;
	}
	net_dev->host_rx_descs_p = descs_base;
	si_dbg_net(pep, "Host RX descriptors are at 0x%llx",
			net_dev->host_rx_descs_p);

	txdesc_size = hwdata->n_txbuf * sizeof(struct si_pkt_desc);
	rxdesc_size = hwdata->n_rxbuf * sizeof(struct si_pkt_desc);
	offset = ALIGN(txdesc_size, 4);
	/* The linked lists and the MSI word ride in the same block */
	ll_offset = ALIGN(offset + rxdesc_size, 8);
	ll_size = SI_PEP_NET_TX_LL_ENTS * sizeof(struct si_pep_dma_ll_delem);
	rx_ll_size = SI_PEP_NET_RX_LL_ENTS * sizeof(struct si_pep_dma_ll_delem);
	net_dev->desc_size = ll_offset + ll_size + rx_ll_size +
		sizeof(*net_dev->msi_word);
	si_dbg_net(pep, "Size of host TX descriptors: %lu, RX Descriptors: %lu",
			txdesc_size, rxdesc_size);
	si_dbg_net(pep, "RX Desc start at %lu - 0x%lx", offset, offset);
	/* Allocate dma buffers */
	net_dev->desc_vaddr = dma_alloc_coherent(dev, net_dev->desc_size,
			&net_dev->desc_paddr, GFP_KERNEL);
	if (net_dev->desc_vaddr == NULL) {
		si_err(pep, "Failed to allocate transmit descriptors");
		return -ENOMEM;
	}

	si_dbg_net(pep, "Allocated descriptors at %px, 0x%llx, Size: %lu",
			net_dev->desc_vaddr, net_dev->desc_paddr,
			net_dev->desc_size);
	net_dev->rx_descs = net_dev->desc_vaddr;
	net_dev->rx_descs_p = net_dev->desc_paddr;
	net_dev->tx_descs = net_dev->desc_vaddr + offset;
	net_dev->tx_descs_p = net_dev->desc_paddr + offset;
	net_dev->tx_ll = net_dev->desc_vaddr + ll_offset;
	net_dev->tx_ll_p = net_dev->desc_paddr + ll_offset;
	net_dev->rx_ll = net_dev->desc_vaddr + ll_offset + ll_size;
	net_dev->rx_ll_p = net_dev->desc_paddr + ll_offset + ll_size;
	net_dev->msi_word = net_dev->desc_vaddr + ll_offset + ll_size +
		rx_ll_size;
	net_dev->msi_word_p = net_dev->desc_paddr + ll_offset + ll_size +
		rx_ll_size;
	si_dbg_net(pep, "SoC Transmit descriptors at %px, 0x%llx, Size: %lu",
			net_dev->tx_descs, net_dev->tx_descs_p, rxdesc_size);
	si_dbg_net(pep, "SoC Receive descriptors at %px, 0x%llx, Size: %lu",
			net_dev->rx_descs, net_dev->rx_descs_p, txdesc_size);

	net_dev->txq = kcalloc(SI_PEP_NET_TXQ_LEN, sizeof(*net_dev->txq),
			GFP_KERNEL);
	if (net_dev->txq == NULL) {
		ret = -ENOMEM;
		si_err(pep, "Failed to allocate the transmit queue");
		goto err_out;
	}

	/* Allocate the devices */
	net_dev->ndev = alloc_etherdev(sizeof(struct si_pep_net_pdata));
	if (net_dev->ndev == NULL) {
		ret = -ENOMEM;
		si_err(pep, "Failed to allocate ethernet device");
		goto err_out;
	}

	SET_NETDEV_DEV(net_dev->ndev, dev);
	net_dev->ndev->mtu = hwdata->mtu;
	net_dev->ndev->min_mtu = hwdata->mtu;
	net_dev->ndev->max_mtu = hwdata->mtu;
	net_dev->ndev->netdev_ops = &si_pep_net_dev_ops;
	/*
	 * Fragments are sent as they are, whoever the host is. The checksum
	 * is another matter: the host has to have said it takes a packet
	 * without one, since a host that has not would drop every packet.
	 */
	net_dev->ndev->hw_features = NETIF_F_SG;
	if (net_dev->features & SI_NET_F_PARTIAL_CSUM)
		net_dev->ndev->hw_features |= NETIF_F_HW_CSUM;
	net_dev->ndev->features |= net_dev->ndev->hw_features;
	memcpy(&si_pep_net_dev_addr[0], "\0SIMA1", ETH_ALEN);
	si_pep_net_dev_addr[0] = (u8)pep->card_num;
	eth_hw_addr_set(net_dev->ndev, &si_pep_net_dev_addr[0]);
	net_dev->ndev->watchdog_timeo = msecs_to_jiffies(2500);
	netif_napi_add_weight(net_dev->ndev, &net_dev->napi, si_pep_net_poll, 64);

	/*
	 * The transmit channel's completion interrupt is kept disabled while
	 * there is no interface to take it. Nothing has used the channel's
	 * interrupts so far, so make sure none is waiting before letting it
	 * through.
	 */
	if (pep->net_tx_irq > 0) {
		DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_setup,
				SI_PEP_NET_TX_INT_IDLE);
		DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_WRITE, int_clear,
				DMA_INT_CLEAR_INTS_MASK);
		enable_irq(pep->net_tx_irq);
		net_dev->tx_irq_on = true;
	}
	if (pep->net_rx_irq > 0) {
		DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_READ, int_setup,
				SI_PEP_NET_TX_INT_IDLE);
		DMA_REG_WRITE(pep, ETH_DATA_DMA_CH, DIR_READ, int_clear,
				DMA_INT_CLEAR_INTS_MASK);
		enable_irq(pep->net_rx_irq);
		net_dev->rx_irq_on = true;
	}

	/*
	 * The interface can be brought up, and can send, the moment it is
	 * registered. Everything that needs must be in place before that:
	 * where its private data points, and the ring positions the host is
	 * shown.
	 */
	pdata = netdev_priv(net_dev->ndev);
	pdata->net_dev = net_dev;
	net_set_rxtail(pep, 0);
	net_set_txhead(pep, 0);

	ret = register_netdev(net_dev->ndev);
	if (ret) {
		si_err(pep, "Failed to register netdev");
		goto err_register;
	}

	return 0;

err_register:
	if (net_dev->tx_irq_on) {
		disable_irq(pep->net_tx_irq);
		net_dev->tx_irq_on = false;
	}
	if (net_dev->rx_irq_on) {
		disable_irq(pep->net_rx_irq);
		net_dev->rx_irq_on = false;
	}
	free_netdev(net_dev->ndev);
	net_dev->ndev = NULL;
err_out:
	kfree(net_dev->txq);
	net_dev->txq = NULL;
	dma_free_coherent(dev, net_dev->desc_size, net_dev->desc_vaddr,
			net_dev->desc_paddr);
	net_dev->desc_vaddr = NULL;
	net_dev->desc_paddr = 0;
	net_dev->desc_size = 0;
	
	return ret;
}

