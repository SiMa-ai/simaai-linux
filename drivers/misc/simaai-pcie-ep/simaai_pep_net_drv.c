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

#include "simaai_pep_drv.h"

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

	si_dbg_net(pep, "Copying %u packets from %u", npkts, start);

	i = start;
	while (npkts) {
		pdesc = &net->rx_descs[i];
		len = pdesc->len;
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
		return PTR_ERR(pdesc->skb);
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

static int si_pep_net_tx(struct sk_buff *skb, struct net_device *ndev)
{
	struct si_pep_net_pdata *pdata = netdev_priv(ndev);
	struct si_pep_net_dev *net_dev = pdata->net_dev;
	struct si_pep_dev *pep = container_of(net_dev, struct si_pep_dev,
			net_dev);
	struct device *dev = pep->dev;
	struct si_pkt_desc *pdesc;
	dma_addr_t dmaaddr;
	int txhead;
	int txtail;
	int avail;

	netif_trans_update(ndev);
	spin_lock_irq(&net_dev->net_tx_lock);

	if (net_dev->host_hwdata_p == 0) {
		si_err(pep, "Connection lost");
		netif_stop_queue(ndev);
		goto error;
	}

	if (net_dev->host_rx_descs_p == 0) {
		si_err(pep, "Host RX buffers unavailable");
		netif_stop_queue(ndev);
		goto error;
	}

	txhead = net_get_txhead(pep);
	txtail = net_get_txtail(pep);
	if (txhead >= net_dev->hwdata.n_rxbuf) {
		txhead = 0;
	}

	avail = CIRC_SPACE(txhead, txtail, net_dev->hwdata.n_rxbuf);
	if (avail <= 1) {
		if (printk_ratelimit())
			si_dbg_net(pep, "No free buffers");
		goto error;
	}

	pdesc = &net_dev->tx_descs[txhead];
	if (pdesc->maxlen == 0 || pdesc->dma_addr == 0) {
		si_err(pep, "Invalid packet descriptors");
		goto error;
	}

	pdesc->len = skb->len;
	/* Copy packet descriptor */
	si_pep_copy_to_host(pep, net_dev->host_rx_descs_p + 
			(sizeof(struct si_pkt_desc) * txhead),
			net_dev->tx_descs_p +
			(sizeof(struct si_pkt_desc) * txhead),
			sizeof(struct si_pkt_desc), ETH_DESC_DMA_CH, -1);
	/* Copy packet data */
	dmaaddr = dma_map_single(dev, skb->data, skb->len, DMA_TO_DEVICE);
	si_pep_copy_to_host(pep, pdesc->dma_addr, dmaaddr, skb->len,
			ETH_DATA_DMA_CH, -1);
	dma_unmap_single(dev, dmaaddr, skb->len, DMA_TO_DEVICE);
	/* Update RX head on host with txhead from soc */
	net_set_txhead(pep, txhead + 1);
	/* All done, unlock and update stats, free skb */
	spin_unlock_irq(&net_dev->net_tx_lock);
	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
error:
	spin_unlock_irq(&net_dev->net_tx_lock);
	ndev->stats.tx_dropped++;
	return NETDEV_TX_BUSY;
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
	pep->drv_ops->net_enable_rxint(pep);

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
		unregister_netdev(net_dev->ndev);
		free_netdev(net_dev->ndev);
		net_dev->ndev = NULL;
	}

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

int si_pep_net_init(struct si_pep_dev *pep, dma_addr_t haddr)
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

	memset(net_dev, 0, sizeof(struct si_pep_net_dev));
	net_dev->host_hwdata_p = haddr;
	spin_lock_init(&net_dev->net_tx_lock);
	pep->drv_ops->net_disable_rxint(pep);
	si_dbg_net(pep, "Initializing network interface");
	hwdata = devm_kzalloc(dev, sizeof(struct si_net_hwdata), GFP_KERNEL);
	if (hwdata == NULL) {
		si_err(pep, "Failed to allocate hardware data");
		return PTR_ERR(hwdata);
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
	devm_kfree(dev, hwdata);
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
	net_dev->desc_size = offset + rxdesc_size;
	si_dbg_net(pep, "Size of host TX descriptors: %lu, RX Descriptors: %lu",
			txdesc_size, rxdesc_size);
	si_dbg_net(pep, "RX Desc start at %lu - 0x%lx", offset, offset);
	/* Allocate dma buffers */
	net_dev->desc_vaddr = dma_alloc_coherent(dev, net_dev->desc_size,
			&net_dev->desc_paddr, GFP_KERNEL);
	if (net_dev->desc_vaddr == NULL) {
		si_err(pep, "Failed to allocate transmit descriptors");
		return PTR_ERR(net_dev->desc_vaddr);
	}

	si_dbg_net(pep, "Allocated descriptors at %px, 0x%llx, Size: %lu",
			net_dev->desc_vaddr, net_dev->desc_paddr,
			net_dev->desc_size);
	net_dev->rx_descs = net_dev->desc_vaddr;
	net_dev->rx_descs_p = net_dev->desc_paddr;
	net_dev->tx_descs = net_dev->desc_vaddr + offset;
	net_dev->tx_descs_p = net_dev->desc_paddr + offset;
	si_dbg_net(pep, "SoC Transmit descriptors at %px, 0x%llx, Size: %lu",
			net_dev->tx_descs, net_dev->tx_descs_p, rxdesc_size);
	si_dbg_net(pep, "SoC Receive descriptors at %px, 0x%llx, Size: %lu",
			net_dev->rx_descs, net_dev->rx_descs_p, txdesc_size);

	/* Allocate the devices */
	net_dev->ndev = alloc_etherdev(sizeof(struct si_pep_net_pdata));
	if (net_dev->ndev == NULL) {
		ret = PTR_ERR(net_dev->ndev);
		si_err(pep, "Failed to allocate ethernet device");
		goto err_out;
	}

	SET_NETDEV_DEV(net_dev->ndev, dev);
	net_dev->ndev->mtu = hwdata->mtu;
	net_dev->ndev->min_mtu = hwdata->mtu;
	net_dev->ndev->max_mtu = hwdata->mtu;
	net_dev->ndev->netdev_ops = &si_pep_net_dev_ops;
	memcpy(&si_pep_net_dev_addr[0], "\0SIMA1", ETH_ALEN);
	si_pep_net_dev_addr[0] = (u8)pep->card_num;
	eth_hw_addr_set(net_dev->ndev, &si_pep_net_dev_addr[0]);
	net_dev->ndev->watchdog_timeo = msecs_to_jiffies(2500);
	netif_napi_add_weight(net_dev->ndev, &net_dev->napi, si_pep_net_poll, 64);

	ret = register_netdev(net_dev->ndev);
	if (ret) {
		si_err(pep, "Failed to register netdev");
		free_netdev(net_dev->ndev);
		goto err_register;
	}

	pdata = netdev_priv(net_dev->ndev);
	pdata->net_dev = net_dev;
	net_set_rxtail(pep, 0);
	net_set_txhead(pep, 0);

	return 0;

err_register:
	free_netdev(net_dev->ndev);
	net_dev->ndev = NULL;
err_out:
	dma_free_coherent(dev, net_dev->desc_size, net_dev->desc_vaddr,
			net_dev->desc_paddr);
	net_dev->desc_vaddr = NULL;
	net_dev->desc_paddr = 0;
	net_dev->desc_size = 0;
	
	return ret;
}

