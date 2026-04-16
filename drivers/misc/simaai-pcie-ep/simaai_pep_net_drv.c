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

#include "simaai_pep_drv.h"

u8 si_pep_net_dev_addr[MAX_ADDR_LEN] = { 0 };

static int si_pep_net_get_txdescs(struct si_pep_net_dev *net)
{
	dma_addr_t tx_descs_base = net->host_rx_descs_p;
	struct si_pep_dev *pep = container_of(net, struct si_pep_dev, net_dev);

	if (net->host_hwdata_p == 0) /* Host hasn't configured us */
		return -ENOENT;

	if (tx_descs_base == 0)
		return -ENOENT;

	si_pep_copy_from_host(pep, net->tx_descs_p, tx_descs_base,
			sizeof(struct si_pkt_desc) * N_RXBUF, ETH_DESC_DMA_CH);

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
	do {
		pdesc = &net->rx_descs[i];
		len = pdesc->len;
		pdesc->skb = dev_alloc_skb(len);
		if (pdesc->skb == NULL) {
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
	} while (npkts);

	return done;
}

int si_pep_check_packets(struct si_pep_dev *pep, u32 head, u32 tail)
{
	struct si_pep_net_dev *net = &pep->net_dev;
	u32 phead = atomic_read(&net->rxphead);

	if (head == 0 || head == phead || head == tail) {
		atomic_set(&net->rx_en, 1);
		return -ENOENT;
	}

	si_dbg_net(pep, "Head: %u, Tail: %u, PHead: %u", head, tail, phead);

	if (tail > head) {
		si_pep_copy_from_host(pep,
				net->rx_descs_p +
				(tail * sizeof(struct si_pkt_desc)),
				net->host_tx_descs_p +
				(tail * sizeof(struct si_pkt_desc)),
				sizeof(struct si_pkt_desc) * (N_RXBUF - tail),
				ETH_DESC_DMA_CH);
		si_pep_net_copy_packets(pep, tail, (N_RXBUF - tail));

		si_pep_copy_from_host(pep,
				net->rx_descs_p, net->host_tx_descs_p,
				sizeof(struct si_pkt_desc) * head,
				ETH_DESC_DMA_CH);
		si_pep_net_copy_packets(pep, 0, head);
	} else {
		si_pep_copy_from_host(pep,
				net->rx_descs_p +
				(tail * sizeof(struct si_pkt_desc)),
				net->host_tx_descs_p +
				(tail * sizeof(struct si_pkt_desc)),
				sizeof(struct si_pkt_desc) * (head - tail),
				ETH_DESC_DMA_CH);
		si_pep_net_copy_packets(pep, tail, head - tail);
	}

	atomic_set(&net->rxphead, head);
	napi_schedule(&net->napi);

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
	rxtail = net_get_rxtail(pep);
	if (rxtail >= N_TXBUF) {
		si_err(pep, "Invalid rx tail");
		rxtail = 0;
		net_set_rxtail(pep, rxtail);
		goto err_out;
	}

	if (rxtail > rxhead) {
		pkts_avail = (N_RXBUF - rxtail) + rxhead;
	} else {
		pkts_avail = rxhead - rxtail;
	}

	if (pkts_avail == 0) {
		si_err(pep, "Packets unavailable. Head:%u, Tail:%u", rxhead,
				rxtail);
		goto err_out;
	}

	si_dbg_net(pep, "Head: %u, Tail: %u, Packets: %d", rxhead, rxtail,
			pkts_avail);

	while (pkts_avail && (pkts_done < budget)) {
		pdesc = &net->rx_descs[rxtail];
		if (pdesc->skb == NULL) {
			ndev->stats.rx_dropped++;
			pkts_avail--;
		} else {
			napi_gro_receive(napi, pdesc->skb);
			ndev->stats.rx_packets++;
			ndev->stats.rx_bytes += pdesc->len;
			pkts_done++;
			pkts_avail--;
		}

		rxtail++;
		if (rxtail == N_RXBUF)
			rxtail = 0;

		net_set_rxtail(pep, rxtail);
	}

	if (pkts_done < budget) {
err_out:
		napi_complete_done(napi, pkts_done);
		atomic_set(&net->rx_en, 1);
	}

	si_dbg_net(pep, "Processed %d packets", pkts_done);
	return pkts_done;
}

int si_pep_net_tx(struct sk_buff *skb, struct net_device *ndev)
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
	if (txhead >= N_TXBUF) {
		txhead = 0;
	}

	avail = CIRC_SPACE(txhead, txtail, N_TXBUF);
	if (avail <= 1) {
		if (printk_ratelimit())
			si_err(pep, "No free buffers");
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

int si_pep_net_open(struct net_device *ndev)
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
	atomic_set(&net_dev->rx_en, 1);

	return 0;
}

int si_pep_net_release(struct net_device *ndev)
{
	struct si_pep_net_pdata *pdata = netdev_priv(ndev);
	struct si_pep_net_dev *net_dev = pdata->net_dev;
	struct si_pep_dev *pep = container_of(net_dev, struct si_pep_dev,
			net_dev);

	atomic_set(&net_dev->rx_en, 0);
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

	dma_free_coherent(dev, sizeof(struct si_pkt_desc) * N_RXBUF,
			net_dev->rx_descs, net_dev->rx_descs_p);
	dma_free_coherent(dev, sizeof(struct si_pkt_desc) * N_TXBUF,
			net_dev->tx_descs, net_dev->tx_descs_p);
	dma_free_coherent(dev, sizeof(struct si_net_hwdata),
			net_dev->hwdata, net_dev->hwdata_p);
}

int si_pep_net_init(struct si_pep_dev *pep)
{
	int ret;
	struct device *dev = pep->dev;
	struct si_pep_net_pdata *pdata;
	struct si_pep_net_dev *net_dev = &pep->net_dev;
	dma_addr_t rx_descs_base;
	dma_addr_t tx_descs_base;

	spin_lock_init(&net_dev->net_tx_lock);
	atomic_set(&net_dev->rx_en, 0);

	/* Allocate dma buffers */
	net_dev->tx_descs = dma_alloc_coherent(dev, sizeof(struct si_pkt_desc) *
			N_TXBUF, &net_dev->tx_descs_p, GFP_KERNEL);
	if (net_dev->tx_descs == NULL) {
		si_err(pep, "Failed to allocate TX descriptors");
		return PTR_ERR(net_dev->tx_descs);
	}

	net_dev->rx_descs = dma_alloc_coherent(dev, sizeof(struct si_pkt_desc) *
			N_RXBUF, &net_dev->rx_descs_p, GFP_KERNEL);
	if (net_dev->rx_descs == NULL) {
		ret = PTR_ERR(net_dev->rx_descs);
		si_err(pep, "Failed to allocate RX descriptors");
		goto err_out_rx_desc;
	}

	net_dev->hwdata = dma_alloc_coherent(dev, sizeof(struct si_net_hwdata),
			&net_dev->hwdata_p, GFP_KERNEL);
	if (net_dev->hwdata == NULL) {
		ret = PTR_ERR(net_dev->hwdata);
		si_err(pep, "Failed to allocate hardware data");
		goto err_out_hwdata;
	}

	/* Allocate the devices */
	net_dev->ndev = alloc_etherdev(sizeof(struct si_pep_net_pdata));
	if (net_dev->ndev == NULL) {
		ret = PTR_ERR(net_dev->ndev);
		si_err(pep, "Failed to allocate ethernet device");
		goto err_out;
	}

	SET_NETDEV_DEV(net_dev->ndev, dev);
	net_dev->ndev->mtu = SIMA_NET_MTU;
	net_dev->ndev->min_mtu = SIMA_NET_MTU;
	net_dev->ndev->max_mtu = SIMA_NET_MTU;
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
		goto err_out;
	}

	pdata = netdev_priv(net_dev->ndev);
	pdata->net_dev = net_dev;

	/* Initialize addresses */
	si_pep_copy_from_host(pep, net_dev->hwdata_p, net_dev->host_hwdata_p,
			sizeof(struct si_net_hwdata), ETH_DESC_DMA_CH);
	rx_descs_base = net_dev->hwdata->tx_desc_high;
	rx_descs_base = (rx_descs_base << 32) | net_dev->hwdata->tx_desc_low;
	if (rx_descs_base == 0) {
		si_err(pep, "Host RX descriptors not found");
		ret = -ENOENT;
		goto err_setup;
	}
	net_dev->host_tx_descs_p = rx_descs_base;

	tx_descs_base = net_dev->hwdata->rx_desc_high;
	tx_descs_base = (tx_descs_base << 32) | net_dev->hwdata->rx_desc_low;
	if (tx_descs_base == 0) {
		si_err(pep, "Host TX descriptors not found");
		ret = -ENOENT;
		goto err_setup;
	}
	net_dev->host_rx_descs_p = tx_descs_base;

	return 0;

err_setup:
	unregister_netdev(net_dev->ndev);
	free_netdev(net_dev->ndev);
	net_dev->ndev = NULL;
err_out:
	dma_free_coherent(dev, sizeof(struct si_net_hwdata),
			net_dev->hwdata, net_dev->hwdata_p);
err_out_hwdata:
	dma_free_coherent(dev, sizeof(struct si_pkt_desc) * N_TXBUF,
			net_dev->rx_descs, net_dev->rx_descs_p);
err_out_rx_desc:
	dma_free_coherent(dev, sizeof(struct si_pkt_desc) * N_RXBUF,
			net_dev->tx_descs, net_dev->tx_descs_p);
	return ret;
}

