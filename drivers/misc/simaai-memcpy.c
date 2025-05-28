// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2025 Sima ai
 *
 * Author: Gopal Devarapu <gopal.devarapu@sima.ai>
 */

#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <uapi/linux/simaai/simaai_memory_ioctl.h>

#define NUM_MEMCPY_SDMA_CHANNELS 8

static DEFINE_MUTEX(memcpy_lock);

struct simaai_sdma_txfer_comp {
	int ch_id;	
	struct completion sdma_m2m_done;
};
struct simaai_sdma_channel {
	bool busy;
	struct dma_chan *chan;
	struct dma_slave_config slave_config;
	struct completion sdma_m2m_done;
	struct simaai_sdma_txfer_comp cmp;
};

static struct simaai_sdma_channel memcpy_channels[NUM_MEMCPY_SDMA_CHANNELS] = {0};
static const struct of_device_id simaai_memcpy_id_table[] = {
	{ .compatible = "simaai,memcpy-manager" },
	{}
};

MODULE_DEVICE_TABLE(of, simaai_memcpy_id_table);

static struct platform_driver simaai_memcpy_driver;

static void sdma_m2m_callback(void *param)
{
	struct simaai_sdma_txfer_comp *op = (struct simaai_sdma_txfer_comp *)param;
	int id = op->ch_id;
	mutex_lock(&memcpy_lock);
	complete(&(memcpy_channels[id].cmp.sdma_m2m_done));
	mutex_unlock(&memcpy_lock);

	return ;
}

/* find a free DMA channel */
static int find_free_dma_channel(void) {
	int i = 0 ;

	mutex_lock(&memcpy_lock);
	while(memcpy_channels[i].chan) {
		if (!memcpy_channels[i].busy) {
			memcpy_channels[i].busy = true;
		mutex_unlock(&memcpy_lock);
			return i;
		}
	i++;
	}
	mutex_unlock(&memcpy_lock);
	return -1;
}

int simaai_sdma_memcpy(struct simaai_memcpy_args *cp_args)
{
	struct dma_async_tx_descriptor *sdma_m2m_desc;
	dma_cookie_t cookie;
	struct simaai_sdma_channel *dma_channel;
	int channel;

	/* Find a free DMA channel */
	while ((channel = find_free_dma_channel()) == -1) {
		/* Block until a DMA channel is free */
		usleep_range(50,100);
	}
	
	mutex_lock(&memcpy_lock);
	dma_channel = &memcpy_channels[channel];
	mutex_unlock(&memcpy_lock);

	init_completion(&(dma_channel->cmp.sdma_m2m_done));
	sdma_m2m_desc = dma_channel->chan->device->device_prep_dma_memcpy(dma_channel->chan, cp_args->dst_addr, cp_args->src_addr, cp_args->size, 0);
	if (sdma_m2m_desc) {
		sdma_m2m_desc->callback = sdma_m2m_callback;
		dma_channel->cmp.ch_id = channel;
		sdma_m2m_desc->callback_param = &(dma_channel->cmp);
		cookie = dmaengine_submit(sdma_m2m_desc);
		dma_async_issue_pending(dma_channel->chan);
		wait_for_completion(&(dma_channel->cmp.sdma_m2m_done));
		dma_channel->busy = false;
	} else {
		return -EFAULT;
	}

	return 0;
}
EXPORT_SYMBOL(simaai_sdma_memcpy);

static int simaai_memcpy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	int ret;
	int index = 0;
	const char *dma_name;

	/* Read each DMA name */
	while ((of_property_read_string_index(np, "dma-names", index, &dma_name) == 0)
			&& (index < NUM_MEMCPY_SDMA_CHANNELS)) {
		memcpy_channels[index].chan = dma_request_slave_channel(&pdev->dev, dma_name);
		if (memcpy_channels[index].chan != NULL) {
			init_completion(&(memcpy_channels[index].cmp.sdma_m2m_done));
			memcpy_channels[index].busy = false;
			memcpy_channels[index].slave_config.direction = DMA_MEM_TO_MEM;
			memcpy_channels[index].slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
			memcpy_channels[index].slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
			ret = dmaengine_slave_config(memcpy_channels[index].chan, &(memcpy_channels[index].slave_config));
			if (ret) {
				dev_err(&pdev->dev, "error in slave config for DMA Channel %d: %s\n", index, dma_name);
				return ret;
			}
		} else {
			dev_err(dev, "Failed to request DMA channel  %d: %s\n", index, dma_name);
			return -ENODEV;
		}
		index++;
	}
	dev_info(&pdev->dev, "Registed %d DMA Channels\n", index);

	return 0;
}

static int simaai_memcpy_remove(struct platform_device *pdev)
{

	for(int i=0; i<NUM_MEMCPY_SDMA_CHANNELS; i++) { 
		if(memcpy_channels[i].chan)
			dma_release_channel(memcpy_channels[i].chan);
	}

	return 0;
}

static struct platform_driver simaai_memcpy_driver = {
	.driver = {
		.name = "simaai,memcpy-manager",
		.of_match_table = of_match_ptr(simaai_memcpy_id_table),
	},
	.probe = simaai_memcpy_probe,
	.remove = simaai_memcpy_remove,

};
module_platform_driver(simaai_memcpy_driver);

MODULE_AUTHOR("Gopal Devarapu <gopal.devarapu@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai memcpy driver");
MODULE_LICENSE("Dual MIT/GPL");
