  /*
  SPDX-License-Identifier: (GPL-2.0+ OR MIT)
  *
  * Copyright (c) 2025 Sima ai
  *
  * Author: Bhimesh  <bhimeswararao.matsa@sima.ai>
  */


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/dmaengine.h>
#include <linux/device.h>
#include <linux/of_reserved_mem.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/libnvdimm.h>

#define SIMAAI_DMATEST_DEVICE_NAME      "simaai_dmatest"

//#define DEBUG
//#define KMALLOC_ALLOCATION

#define SDMA_BUF_SIZE  (6 * 1024 * 1024)
#define PATTERN_A	0xDEEDBEEF
#define PATTERN_B	0x57891234
#define PATTERN_C	0x45AB7712
#define MODALIX_OCM_OFFSET	0x200000

struct completion dma_m2m_ok;
struct simaai_dmatest_device {
	struct platform_device *pdev;
	struct dma_chan *chan;
};

static void dma_m2m_callback(void *data)
{
    complete(&dma_m2m_ok);
    return ;
}

static int buffer_verify(u64 *wdata, u64 *rdata)
{
    u64 i;

    for (i=0; i<SDMA_BUF_SIZE/8; i++) {
        if (*(rdata+i) != *(wdata+i)) {
            pr_info("buffer check failed at index:%lld, rbuf:0x%llx, wbuf:0x%llx \n", i, *(rdata +i), *(wdata+i));
        }
    }
   return 0;
}

static int io_buffer_verify(u64 *mem_data, void  __iomem *io_buffer)
{
	u64 i,tmp;
	int error=0;
	void __iomem *io_data = io_buffer;

	for (i=0; i<SDMA_BUF_SIZE/8; i++) {
		tmp = ioread64(io_data + (i*8));
		if (tmp != *(mem_data+i)) {
			pr_info("buffer check failed at index:%lld, iobuf:0x%llx, ddrbuf:0x%llx \n", i, tmp, *(mem_data+i));
			error++;
			if(error >20)  {	
			pr_info("more errors, stopped buffer verification \n");
				return -1;
			}
		}
	}
	return 0;
}

static int ddr_to_ocm_test(struct simaai_dmatest_device *sdma_dev)
{
	struct platform_device *pdev = sdma_dev->pdev;
	u64 *wbuf;
	void __iomem *rbuf;
	u64 *index, i;
	struct dma_async_tx_descriptor *dma_m2m_desc;
	dma_addr_t dma_src;
	dma_cookie_t cookie;
	u32 ret;
	u64 end, start;
	size_t aligned_size;

#ifndef KMALLOC_ALLOCATION
	struct page *pg_ptr;
#endif
#ifdef KMALLOC_ALLOCATION
	pr_info("ddr_to_ocm_test for small buffer size\n");
	wbuf = devm_kzalloc(&pdev->dev, SDMA_BUF_SIZE, GFP_KERNEL);
	if(!wbuf) {
		pr_info("error wbuf !!!!!!!!!!!\n");
		return -1;
	}
	dma_src = dma_map_single(sdma_dev->chan->device->dev, wbuf, SDMA_BUF_SIZE, DMA_TO_DEVICE);
#else
	pr_info("ddr_to_ocm_test for large buffer size\n");
       	aligned_size = PAGE_ALIGN(SDMA_BUF_SIZE);
        wbuf = dma_alloc_coherent(sdma_dev->chan->device->dev, aligned_size, &dma_src, GFP_USER);
        if (!wbuf) {
                dev_err(sdma_dev->chan->device->dev, "dma_alloc_coherent alloc of %zu bytes failed\n", aligned_size);
                return -1;
        }

#endif
	/* at ocm offset:2MB */
	rbuf = devm_ioremap(&pdev->dev, MODALIX_OCM_OFFSET, SDMA_BUF_SIZE);
	index = wbuf;
	for (i=0; i<SDMA_BUF_SIZE/8; i++) {
		*index = PATTERN_C;
		index++;
	}
	arch_wb_cache_pmem((void *)wbuf, SDMA_BUF_SIZE);
	dma_m2m_desc = sdma_dev->chan->device->device_prep_dma_memcpy(sdma_dev->chan, MODALIX_OCM_OFFSET, dma_src, SDMA_BUF_SIZE,0);
	if (dma_m2m_desc) {
		dma_m2m_desc->callback = dma_m2m_callback;
	} else {
#ifdef KMALLOC_ALLOCATION
		dma_unmap_single(sdma_dev->chan->device->dev, dma_src, SDMA_BUF_SIZE, DMA_TO_DEVICE);
#else
		dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, wbuf, dma_src);
#endif
		return -1;
	}

	cookie = dmaengine_submit(dma_m2m_desc);

	start = ktime_get_ns();
	dma_async_issue_pending(sdma_dev->chan);

	wait_for_completion(&dma_m2m_ok);
	
	end = ktime_get_ns();
	pr_info("time diff = %lld ns\n", end - start);

#ifdef KMALLOC_ALLOCATION
	dma_unmap_single(sdma_dev->chan->device->dev, dma_src, SDMA_BUF_SIZE, DMA_TO_DEVICE);
#endif

#ifdef DEBUG	
	index = rbuf;
	for (i=0; i<SDMA_BUF_SIZE/8; i++) {
		pr_info("dest buf:0x%llx \n", *(index+i));
	}
#endif	
	arch_invalidate_pmem((void *) rbuf, SDMA_BUF_SIZE);
	pr_info("buffer data verification...\n");
	ret = io_buffer_verify(wbuf, rbuf);
	if (!ret)
		pr_info("buffer copy passed...\n");

#ifndef KMALLOC_ALLOCATION
	dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, wbuf, dma_src);
#endif
	return 0;
}

static int ocm_to_ddr_test(struct simaai_dmatest_device *sdma_dev)
{
	struct platform_device *pdev = sdma_dev->pdev;
	u64 *rbuf;
	void __iomem *wbuf;
	u64 *index, i;
	struct dma_async_tx_descriptor *dma_m2m_desc;
	dma_addr_t dma_dst;
	dma_cookie_t cookie;
	u32 ret;
	u64 end, start;
	size_t aligned_size;

#ifndef KMALLOC_ALLOCATION
	struct page *pg_ptr;
#endif

#ifdef KMALLOC_ALLOCATION
	pr_info("ocm_to_ddr_test for small buffer size\n");
	rbuf = devm_kzalloc(&pdev->dev, SDMA_BUF_SIZE, GFP_KERNEL);
	if(!rbuf) {
		pr_info("error rbuf !!!!!!!!!!!\n");
		return -1;
	}
	dma_dst = dma_map_single(sdma_dev->chan->device->dev, rbuf, SDMA_BUF_SIZE, DMA_FROM_DEVICE);
#else
	pr_info("ocm_to_ddr_test for large buffer size\n");
       	aligned_size = PAGE_ALIGN(SDMA_BUF_SIZE);
        rbuf = dma_alloc_coherent(sdma_dev->chan->device->dev,aligned_size, &dma_dst, GFP_USER);
        if (!rbuf) {
                dev_err(sdma_dev->chan->device->dev, "dma_alloc_coherent alloc of %zu bytes failed\n", aligned_size);
                return -1;
        }
#endif
	/* at ocm offset:2MB */
	wbuf = devm_ioremap(&pdev->dev, MODALIX_OCM_OFFSET, SDMA_BUF_SIZE);

	for (i=0; i<SDMA_BUF_SIZE/8; i++) {
		iowrite64(PATTERN_B, wbuf + i*8);
	}

	arch_wb_cache_pmem((void *)wbuf, SDMA_BUF_SIZE);
	dma_m2m_desc = sdma_dev->chan->device->device_prep_dma_memcpy(sdma_dev->chan, dma_dst, MODALIX_OCM_OFFSET, SDMA_BUF_SIZE,0);
	if (dma_m2m_desc) {
		dma_m2m_desc->callback = dma_m2m_callback;
	} else {
#ifdef KMALLOC_ALLOCATION
		dma_unmap_single(sdma_dev->chan->device->dev, dma_dst, SDMA_BUF_SIZE, DMA_TO_DEVICE);
#else
 		dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, rbuf, dma_dst);
#endif
		return -1;
	}

	cookie = dmaengine_submit(dma_m2m_desc);

	start = ktime_get_ns();
	dma_async_issue_pending(sdma_dev->chan);

	wait_for_completion(&dma_m2m_ok);
	
	end = ktime_get_ns();
	pr_info("time diff = %lld ns\n", end - start);

#ifdef KMALLOC_ALLOCATION
	dma_unmap_single(sdma_dev->chan->device->dev, dma_dst, SDMA_BUF_SIZE, DMA_FROM_DEVICE);
#endif

#ifdef DEBUG
        index = rbuf;
        for (i=0; i<SDMA_BUF_SIZE/8; i++) {
                pr_info("dest buf:0x%llx \n", *(index+i));
        }
#endif
	arch_invalidate_pmem((void *) rbuf, SDMA_BUF_SIZE);
	pr_info("buffer data verification...\n");
	ret = io_buffer_verify(rbuf, wbuf);
	if (!ret)
		pr_info("buffer copy passed...\n");

#ifndef KMALLOC_ALLOCATION
 	dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, rbuf, dma_dst);
#endif
	return 0;
}

static int ddr_to_ddr_test(struct simaai_dmatest_device *sdma_dev)
{
	struct platform_device *pdev = sdma_dev->pdev;
	u64 *wbuf;
	u64 *rbuf;
	u64 *index, i;
	struct dma_async_tx_descriptor *dma_m2m_desc;
	dma_addr_t dma_src, dma_dst ;
	dma_cookie_t cookie;
	u32 ret;
	u64 end, start;
	size_t aligned_size;

#ifndef KMALLOC_ALLOCATION
        struct page *rd_pg_ptr;
        struct page *wr_pg_ptr;
#endif

#ifdef KMALLOC_ALLOCATION
	pr_info("ddr_to_ddr_test for small buffer size\n");
	wbuf = devm_kzalloc(&pdev->dev, SDMA_BUF_SIZE, GFP_KERNEL);
	if(!wbuf) {
		pr_info("error wbuf !!!!!!!!!!!\n");
		return -1;
	}

	rbuf = devm_kzalloc(&pdev->dev, SDMA_BUF_SIZE, GFP_KERNEL);
	if(!rbuf) {
		pr_info("error rbuf !!!!!!!!!!!\n");
		return -1;
	}


	dma_src = dma_map_single(sdma_dev->chan->device->dev, wbuf, SDMA_BUF_SIZE, DMA_TO_DEVICE);
	/* Check if the mapping failed */
	if (dma_mapping_error(sdma_dev->chan->device->dev, dma_src)) {
	    dev_err(sdma_dev->chan->device->dev, "DMA mapping failed for source buffer\n");
	    return -ENOMEM;  /* Or another error code based on your error handling policy */
	}

	pr_info("wbuf dma map done\n");
	dma_dst = dma_map_single(sdma_dev->chan->device->dev, rbuf, SDMA_BUF_SIZE, DMA_FROM_DEVICE);
	/* Check if the mapping failed */
	if (dma_mapping_error(sdma_dev->chan->device->dev, dma_dst)) {
	    dev_err(sdma_dev->chan->device->dev, "DMA mapping failed for destination buffer\n");
	    return -ENOMEM;  /* Or another error code based on your error handling policy */
	}
	
#else
	pr_info("ddr_to_ddr_test for large buffer size\n");
	aligned_size = PAGE_ALIGN(SDMA_BUF_SIZE);
        wbuf = dma_alloc_coherent(sdma_dev->chan->device->dev, aligned_size, &dma_src, GFP_USER);
        if (!wbuf) {
                dev_err(sdma_dev->chan->device->dev, "dma_alloc_coherent alloc of %zu bytes failed\n", aligned_size);
                return -1;
        }
        rbuf = dma_alloc_coherent(sdma_dev->chan->device->dev,aligned_size, &dma_dst, GFP_USER);
        if (!rbuf) {
                dev_err(sdma_dev->chan->device->dev, "dma_alloc_coherent alloc of %zu bytes failed\n", aligned_size);
                return -1;
        }
#endif
	index = wbuf;
	for (i=0; i<SDMA_BUF_SIZE/8; i++) {
		*index = PATTERN_A;
		index++;
	}

	arch_wb_cache_pmem((void *)wbuf, SDMA_BUF_SIZE);
	dma_m2m_desc = sdma_dev->chan->device->device_prep_dma_memcpy(sdma_dev->chan, dma_dst, dma_src, SDMA_BUF_SIZE,0);
	if (dma_m2m_desc) {
		dma_m2m_desc->callback = dma_m2m_callback;
	} else {
#ifdef KMALLOC_ALLOCATION
	dma_unmap_single(sdma_dev->chan->device->dev, dma_src, SDMA_BUF_SIZE, DMA_TO_DEVICE);
	dma_unmap_single(sdma_dev->chan->device->dev, dma_dst, SDMA_BUF_SIZE, DMA_FROM_DEVICE);
#else
	dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, wbuf, dma_src);
 	dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, rbuf, dma_dst);

#endif
		return -1;
	}
	cookie = dmaengine_submit(dma_m2m_desc);
	smp_wmb();
	start = ktime_get_ns();
	dma_async_issue_pending(sdma_dev->chan);
	wait_for_completion(&dma_m2m_ok);
	end = ktime_get_ns();
	pr_info("time diff = %lld ns\n", end - start);


#ifdef KMALLOC_ALLOCATION
	if (dma_mapping_error(sdma_dev->chan->device->dev, dma_src)) {
                        pr_err("DMA unmap failed for source buffer\n");
			/* Handle the error appropriately, such as returning an error code or cleaning up */
                        return -EIO;  /* Return error code, modify as needed */
	}

	dma_unmap_single(sdma_dev->chan->device->dev, dma_src, SDMA_BUF_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(sdma_dev->chan->device->dev, dma_dst)) {
        	        pr_err("DMA unmap failed for destination buffer\n");
                        /* Handle the error appropriately, such as returning an error code or cleaning up */
                        return -EIO;  /* Return error code, modify as needed */
        }

	dma_unmap_single(sdma_dev->chan->device->dev, dma_dst, SDMA_BUF_SIZE, DMA_FROM_DEVICE);
#endif

#ifdef DEBUG
        index = rbuf;
        for (i=0; i<SDMA_BUF_SIZE/8; i++) {
                pr_info("dest buf:0x%llx \n", *(index+i));
        }
#endif
	arch_invalidate_pmem((void *) rbuf, SDMA_BUF_SIZE);
	pr_info("buffer data verification...\n");
	ret = buffer_verify(wbuf, rbuf);
	if (!ret)
		pr_info("buffer copy passed...\n");

#ifndef KMALLOC_ALLOCATION
	dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, wbuf, dma_src);
 	dma_free_coherent(sdma_dev->chan->device->dev, aligned_size, rbuf, dma_dst);
#endif
	return 0;
}


static int simaai_dmatest_sdma_init(struct simaai_dmatest_device *sdma_dev)
{
	struct dma_slave_config slave_config = {};
	struct platform_device *pdev = sdma_dev->pdev;
	int ret = 0;

	init_completion(&dma_m2m_ok);
	slave_config.direction = DMA_MEM_TO_MEM;
	slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	dev_info(&pdev->dev, "slave config \n");
	ret = dmaengine_slave_config(sdma_dev->chan, &slave_config);
	if (ret) {
		dev_err(&pdev->dev, "error in sdma slave config \n");
		goto end;
	}
	dev_info(&pdev->dev, "SDMA slave config done\n");
	return 0;

end:
	dev_err(&pdev->dev, "sima DMATEST sdma_init failed \n");
	return ret;
}

static const struct of_device_id simaai_dmatest_dev_of_match[];

static int simaai_dmatest_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev; 
	const struct of_device_id *of_id;
	int ret = 0;
	struct simaai_dmatest_device *simaai_dmatest_dev;

	dev_info(dev, "Probe: sima SDMA Test driver  \n");

	if (!dev->of_node)
		return -ENODEV;

	simaai_dmatest_dev = devm_kzalloc(dev, sizeof(*simaai_dmatest_dev), GFP_KERNEL);
	if (!simaai_dmatest_dev)
		return -ENOMEM;

	of_id = of_match_node(simaai_dmatest_dev_of_match, dev->of_node);
	if (WARN_ON(of_id == NULL))
		return -EINVAL;

	dev_info(dev, "Requesting DMA channel\n");
	simaai_dmatest_dev->chan = dma_request_slave_channel(&pdev->dev, "sdma");
	if (simaai_dmatest_dev->chan == NULL) {
		dev_err(dev, "no DMA channel found by name sdma\n");
		ret = -ENODEV;
		goto end;
	}

	dev_info(&pdev->dev, "dma chan id %d\n", simaai_dmatest_dev->chan->chan_id);
	simaai_dmatest_dev->pdev = pdev;
	platform_set_drvdata(pdev, simaai_dmatest_dev);

	ret = simaai_dmatest_sdma_init(simaai_dmatest_dev);
	dev_info(dev,"DDR to DDR memcpy start \n");
	ret = ddr_to_ddr_test(simaai_dmatest_dev);
	dev_info(dev,"DDR to DDR memcpy end \n");
	dev_info(dev,"DDR to OCM memcpy start \n");
	ret = ddr_to_ocm_test(simaai_dmatest_dev);
	dev_info(dev,"DDR to OCM memcpy end \n");
	dev_info(dev,"OCM to DDR memcpy start \n");
	ret = ocm_to_ddr_test(simaai_dmatest_dev);
	dev_info(dev,"OCM to DDR memcpy end \n");
	return 0;
end:
	dev_err(&pdev->dev, "sima DMATEST Device not registered!!\n");
	return ret;
}

static int simaai_dmatest_dev_remove(struct platform_device *pdev)
{
       struct simaai_dmatest_device *dev = platform_get_drvdata(pdev);

	dma_release_channel(dev->chan);
        dev_info(&pdev->dev, "Driver removed\n");

       return 0;
}

static const struct of_device_id simaai_dmatest_dev_of_match[] = {
       {.compatible = "simaai,dmatest"},
       {}
};

MODULE_DEVICE_TABLE(of, simaai_dmatest_dev_of_match);

static struct platform_driver __refdata simaai_dmatest_dev_pdrv = {
       .remove = simaai_dmatest_dev_remove,
       .probe = simaai_dmatest_dev_probe,
       .driver = {
                  .name = SIMAAI_DMATEST_DEVICE_NAME,
                  .owner = THIS_MODULE,
                  .of_match_table = simaai_dmatest_dev_of_match,
                  },
};

module_platform_driver(simaai_dmatest_dev_pdrv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Bhimesh <bhimeswararao.matsa@sima.ai>");
MODULE_DESCRIPTION("Simaai SDMA Test driver");

