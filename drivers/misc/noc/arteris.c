// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright (c) 2025 Sima ai
 *
 * Author: Abhimanyu G <abhimanyu.g@sima.ai>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/cdev.h>

#include <linux/simaai/nocif.h>
#include "arteris.h"


struct noc_device {
  dev_t dev_num;
  struct device *dev;
  struct cdev cdev;
  struct class *dev_class;
};

struct nocDesc {
  /*
   *TODO: Add support for rest of noc (pkt probe, QoS, firewall...)
   */
  struct transactionProbe txnProbes[MAX_TRANSACTION_PROBES];
};

// Runtime context structure
static struct nocDesc nocCtx;

static int configureAndEnableTxnProbe(struct txnProbeDesc *desc)
{
	uint32_t regVal = 0;
	struct transactionProbe *txnProbe;
	struct transactionProfiler *txnProfiler;

	if (!desc)
		return -EINVAL;


	txnProbe = &nocCtx.txnProbes[desc->id];
	txnProfiler = &nocCtx.txnProbes[desc->id].profiler;

	strncpy(desc->label, txnProbe->label,
		sizeof(desc->label) / sizeof(desc->label[0]));

	/* Setup */
	//StatEn-Disable
	regVal = ioread32(txnProbe->reg + TXN_PROBE_MAIN_CTL_OFFSET) &
		 ~TXN_PROBE_MAIN_CTL_STATEN;
	iowrite32(regVal, txnProbe->reg + TXN_PROBE_MAIN_CTL_OFFSET);

	//GlobalEn-Disable
	iowrite32(~TXN_PROBE_CFG_CTL_GLOBAL_EN,
		  txnProbe->reg + TXN_PROBE_CFG_CTL_OFFSET);

	//En profiler-Disable
	iowrite32(~TXN_PROFILER_EN, txnProfiler->reg + TXN_PROFILER_EN_OFFSET);

	// Profiler configuration
	iowrite32(((uint32_t)desc->profilerMode) & TXN_PROFILER_MODE_MASK,
		  (txnProfiler->reg + TXN_PROFILER_MODE_OFFSET));

	iowrite32(((uint32_t)desc->nTenureLines) &
			  TXN_PROFILER_TENURE_LINES_MASK,
		  (txnProfiler->reg + TXN_PROFILER_TENURE_LINES_OFFSET));
	iowrite32(TXN_PROFILER_OVF_RESET_STATUS,
		  (txnProfiler->reg + TXN_PROFILER_OVF_RESET_OFFSET));


	// Update counter source to external
	for (int j = 0; j < txnProbe->nCounters; j++) {
		iowrite32(TXN_PROBE_CNT_SRC_EXT | j,
			  txnProbe->reg + TXN_PROBE_CNT_SRC_OFFSET(j));
	}

	//Observed/Filter configuration
	for (int i = 0; i < txnProbe->profiler.nObsSel; i++) {
		struct observedDesc *obs = &desc->observedSel[i];
		struct transactionFilter *filter =
			&txnProbe->fltDesc[desc->observedSel[i].fltId];

		strncpy(obs->label, filter->label,
			sizeof(obs->label) / sizeof(obs->label[0]));

		//Mode filter
		iowrite32(((uint32_t)obs->mode) & TXN_FILTER_MODE_MASK,
			  filter->reg + TXN_FILTER_MODE_OFFSET);
		//Opcode filter
		iowrite32(((uint32_t)obs->opcode) & TXN_FILTER_OPCODE_MASK,
			  filter->reg + TXN_FILTER_OPCODE_OFFSET);

		//address filter
		iowrite32(obs->addrBase.addrBase_low,
			  filter->reg + TXN_FILTER_ADDRBASE_LOW_OFFSET);
		iowrite32(obs->addrBase.addrBase_high,
			  filter->reg + TXN_FILTER_ADDRBASE_HIGH_OFFSET);
		iowrite32((uint32_t)obs->addrWindowSize &
			  TXN_FILTER_ADDRWINDOW_SIZE_MASK,
			  filter->reg + TXN_FILTER_ADDRWINDOW_SIZE_OFFSET);

		//Select observers
		iowrite32((uint32_t)obs->fltId &
			  TXN_PROFILER_OBS_SEL_MASK(txnProbe->nFilters),
			  txnProfiler->reg + TXN_PROFILER_OBS_SEL_OFFSET(i));

		// Write Thresholds_i_j for the current stat profiler
		for (int j = 0; j < MAX_THRESHOLD_COUNT_PER_CH; j++) {
		  iowrite32(obs->threshold[j] &
			    TXN_PROFILER_THRESHOLD_MASK,
			    txnProfiler->reg +
			    TXN_PROFILER_THRESHOLD_OFFSET(i) +
			    (j * 4));
		}
	}

	//StatEn
	regVal = ioread32(txnProbe->reg + TXN_PROBE_MAIN_CTL_OFFSET) |
		 TXN_PROBE_MAIN_CTL_STATEN;
	iowrite32(regVal, txnProbe->reg + TXN_PROBE_MAIN_CTL_OFFSET);

	//GlobalEn
	iowrite32(TXN_PROBE_CFG_CTL_GLOBAL_EN,
		  txnProbe->reg + TXN_PROBE_CFG_CTL_OFFSET);

	//En profiler
	iowrite32(TXN_PROFILER_EN, txnProfiler->reg + TXN_PROFILER_EN_OFFSET);

	return 0;
}

static int readHistogramBins(struct txnProbeDesc *probe)
{
	struct transactionProbe *txnProbe;
	struct transactionProfiler *txnProfiler;
	uint8_t cntValOffset = 0, cnt = 0;

	if (!probe)
	  return -EINVAL;

	txnProbe = &nocCtx.txnProbes[probe->id];
	txnProfiler = &nocCtx.txnProbes[probe->id].profiler;
	probe->ovfStatus = ioread32(txnProfiler->reg + TXN_PROFILER_OVF_STATUS_OFFSET);
	for (int i = 0; i < txnProbe->profiler.nObsSel; i++) {
	  struct observedDesc *obs = &probe->observedSel[i];
	  for(int j = 0; j < 5; j++){
	    obs->histogramBin[j] = ioread32(txnProbe->reg + TXN_PROBE_CNT_VAL_OFFSET(cntValOffset++));
	  }
	}
	return 0;
}

static long noc_platform_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
  int ret = 0;
  struct txnProbeDesc probeDesc;
  copy_from_user(&probeDesc, (struct txnProbeDesc *)arg,
		 sizeof(struct txnProbeDesc));

  switch (cmd) {
  case NOC_IOCTL_CONFIGURE_AND_RUN:
    ret = configureAndEnableTxnProbe(&probeDesc);
    copy_to_user((struct txnProbeDesc *)arg, &probeDesc, sizeof(struct txnProbeDesc));
    break;

  case NOC_IOCTL_READ:
    ret = readHistogramBins(&probeDesc);
    copy_to_user((struct txnProbeDesc *)arg, &probeDesc, sizeof(struct txnProbeDesc));
    break;

  default:
	  pr_err("Unsupported IOCTL :%d\n", cmd);
	  return -ENOTTY; // Command not supported
  }

  return ret;
}

static const struct file_operations noc_platform_fops = {
  .owner          = THIS_MODULE,
  .unlocked_ioctl = noc_platform_ioctl,  // IOCTL handler
};

static int parse_transaction_probes(struct device *dev,struct device_node *node)
{
	int i = 0;
	struct device_node *probe_node;

	if (!node) {
		dev_err(dev, "Transaction probe node not found\n");
		return -EINVAL;
	}

	for_each_child_of_node(node, probe_node) {
		uint32_t reg[2], probeId, fltId;
		struct device_node *profiler_node, *filter_node, *flts;

		//ID
		of_property_read_u32(probe_node, "probe-id", &probeId);

		// Name
		of_property_read_string(probe_node, "label",
					&nocCtx.txnProbes[probeId].label);
		of_property_read_u32(probe_node, "num-counters",
				     &nocCtx.txnProbes[probeId].nCounters);
		//Reg Description
		of_property_read_u32_array(probe_node, "reg", reg, 2);
		nocCtx.txnProbes[probeId].reg =
			devm_ioremap(dev, reg[0], reg[1]);


		/* Profiler*/
		profiler_node = of_get_child_by_name(probe_node, "profiler");
		// Reg Description
		of_property_read_u32_array(profiler_node, "reg", reg, 2);
		nocCtx.txnProbes[probeId].profiler.reg =
			devm_ioremap(dev, reg[0], reg[1]);
		dev_info(dev,
			 "probe:%d, addr:0x%X, size:0x%X, mapped at:0x%llx\n",
			 probeId, reg[0], reg[1],
			 nocCtx.txnProbes[probeId].profiler.reg);
		of_property_read_u32(
			profiler_node, "observed-sel-cnt",
			&nocCtx.txnProbes[probeId].profiler.nObsSel);

		/* Filters */
		filter_node = of_get_child_by_name(probe_node, "filters");
		of_property_read_u32(filter_node, "flt-cnt",
				     &nocCtx.txnProbes[probeId].nFilters);
		for_each_child_of_node(filter_node, flts) {

		  of_property_read_u32(flts, "flt-id", &fltId);
		  of_property_read_u32_array(flts, "reg", reg, 2);
			nocCtx.txnProbes[probeId].fltDesc[fltId].reg =
				devm_ioremap(dev, reg[0], reg[1]);

			of_property_read_string(
				flts, "label",
				&nocCtx.txnProbes[probeId].fltDesc[fltId].label);
			dev_info(
				dev,
				"Successfully added transaction filter \"%s\"\n",
				nocCtx.txnProbes[probeId].fltDesc[fltId].label);
		}
			  dev_info(dev, "Successfully added transaction probe \"%s\"\n",
			 nocCtx.txnProbes[probeId].label);
	}
	return 0;
}

static int noc_platform_probe(struct platform_device *pdev)
{
	int ret, i = 0;
	struct device *dev = &pdev->dev, *sysDev = NULL;
	struct noc_device *nocdev;
	struct device_node *probe_node, *txn_probes_node;

	dev_info(dev, "Probing platform device: %s\n", SIMAAI_NOC_DEV_NAME);

	nocdev = devm_kzalloc(dev, sizeof(struct noc_device), GFP_KERNEL);
	if (!nocdev)
		return -ENOMEM;

	nocdev->dev = dev;

	// Allocate a device number dynamically
	ret = alloc_chrdev_region(&nocdev->dev_num, 0, 1, SIMAAI_NOC_DEV_NAME);
	if (ret) {
		dev_err(dev, "Failed to allocate device number\n");
		return ret;
	}

	// Initialize cdev structure
	cdev_init(&nocdev->cdev, &noc_platform_fops);

	// Add cdev to the system
	ret = cdev_add(&nocdev->cdev, nocdev->dev_num, 1);
	if (ret) {
		dev_err(dev, "Failed to add cdev\n");
		unregister_chrdev_region(nocdev->dev_num, 1);
		return ret;
	}

	nocdev->dev_class = class_create(THIS_MODULE, SIMAAI_NOC_DEV_NAME);
	if (IS_ERR(nocdev->dev_class)) {
		dev_err(dev, "Failed: class_create\n");
		ret = PTR_ERR(nocdev->dev_class);
		/*TODO: Clean-up exit*/
		return ret;
	}

	sysDev = device_create(nocdev->dev_class, NULL, nocdev->dev_num, NULL,
			       SIMAAI_NOC_DEV_NAME);
	if (IS_ERR(sysDev)) {
		dev_err(dev, "Could not create a sysfs entry\n");
		ret = PTR_ERR(nocdev->dev_class);
		/*TODO: Clean-up exit*/
		return ret;
	}
	//Parse DT
	txn_probes_node =
		of_get_child_by_name(dev->of_node, "transaction-probes");

	ret = parse_transaction_probes(dev,txn_probes_node);

	platform_set_drvdata(pdev, nocdev);
	dev_info(dev, "Character device registered with major number %d\n",
		 MAJOR(nocdev->dev_num));

	return 0;
}

static int noc_platform_remove(struct platform_device *pdev)
{
	struct noc_device *nocdev = (struct noc_device *)platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	dev_info(dev, "Removing platform device: %s\n", SIMAAI_NOC_DEV_NAME);

	// Remove cdev and unregister device number
	device_destroy(nocdev->dev_class, nocdev->dev_num);
	class_destroy(nocdev->dev_class);
	cdev_del(&nocdev->cdev);
	unregister_chrdev_region(nocdev->dev_num, 1);

	return 0;
}

static const struct of_device_id noc_platform_of_match[] = {
	{ .compatible = "arteris,flex-noc" },
	{},
};
MODULE_DEVICE_TABLE(of, noc_platform_of_match);

static struct platform_driver noc_platform_driver = {
    .probe = noc_platform_probe,
    .remove = noc_platform_remove,
    .driver = {
        .name = "noc_platform_driver",
        .of_match_table = noc_platform_of_match,
    },
};

// Register platform driver using module_platform_driver macro
module_platform_driver(noc_platform_driver);

MODULE_AUTHOR("Abhimanyu G <abhimanyu.g@sima.ai>");
MODULE_DESCRIPTION("SiMa.ai Modalix NoC control driver");
MODULE_LICENSE("Dual MIT/GPL");
