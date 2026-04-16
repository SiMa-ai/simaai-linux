// SPDX-License-Identifier: GPL-2.0
/*
 * Platform PCIe EP driver for SiMa.ai Davinci SoCs
 *
 * Copyright (C) 2021-2026 SiMa.ai
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/version.h>
#include <linux/syslog.h>
#include <linux/poll.h>
#include <linux/reboot.h>
#include <linux/timekeeping.h>
#include <linux/circ_buf.h>

#include <linux/simaai_pcie_api.h>
#include "simaai_pcie_drv.h"

#include "simaai_pep_hw.h"
#include "simaai_pep_drv.h"
#include "simaai_pep_drv_modalix.h"
#include "simaai_pep_drv_davinci.h"

#define DRV_MODULE_NAME "simaai_pep_drv"

static bool use_driver_loopback = false;
static struct si_pep_thread_param t_params[SI_MAX_DQS];
static long int breakfreq = 0;
static long int breakcntr = 0;
static bool dump_queue_occupency = false;

module_param(breakfreq, long, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(breakfreq,
		 "Frequency with which you want to skip dreq commands");
module_param(use_driver_loopback, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(use_driver_loopback, "Setup a data loopback within driver");
module_param(dump_queue_occupency, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dump_queue_occupency, "Dump queue occupency of all queues");
int dbg_level = 1;
module_param(dbg_level, int, (S_IRUGO | S_IWUSR));
MODULE_PARM_DESC(dbg_level,
		"Set driver debug level. 0b111111 to enable all\n"
		"bit 0 - Information messages\n"
		"bit 1 - Management path debug logs\n"
		"bit 2 - Data path debug logs\n"
		"bit 3 - Common debug logs\n"
		"bit 4 - Low priority debug logs\n"
		"bit 5 - Net interface debug logs\n"
		"bit 6 - Dump of data types\n"
		"bit 7 - DMA transaction debug logs\n"
		"Default value is 1. Errors are always shown");

struct si_pep_dev *g_pep_dev;
static const struct of_device_id si_pep_of_match[];

static void si_pep_alert_host(struct si_pep_dev *, struct si_alert *);
static int si_pep_send_resp(struct si_pep_qdev *, struct si_io *,
		struct si_qe_id *);
static int si_pep_data_loopback_handler(void *);

static ssize_t sysfs_show_timestamp(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf);
struct kobj_attribute sys_timestamp =
	__ATTR(show_timestamps, 0440, sysfs_show_timestamp, NULL);

static ssize_t sysfs_show_data_stats(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf);
struct kobj_attribute sys_data_stats =
	__ATTR(data_stats, 0440, sysfs_show_data_stats, NULL);
static ssize_t sysfs_show_mgmt_stats(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf);
struct kobj_attribute sys_mgmt_stats =
	__ATTR(mgmt_stats, 0440, sysfs_show_mgmt_stats, NULL);
static ssize_t sysfs_show_queue_occupency(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf);
struct kobj_attribute sys_queue_occup =
	__ATTR(queue_occup, 0440, sysfs_show_queue_occupency, NULL);

enum si_pep_timestamp_action {
	RECORD_ENTRY_TS,
	RECORD_EXIT_TS,
	COMPUTE_TS_METRICS
};

inline bool si_pep_is_data_queue(u32 qtype)
{
	return (qtype >= SI_DWQ && qtype <= SI_DRQ);
}

inline bool si_pep_is_mgmt_queue(u32 qtype)
{
	return (qtype >= SI_MWQ && qtype <= SI_MRQ);
}

char *si_pep_qtype_str(enum si_qtype t)
{
	switch (t) {
	case SI_ALL:
		return "All";
	case SI_MWQ:
		return "Management work";
	case SI_MCQ:
		return "Management completion";
	case SI_MRQ:
		return "Management receive";
	case SI_DWQ:
		return "Data work";
	case SI_DCQ:
		return "Data completion";
	case SI_DRQ:
		return "Data receive";
	default:
		return "Unknown";
	}
}

char *si_pep_qstatus_string(enum si_qstatus s)
{
	switch (s) {
	case SI_INVALID:
		return "Queue is not created yet or already destroyed";
	case SI_ACTIVE:
		return "Active";
	case SI_FROZEN:
		return "Frozen";
	case SI_FLUSHING:
		return "Flushing";
	default:
		return "Invalid option";
	}
}

static ssize_t sysfs_show_timestamp(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int pos = 0;
	int ts_index, qid;
	struct si_pep_dev *pep = g_pep_dev;
	struct si_pep_qdev *qdev;

	for (qid = 0; qid < SI_MAX_DQS; qid++) {
		qdev = &pep->qdev[qid];
		pos += sprintf(buf + pos,
			"Timestamps For Caching Requests for Queue %d\n", qid);
		pos += sprintf(buf + pos,
			"Entry timestamp | Exit timestamp\n");

		for (ts_index = 0; ts_index < MAX_TIMESTAMPS; ts_index++)
			pos += sprintf(buf + pos, "%lld | %lld\n",
					qdev->cache_dreq_ts.start[ts_index],
					qdev->cache_dreq_ts.stop[ts_index]);
		pos += sprintf(buf + pos, "Minimum timestamp %lld\n",
				qdev->cache_dreq_ts.min_time);
		pos += sprintf(buf + pos, "Maximum timestamp %lld\n",
				qdev->cache_dreq_ts.max_time);
	}

	for (qid = 0; qid < SI_MAX_DQS; qid++) {
		qdev = &pep->qdev[qid];
		pos += sprintf(buf + pos,
			"\nTimestamps Of Userspace reads for queue %d\n", qid);
		pos += sprintf(buf + pos,
				"Entry timestamp | Exit timestamp\n");
		for (ts_index = 0; ts_index < MAX_TIMESTAMPS; ts_index++)
			pos += sprintf(buf + pos, "%lld | %lld\n",
					qdev->read_ts.start[ts_index],
					qdev->read_ts.stop[ts_index]);
		pos += sprintf(buf + pos, "Minimum timestamp %lld\n",
				qdev->read_ts.min_time);
		pos += sprintf(buf + pos, "Maximum timestamp %lld\n",
				qdev->read_ts.max_time);
	}

	for (qid = 0; qid < SI_MAX_DQS; qid++) {
		qdev = &pep->qdev[qid];
		pos += sprintf(buf + pos,
			"\nTimestamps For userspace writes to queue %d\n", qid);
		pos += sprintf(buf + pos,
				"Entry timestamp | Exit timestamp\n");

		for (ts_index = 0; ts_index < MAX_TIMESTAMPS; ts_index++)
			pos += sprintf(buf + pos, "%lld | %lld\n",
					qdev->write_ts.start[ts_index],
					qdev->write_ts.stop[ts_index]);
		pos += sprintf(buf + pos, "Minimum timestamp %lld\n",
				qdev->write_ts.min_time);
		pos += sprintf(buf + pos, "Maximum timestamp %lld\n",
				qdev->write_ts.max_time);
	}

	return pos;
}

static ssize_t sysfs_show_mgmt_stats(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int pos = 0;
	struct si_pep_qdev *qdev;
	struct si_pep_dev *pep = g_pep_dev;

	qdev = &pep->qdev[0];
	pos += sprintf(buf + pos,
			"PCIe Statistics For Management Queue\n");
	pos += sprintf(buf + pos,
			"Queue status                           : ");
	pos += sprintf(buf + pos, "%s\n",
			si_pep_qstatus_string(pep->mwq_ctx->status));
	pos += sprintf(buf + pos,
			"No. of bytes transfered                : ");
	pos += sprintf(buf + pos,"%lld\n", qdev->stats.tx_bytes);
	pos += sprintf(buf + pos,
			"No. of bytes received                  : ");
	pos += sprintf(buf + pos, "%lld\n", qdev->stats.rx_bytes);
	pos += sprintf(buf + pos,
			"No. of requests received               : ");
	pos += sprintf(buf + pos, "%d\n", qdev->stats.rx_reqs);
	pos += sprintf(buf + pos,
			"No. of responses transfered            : ");
	pos += sprintf(buf + pos, "%d\n", qdev->stats.tx_resps);
	pos += sprintf(buf + pos,
			"No. of requests aborted successfully   : ");
	pos += sprintf(buf + pos, "%d\n",
			qdev->stats.abort_reqs_suc);
	pos += sprintf(buf + pos,
			"No. of requests aborted unsuccessfully : ");
	pos += sprintf(buf + pos,"%d\n",
			qdev->stats.abort_reqs_fail);
	pos += sprintf(buf + pos,
			"Timestamp of last data response sent   : ");
	pos += sprintf(buf + pos, "%lld ns\n", qdev->stats.last_resp);
	pos += sprintf(buf + pos, "\n");

	return pos;
}

static ssize_t sysfs_show_data_stats(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int pos = 0;
	int qid;
	struct si_pep_qdev *qdev;
	struct si_pep_dev *pep = g_pep_dev;
	enum si_qstatus st;

	for (qid = 1; qid <= SI_MAX_DQS; qid++) {
		qdev = &pep->qdev[qid];
		pos += sprintf(buf + pos,
			"PCIe Statistics For Data Queue %d\n", qid - 1);
		pos += sprintf(buf + pos,
				"Queue status                           : ");
		st = qdev->pep->dwq_ctx[qid-1]->status;
		pos += sprintf(buf + pos, "%s\n", si_pep_qstatus_string(st));
		pos += sprintf(buf + pos,
				"No. of bytes transfered                : ");
		pos += sprintf(buf + pos,"%lld\n", qdev->stats.tx_bytes);
		pos += sprintf(buf + pos,
				"No. of bytes received                  : ");
		pos += sprintf(buf + pos, "%lld\n", qdev->stats.rx_bytes);
		pos += sprintf(buf + pos,
				"No. of requests received               : ");
		pos += sprintf(buf + pos, "%d\n", qdev->stats.rx_reqs);
		pos += sprintf(buf + pos,
				"No. of responses transfered            : ");
		pos += sprintf(buf + pos, "%d\n", qdev->stats.tx_resps);
		pos += sprintf(buf + pos,
				"No. of requests aborted successfully   : ");
		pos += sprintf(buf + pos, "%d\n",
				qdev->stats.abort_reqs_suc);
		pos += sprintf(buf + pos,
				"No. of requests aborted unsuccessfully : ");
		pos += sprintf(buf + pos,"%d\n",
				qdev->stats.abort_reqs_fail);
		pos += sprintf(buf + pos,
				"Timestamp of last data response sent   : ");
		pos += sprintf(buf + pos, "%lld\n", qdev->stats.last_resp);
		pos += sprintf(buf + pos, "\n");
	}

	return pos;
}

#define MAX_TIMEPOINTS	512
static ssize_t sysfs_show_queue_occupency(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int pos = 0;
	int qid;
	int i;
	struct si_pep_qdev *qdev;
	struct si_qctx *wq_ctx;
	struct si_pep_dev *pep = g_pep_dev;
	ktime_t min_time;
	ktime_t max_time;
	ktime_t mean_time;
	ktime_t time_diff;
	ktime_t *timepoint_1;
	ktime_t *timepoint_2;
	ktime_t *timepoint_3;
	ktime_t *timepoint_4;
	int ntimeps;

	for(qid = 1; qid <= SI_MAX_DQS; qid++) {
		qdev = &pep->qdev[qid];
		timepoint_1 = qdev->timepoint_1;
		timepoint_2 = qdev->timepoint_2;
		timepoint_3 = qdev->timepoint_3;
		timepoint_4 = qdev->timepoint_4;
		wq_ctx = qdev->pep->dwq_ctx[qid - 1];

		pos += sprintf(buf + pos, "DATA QUEUE %d\n", qid - 1);
		pos += sprintf(buf + pos, "Queue status   : %s\n",
				si_pep_qstatus_string(wq_ctx->status));

		ntimeps = 0;
		min_time = 0;
		max_time = 0;
		mean_time = 0;
		for (i = 0; i < MAX_TIMEPOINTS; i++) {
			if (timepoint_2[i] == 0 || timepoint_1[i] == 0) {
				continue;
			}

			time_diff = ktime_sub(timepoint_2[i], timepoint_1[i]);
			if (i == 0) {
				mean_time = time_diff;
				min_time = time_diff;
				max_time = time_diff;
				ntimeps++;
				continue;
			}
			mean_time += time_diff;
			ntimeps++;
			if (time_diff < min_time)
				min_time = time_diff;
			if (time_diff > max_time)
				max_time = time_diff;
		}
		
		mean_time = (ntimeps <= 0) ? 0 : (mean_time / ntimeps);
		pos += sprintf(buf + pos, "Timepoint_1 <-> Timepoint_2\n");
		pos += sprintf(buf + pos, "Min time diff  : ");
		pos += sprintf(buf + pos, "%lld ns\n", min_time);
		pos += sprintf(buf + pos, "Max time diff  : ");
		pos += sprintf(buf + pos, "%lld ns\n", max_time);
		pos += sprintf(buf + pos, "Mean time diff : ");
		pos += sprintf(buf + pos, "%lld ns\n", mean_time);
		pos += sprintf(buf + pos, "\n");

		ntimeps = 0;
		min_time = 0;
		max_time = 0;
		mean_time = 0;
		for (i = 0; i < MAX_TIMEPOINTS; i++) {
			if (timepoint_2[i] == 0 || timepoint_3[i] == 0) {
				continue;
			}

			time_diff = ktime_sub(timepoint_3[i], timepoint_2[i]);
			if (i == 0) {
				mean_time = time_diff;
				min_time = time_diff;
				max_time = time_diff;
				ntimeps++;
				continue;
			}
			mean_time += time_diff;
			ntimeps++;
			if (time_diff < min_time)
				min_time = time_diff;
			if (time_diff > max_time)
				max_time = time_diff;
		}

		mean_time = (ntimeps <= 0) ? 0 : (mean_time / ntimeps);
		pos += sprintf(buf + pos, "Timepoint_2 <-> Timepoint_3\n");
		pos += sprintf(buf + pos, "Min time diff  : ");
		pos += sprintf(buf + pos, "%lld ns\n", min_time);
		pos += sprintf(buf + pos, "Max time diff  : ");
		pos += sprintf(buf + pos, "%lld ns\n", max_time);
		pos += sprintf(buf + pos, "Mean time diff : ");
		pos += sprintf(buf + pos, "%lld ns\n", mean_time);
		pos += sprintf(buf + pos, "\n");

		ntimeps = 0;
		min_time = 0;
		max_time = 0;
		mean_time = 0;
		for (i = 0; i < MAX_TIMEPOINTS; i++) {
			if (timepoint_3[i] == 0 || timepoint_4[i] == 0) {
				continue;
			}

			time_diff = ktime_sub(timepoint_4[i], timepoint_3[i]);
			if (i == 0) {
				mean_time = time_diff;
				min_time = time_diff;
				max_time = time_diff;
				ntimeps++;
				continue;
			}
			mean_time += time_diff;
			ntimeps++;
			if (time_diff < min_time)
				min_time = time_diff;
			if (time_diff > max_time)
				max_time = time_diff;
		}

		mean_time = (ntimeps <= 0) ? 0 : (mean_time / ntimeps);
		pos += sprintf(buf + pos, "Timepoint_3 <-> Timepoint_4\n");
		pos += sprintf(buf + pos, "Min time diff : ");
		pos += sprintf(buf + pos, "%lld ns\n", min_time);
		pos += sprintf(buf + pos, "Max time diff : ");
		pos += sprintf(buf + pos, "%lld ns\n", max_time);
		pos += sprintf(buf + pos, "Mean time diff : ");
		pos += sprintf(buf + pos, "%lld ns\n", mean_time);
		pos += sprintf(buf + pos, "\n");
	}

	return pos;
}

void si_pep_iomem_read_rep(void *dst, void __iomem *src, size_t size)
{
	size_t lrsize = size / 4;
	size_t brsize = size % 4;
	u32 *s = (u32 *)src;
	u32 *d = (u32 *)dst;

	while (lrsize) {
		*d = readl(s);
		d++;
		s++;
		lrsize--;
	}

	while (brsize) {
		*(u8 *)d = readb(s);
		(u8 *)d++;
		(u8 *)s++;
		brsize--;
	}
}

void si_pep_iomem_write_rep(void __iomem *dst, void *src, size_t size)
{
	size_t lwsize = size / 4;
	size_t bwsize = size % 4;
	u32 *s = (u32 *)src;
	u32 *d = (u32 *)dst;

	while (lwsize) {
		writel(*s, d);
		d++;
		s++;
		lwsize--;
	}

	while (bwsize) {
		writeb(*(u8 *)s, d);
		(u8 *)d++;
		(u8 *)s++;
		bwsize--;
	}
}

void si_pep_iomem_memset(void __iomem *dst, u8 val, size_t size)
{
	size_t lwsize = size / 4;
	size_t bwsize = size % 4;
	u32 *d = (u32 *)dst;
	u32 l;

	memset(&l, val, sizeof(l));

	while (lwsize) {
		writel(l, d);
		d++;
		lwsize--;
	}

	while (bwsize) {
		writeb(val, d);
		(u8 *)d++;
		bwsize--;
	}
}

void *si_pep_dma_regs(struct si_pep_dev *pep, int ch_num, enum si_dma_dir dir)
{
	if (pep->hdma_base != 0) { /* Modalix */
		return (pep->hdma_base + (ch_num * SI_DMA_INTERCH_SPACING +
		(dir == DIR_WRITE?SI_DMA_WCH_OFFSET:SI_DMA_RCH_OFFSET)));
	}

	return (pep->dbi2_base +
		(SI_PF0_DMA_REGS + ch_num * SI_DMA_INTERCH_SPACING +
		 (dir == DIR_WRITE?SI_DMA_WCH_OFFSET:SI_DMA_RCH_OFFSET)));
}

int si_pep_register_irq_handler(struct si_pep_dev *pep, char *irqname,
		irqreturn_t (*irq_handler)(int, void*), void *param)
{
	int irq;
	int ret;
	struct device *dev = pep->dev;

	irq = platform_get_irq_byname(pep->pdev, irqname);
	if (irq < 0) {
		si_err(pep, "Failed to find IRQ for '%s'", irqname);
		return irq;
	}

	ret = devm_request_irq(dev, irq, irq_handler, IRQF_NO_THREAD,
			pep->dev->driver->name, param);
	if (ret) {
		si_err(pep, "Failed to request IRQ 0x%x for '%s'", irq, irqname);
		return ret;
	}

	return 0;
}

#define IRQ_PARAM_LIST \
	IP(0) IP(1) IP(2) IP(3) IP(4) IP(5) IP(6) IP(7) \
	IP(8) IP(9) IP(10) IP(11) IP(12) IP(13) IP(14) IP(15) \
	IP(16) IP(17) IP(18) IP(19) IP(20) IP(21) IP(22) IP(23) IP(24) \
	IP(25) IP(26) IP(27) IP(28) IP(29) IP(30) IP(31)

struct si_pep_edma_irq_param w_irq_param[] = {
#define IP(c) \
	{ \
		.ch = c, \
		.dir = DIR_WRITE \
	},
	IRQ_PARAM_LIST
#undef IP
};

struct si_pep_edma_irq_param r_irq_param[] = {
#define IP(c) \
	{ \
		.ch = c, \
		.dir = DIR_READ \
	},
	IRQ_PARAM_LIST
#undef IP
};

irqreturn_t si_pep_dma_irq_handler(int irq, void *arg)
{
	struct si_pep_edma_irq_param *param =
		(struct si_pep_edma_irq_param *)arg;
	struct si_pep_dev *pep = g_pep_dev;
	volatile u32 int_status;
	volatile u32 dma_status;
	volatile u32 reg = 0;
	int ch = param->ch;
	int dir = param->dir;

	si_dbg_l(pep, "Called, irq %d, channel %d", irq, ch);
	int_status = DMA_REG_READ(pep, ch, dir, int_status);
	dma_status = DMA_REG_READ(pep, ch, dir, status);
	si_dbg_l(pep, "IRQ: %d [ch: %d, dir: %d] int_status: 0x%x, status:0x%x",
		     irq, ch, dir, int_status, dma_status);
	reg = int_status & DMA_INT_CLEAR_INTS_MASK;
	DMA_REG_WRITE(pep, ch, dir, int_clear, reg);
	DMA_REG_WRITE(pep, ch, dir, int_setup, 0);

	if ((DMA_INT_ST_INTS_MASK & int_status) == 0) {
		si_err(pep, "Spurios interrupt");
		return IRQ_HANDLED;
	}

	if (dma_status == DMA_STATUS_ABORTED) {
		si_err(pep, "DMA Operation aborted");
		pep->drv_ops->set_soc_state(pep, SOC_STATE_INIT);
		return IRQ_HANDLED;
	} else if (dma_status == DMA_STATUS_RUNNING) {
		si_err(pep, "DMA Still Running");
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

irqreturn_t si_pep_irq_link_down(int irq, void *arg)
{
	struct si_pep_dev *pep = (struct si_pep_dev *)arg;

	if (unlikely(!pep)) {
		pr_err("%s: Invalid pointer\n", __func__);
		return IRQ_NONE;
	}

	pep->drv_ops->set_soc_state(pep, SOC_STATE_INIT);
	queue_work(system_wq, &pep->link_down_handler.work);

	return IRQ_HANDLED;
}

static int si_pep_pcie_read(void __iomem *addr, int size, u32 *val)
{
	if (!IS_ALIGNED((uintptr_t)addr, size)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 4) {
		*val = readl(addr);
	} else if (size == 2) {
		*val = readw(addr);
	} else if (size == 1) {
		*val = readb(addr);
	} else {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int si_pep_pcie_write(void __iomem *addr, int size, u32 val)
{
	if (!IS_ALIGNED((uintptr_t)addr, size))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static u32 si_pep_pcie_read_dbi(struct si_pep_dev *pep, u32 reg, size_t size)
{
	int ret;
	u32 val = 0;

	ret = si_pep_pcie_read(pep->dbi_base + reg, size, &val);
	if (ret)
		si_err(pep, "Failed to read 0x%08x, error %d\n", reg, ret);

	return val;
}

static void si_pep_pcie_write_dbi(struct si_pep_dev *pep, u32 reg, size_t size,
		u32 val)
{
	int ret;

	ret = si_pep_pcie_write(pep->dbi_base + reg, size, val);
	if (ret)
		si_err(pep, "Failed to write 0x%08x, error %d\n", reg, ret);
}

static inline void si_pep_pcie_writel_dbi(struct si_pep_dev *pep, u32 reg,
					  u32 val)
{
	si_pep_pcie_write_dbi(pep, reg, 4, val);
}

static inline u32 si_pep_pcie_readl_dbi(struct si_pep_dev *pep, u32 reg)
{
	return si_pep_pcie_read_dbi(pep, reg, 4);
}

static inline void si_pep_pcie_writew_dbi(struct si_pep_dev *pep, u32 reg,
					  u16 val)
{
	si_pep_pcie_write_dbi(pep, reg, 2, val);
}

static inline u16 si_pep_pcie_readw_dbi(struct si_pep_dev *pep, u32 reg)
{
	return si_pep_pcie_read_dbi(pep, reg, 2);
}

static inline void si_pep_pcie_writeb_dbi(struct si_pep_dev *pep, u32 reg,
					  u8 val)
{
	si_pep_pcie_write_dbi(pep, reg, 1, val);
}

static inline u8 si_pep_pcie_readb_dbi(struct si_pep_dev *pep, u32 reg)
{
	return si_pep_pcie_read_dbi(pep, reg, 1);
}

static u8 si_pep_pcie_find_next_cap(struct si_pep_dev *pep, u8 cap_ptr,
				    u8 cap)
{
	u8 cap_id;
	u8 next_cap_ptr;
	u16 reg;

	if (!cap_ptr)
		return 0;

	reg = si_pep_pcie_readw_dbi(pep, cap_ptr);
	cap_id = (reg & 0x00ff);

	if (cap_id > PCI_CAP_ID_MAX)
		return 0;

	if (cap_id == cap)
		return cap_ptr;

	next_cap_ptr = (reg & 0xff00) >> 8;
	return si_pep_pcie_find_next_cap(pep, next_cap_ptr, cap);
}

static u8 si_pep_pcie_find_capability(struct si_pep_dev *pep, u8 cap)
{
	u8 next_cap_ptr;
	u16 reg;

	reg = si_pep_pcie_readw_dbi(pep, PCI_CAPABILITY_LIST);
	next_cap_ptr = (reg & 0x00ff);

	return si_pep_pcie_find_next_cap(pep, next_cap_ptr, cap);
}

static void si_pep_get_msi_prams(struct si_pep_dev *pep, u32 *msg_addr_lower,
				 u32 *msg_addr_upper)
{
	u32 msi_cap;
	u32 msi_reg;
	bool has_upper;
	u16 msg_ctrl;

	msi_cap = si_pep_pcie_find_capability(pep, PCI_CAP_ID_MSI);
	msi_reg = msi_cap + PCI_MSI_FLAGS;
	msg_ctrl = si_pep_pcie_readw_dbi(pep, msi_reg);
	has_upper = msg_ctrl & PCI_MSI_FLAGS_64BIT;
	msi_reg = msi_cap + PCI_MSI_ADDRESS_LO;
	*msg_addr_lower = si_pep_pcie_readl_dbi(pep, msi_reg);
	if (has_upper) {
		msi_reg = msi_cap + PCI_MSI_ADDRESS_HI;
		*msg_addr_upper = si_pep_pcie_readl_dbi(pep, msi_reg);
	} else {
		*msg_addr_upper = 0;
	}
}

int si_pep_copy_to_host(struct si_pep_dev *pep, u64 dest, u64 src, u32 size,
		int ch, int msi)
{
	int dir = DIR_WRITE;
	volatile u32 reg;
	int retry_count = 3;

	if ((pep->state != SOC_STATE_RUN) && (msi == -1))
		return -EAGAIN;

	if (!pep->modalix)
		ch = 0;

	si_dbg_dma(pep, "DMA%d from 0x%llx to 0x%llx, size %u, msi %d", ch, src,
			dest, size, msi);
	spin_lock(&pep->dma_write_lock[ch]);
	DMA_REG_WRITE(pep, ch, dir, en, DMA_EN_ENABLE);
	DMA_REG_WRITE(pep, ch, dir, sar_high, upper_32_bits(src));
	DMA_REG_WRITE(pep, ch, dir, sar_low, lower_32_bits(src));
	DMA_REG_WRITE(pep, ch, dir, dar_high, upper_32_bits(dest));
	DMA_REG_WRITE(pep, ch, dir, dar_low, lower_32_bits(dest));
	DMA_REG_WRITE(pep, ch, dir, control1, 0x2);
	DMA_REG_WRITE(pep, ch, dir, int_clear, DMA_INT_CLEAR_INTS_MASK);
	DMA_REG_WRITE(pep, ch, dir, int_setup, DMA_INT_SETUP_LAIE);
	if (msi >= 0) {
		if (!pep->msg_addr_lower)
			si_pep_get_msi_prams(pep, &pep->msg_addr_lower,
					&pep->msg_addr_upper);
		DMA_REG_WRITE(pep, ch, dir, msi_stop_low, pep->msg_addr_lower);
		DMA_REG_WRITE(pep, ch, dir, msi_stop_high,
				pep->msg_addr_upper);
		DMA_REG_WRITE(pep, ch, dir, msi_abort_low,
			      pep->msg_addr_lower);
		DMA_REG_WRITE(pep, ch, dir, msi_abort_high,
			      pep->msg_addr_upper);
		DMA_REG_WRITE(pep, ch, dir, msi_msgd, msi);
		DMA_REG_WRITE(pep, ch, dir, int_setup,
				DMA_INT_SETUP_LAIE |
				DMA_INT_SETUP_RAIE | DMA_INT_SETUP_RSIE);
	}

	while (retry_count-- >= 0) {
		DMA_REG_WRITE(pep, ch, dir, xfersize, size);
		DMA_REG_WRITE(pep, ch, dir, doorbell, DMA_DOORBELL_DB_START);

		do { /* Wait for completion|error */
			reg = DMA_REG_READ(pep, ch, dir, status);
		} while ((DMA_STATUS_STATUS_MASK & reg) == DMA_STATUS_RUNNING);

		if ((DMA_STATUS_STATUS_MASK & reg) == DMA_STATUS_ABORTED) {
			if (retry_count > 0)
				continue;

			reg = DMA_REG_READ(pep, ch, dir, int_status);
			si_err(pep, "DMA Aborted: int status 0x%x. From 0x%llx "
					"to 0x%llx, size %u, msi %d",
			       reg, src, dest, size, msi);
			spin_unlock(&pep->dma_write_lock[ch]);
			return -EIO;
		}
		break;
	}
	DMA_REG_WRITE(pep, ch, dir, en, 0);
	spin_unlock(&pep->dma_write_lock[ch]);

	return 0;
}

int si_pep_copy_from_host(struct si_pep_dev *pep, u64 dest, u64 src, u32 size,
		int ch)
{
	int dir = DIR_READ;
	int retry_count = 3;
	u32 reg;

	if (pep->state != SOC_STATE_RUN)
		return -EAGAIN;

	if (!pep->modalix)
		ch = 0;

	si_dbg_dma(pep, "DMA%d from 0x%llx to 0x%llx, size %u", ch, src, dest,
			size);
	spin_lock(&pep->dma_read_lock[ch]);
	DMA_REG_WRITE(pep, ch, dir, en, DMA_EN_ENABLE);
	DMA_REG_WRITE(pep, ch, dir, sar_high, upper_32_bits(src));
	DMA_REG_WRITE(pep, ch, dir, sar_low, lower_32_bits(src));
	DMA_REG_WRITE(pep, ch, dir, dar_high, upper_32_bits(dest));
	DMA_REG_WRITE(pep, ch, dir, dar_low, lower_32_bits(dest));
	DMA_REG_WRITE(pep, ch, dir, control1, 0x2);
	DMA_REG_WRITE(pep, ch, dir, int_clear, DMA_INT_CLEAR_INTS_MASK);
	DMA_REG_WRITE(pep, ch, dir, int_setup, 0);

	while (retry_count-- >= 0) {
		DMA_REG_WRITE(pep, ch, dir, xfersize, size);
		DMA_REG_WRITE(pep, ch, dir, doorbell, DMA_DOORBELL_DB_START);

		do { /* Wait for completion|error */
			reg = DMA_REG_READ(pep, ch, dir, status);
		} while ((DMA_STATUS_STATUS_MASK & reg) == DMA_STATUS_RUNNING);

		if ((DMA_STATUS_STATUS_MASK & reg) == DMA_STATUS_ABORTED) {
			if (retry_count > 0)
				continue;

			reg = DMA_REG_READ(pep, ch, dir, int_status);
			si_err(pep, "Transfer Aborted: int status 0x%x. "
					"From 0x%llx to 0x%llx, size %u",
			       reg, src, dest, size);
			spin_unlock(&pep->dma_read_lock[ch]);
			return -EIO;
		}
		break;
	}
	DMA_REG_WRITE(pep, ch, dir, en, 0);

	spin_unlock(&pep->dma_read_lock[ch]);
	return 0;
}

int si_pep_copy_do_ll(struct si_pep_dev *pep, struct si_pep_dma_ll_ctx *llctx,
		int ch, int msi)
{
	int dir;
	volatile u32 reg;
	spinlock_t *lock;

	if ((pep->state != SOC_STATE_RUN) && (msi == -1))
		return -EAGAIN;

	if (!pep->modalix)
		ch = 0;

	if (llctx->dir >= DIR_MAX) {
		si_err(pep, "Invalid transfer context");
		return -EINVAL;
	}

	dir = llctx->dir;
	lock = (dir==DIR_READ)?&pep->dma_write_lock[ch]:&pep->dma_read_lock[ch];

	si_dbg_dma(pep, "DMA %s with linked list on channel %d, msi %d",
			(dir==DIR_READ)?"read":"write", ch, msi);

	spin_lock(lock);
	DMA_REG_WRITE(pep, ch, dir, en, DMA_EN_ENABLE);
        DMA_REG_WRITE(pep, ch, dir, int_clear, DMA_INT_CLEAR_INTS_MASK);
        DMA_REG_WRITE(pep, ch, dir, control1, 0x03);
        DMA_REG_WRITE(pep, ch, dir, cycle, 0x02);
        DMA_REG_WRITE(pep, ch, dir, watermark_en, 0x03);
        DMA_REG_WRITE(pep, ch, dir, xfersize, 0);
        DMA_REG_WRITE(pep, ch, dir, sar_low, 0);
        DMA_REG_WRITE(pep, ch, dir, sar_high, 0);
        DMA_REG_WRITE(pep, ch, dir, dar_low, 0);
        DMA_REG_WRITE(pep, ch, dir, dar_high, 0);
        DMA_REG_WRITE(pep, ch, dir, llp_low, lower_32_bits(llctx->p_dma_ll));
        DMA_REG_WRITE(pep, ch, dir, llp_high, upper_32_bits(llctx->p_dma_ll));

	if ((dir == DIR_WRITE) && (msi >= 0)) {
		if (!pep->msg_addr_lower)
			si_pep_get_msi_prams(pep, &pep->msg_addr_lower,
					&pep->msg_addr_upper);
		DMA_REG_WRITE(pep, ch, dir, msi_stop_low, pep->msg_addr_lower);
		DMA_REG_WRITE(pep, ch, dir, msi_stop_high,
				pep->msg_addr_upper);
		DMA_REG_WRITE(pep, ch, dir, msi_abort_low,
			      pep->msg_addr_lower);
		DMA_REG_WRITE(pep, ch, dir, msi_abort_high,
			      pep->msg_addr_upper);
		DMA_REG_WRITE(pep, ch, dir, msi_msgd, msi);
		DMA_REG_WRITE(pep, ch, dir, int_setup,
				DMA_INT_SETUP_LAIE |
				DMA_INT_SETUP_RAIE | DMA_INT_SETUP_RSIE);
	}

	DMA_REG_WRITE(pep, ch, dir, doorbell, DMA_DOORBELL_DB_START);
	do { /* Wait for completion|error */
		reg = DMA_REG_READ(pep, ch, dir, status);
	} while ((DMA_STATUS_STATUS_MASK & reg) == DMA_STATUS_RUNNING);

	if ((DMA_STATUS_STATUS_MASK & reg) == DMA_STATUS_ABORTED) {
		reg = DMA_REG_READ(pep, ch, dir, int_status);
		si_err(pep, "DMA Aborted: INT status 0x%x, msi %d", reg, msi);
		spin_unlock(lock);
		return -EIO;
	}

	DMA_REG_WRITE(pep, ch, dir, en, 0);
	spin_unlock(lock);

	return 0;
}

/*
 * Management command handlers
 */
static int si_pep_destroy_queue(struct si_pep_dev *pep, enum si_qtype qtype,
				void *params)
{
	u32 qid;
	struct device *dev = pep->dev;
	struct si_pep_qdev *qdev;
	struct si_qctx *ctx;
	dev_t devt;
	unsigned int major;
	size_t size;
	char *qstr = si_pep_qtype_str(qtype);

	qid = *((u32 *)params);
	si_minfo(pep, "Destroying %s queue %u", qstr, qid);
	switch (qtype) {
	case SI_MWQ:
	case SI_DWQ:
		if (qtype == SI_MWQ)
			ctx = pep->mwq_ctx;
		else
			ctx = pep->dwq_ctx[qid];

		if (ctx->status == SI_INVALID) {
			si_err(pep, "Couldn't find %s queue %u", qstr, qid);
			return -ENOENT;
		}

		si_pep_iomem_memset(ctx, 0, sizeof(struct si_qctx));
		if (qtype == SI_DWQ) {
			dq_set_wq_tail(pep, qid, 0);
			major = MAJOR(pep->devt);
			devt = MKDEV(major, qid + 1);
			device_destroy(pep->class, devt);
			kill_fasync(&pep->fasync, SIGIO, POLL_IN);
			mutex_lock(&pep->dreq_cache_mutex[qid]);
			pep->dchdr[qid].nreqs = 0;
			pep->dchdr[qid].maxreqs = SI_DWQ_CACHE_SIZE;
			pep->dchdr[qid].head = 0;
			pep->dchdr[qid].tail = 0;
			pep->dreq_cached[qid] = 0;
			mutex_unlock(&pep->dreq_cache_mutex[qid]);
		} else {
			mq_set_wq_tail(pep, 0);
			atomic_set(&pep->mwq_prev_head, 0);
			mutex_lock(&pep->mreq_cache_mutex);
			pep->mchdr.nreqs = 0;
			pep->mchdr.maxreqs = SI_DWQ_CACHE_SIZE;
			pep->mchdr.head = 0;
			pep->mchdr.tail = 0;
			pep->mreq_cached = 0;
			mutex_unlock(&pep->mreq_cache_mutex);
		}
		break;
	case SI_MCQ:
	case SI_DCQ:
		if (qtype == SI_MCQ) {
			ctx = pep->mcq_ctx;
			qdev = &pep->qdev[0];
		} else {
			ctx = pep->dcq_ctx[qid];
			qdev = &pep->qdev[qid + 1];
		}

		if (ctx->status == SI_INVALID) {
			si_err(pep, "Couldn't find %s queue %u", qstr, qid);
			return -ENOENT;
		}

		si_pep_iomem_memset(ctx, 0, sizeof(struct si_qctx));
		atomic_set(&qdev->seqid, 0);
		if (qtype == SI_MCQ)
			mq_set_cq_head(pep, 0);
		else
			dq_set_cq_head(pep, qid, 0);
		break;
	case SI_MRQ:
	case SI_DRQ:
		if (qtype == SI_MRQ)
			ctx = pep->mrq_ctx;
		else
			ctx = pep->drq_ctx[qid];

		if (ctx->status == SI_INVALID) {
			si_err(pep, "Couldn't find %s queue %u", qstr, qid);
			return -ENOENT;
		}

		size = ctx->entries * sizeof(struct si_rqe);
		si_pep_iomem_memset(ctx, 0, sizeof(struct si_qctx));
		ctx->status = SI_INVALID;
		if (qtype == SI_DRQ) {
			dma_free_coherent(dev, size, pep->drqe_caches[qid],
					pep->p_drqe_caches[qid]);
			pep->drqe_caches[qid] = NULL;
			pep->p_drqe_caches[qid] = 0;
			dq_set_rq_head(pep, qid, 0);
		} else {
			dma_free_coherent(dev, size, pep->mrqe_cache,
					pep->p_mrqe_cache);
			pep->mrqe_cache = NULL;
			pep->mrqe_cache = 0;
			mq_set_rq_head(pep, 0);
		}
		break;
	default:
		si_err(pep, "Unknown queue type");
		return -EINVAL;
	}

	pep->drv_ops->queue_deleted_callback(pep, qtype, qid);
	return 0;
}

static int si_pep_create_queue(struct si_pep_dev *pep, struct si_qctx *ctx)
{
	struct device *dev = pep->dev;
	enum si_qtype t = ctx->qtype;
	unsigned int major;
	size_t size = sizeof(struct si_qctx);
	int tp_size;
	int ret;
	u32 qid = ctx->qid;

	major = MAJOR(pep->devt);
	si_minfo(pep, "Creating %s queue %d, entries %d", si_pep_qtype_str(t),
			qid, ctx->entries);
	switch (t) {
	case SI_MWQ:
		if (pep->mwq_ctx->status == SI_ACTIVE) {
			si_pep_destroy_queue(pep, SI_MWQ, &qid);
		}
		memcpy(pep->mwq_ctx, ctx, size);
		atomic_set(&pep->mwq_ctx->head, 0);
		atomic_set(&pep->mwq_ctx->tail, 0);
		atomic_set(&pep->mwq_prev_head, 0);
		pep->mwq_ctx->status = SI_ACTIVE;
		mq_set_wq_tail(pep, 0);
		break;
	case SI_MCQ:
		if (pep->mcq_ctx->status == SI_ACTIVE) {
			si_pep_destroy_queue(pep, SI_MCQ, &qid);
		}
		memcpy(pep->mcq_ctx, ctx, size);
		atomic_set(&pep->mcq_ctx->head, 0);
		atomic_set(&pep->mcq_ctx->tail, 0);
		pep->mcq_ctx->status = SI_ACTIVE;
		mq_set_cq_head(pep, 0);
		break;
	case SI_MRQ:
		if (pep->mrq_ctx->status == SI_ACTIVE) {
			si_pep_destroy_queue(pep, SI_MRQ, &qid);
		}
		memcpy(pep->mrq_ctx, ctx, size);
		atomic_set(&pep->mrq_ctx->head, 0);
		atomic_set(&pep->mrq_ctx->tail, 0);
		size = ctx->entries * sizeof(struct si_rqe);
		pep->mrqe_cache = dma_alloc_coherent(dev, size,
				&pep->p_mrqe_cache, GFP_KERNEL);
		if (pep->mrqe_cache == NULL) {
			si_err(pep, "Failed to setup mrqe caches");
			return -ENOMEM;
		}

		ret = si_pep_copy_from_host(pep, pep->p_mrqe_cache,
				ctx->qe_pb, size, MWQ_CACHE_DMA_CH);
		if (ret) {
			si_err(pep, "Failed to cache rqes");
			dma_free_coherent(dev, size, pep->mrqe_cache,
					pep->p_mrqe_cache);
			return -EIO;
		}
		pep->mrq_ctx->status = SI_ACTIVE;
		mq_set_rq_head(pep, 0);
		break;
	case SI_DWQ:
		if (pep->dwq_ctx[qid]->status == SI_ACTIVE) {
			si_pep_destroy_queue(pep, SI_DWQ, &qid);
		}

		if (dump_queue_occupency == true) {
			tp_size = sizeof(ktime_t) * MAX_TIMEPOINTS;
			memset(pep->qdev[qid+1].timepoint_1, 0, tp_size);
			memset(pep->qdev[qid+1].timepoint_2, 0, tp_size);
			memset(pep->qdev[qid+1].timepoint_3, 0, tp_size);
			memset(pep->qdev[qid+1].timepoint_4, 0, tp_size);
		}

		memcpy(pep->dwq_ctx[qid], ctx, size);
		atomic_set(&pep->dwq_ctx[qid]->head, 0);
		atomic_set(&pep->dwq_ctx[qid]->tail, 0);
		atomic_set(&pep->dwq_prev_heads[qid], 0);

		device_create(pep->class, NULL, MKDEV(major, qid + 1),
			      &pep->qdev[qid + 1],
			      &pep->qdev[qid + 1].devname[0]);

		if (use_driver_loopback) {
			wake_up_process(pep->loopback_hdlr[qid]);
		}

		pep->dwq_ctx[qid]->status = SI_ACTIVE;
		dq_set_wq_tail(pep, qid, 0);
		break;
	case SI_DCQ:
		if (pep->dcq_ctx[qid]->status == SI_ACTIVE) {
			si_pep_destroy_queue(pep, SI_DCQ, &qid);
		}
		memcpy(pep->dcq_ctx[qid], ctx, size);
		atomic_set(&pep->dcq_ctx[qid]->head, 0);
		atomic_set(&pep->dcq_ctx[qid]->tail, 0);
		pep->dcq_ctx[qid]->status = SI_ACTIVE;
		dq_set_cq_head(pep, qid, 0);
		break;
	case SI_DRQ:
		if (pep->drq_ctx[qid]->status == SI_ACTIVE) {
			si_pep_destroy_queue(pep, SI_DRQ, &qid);
		}
		memcpy(pep->drq_ctx[qid], ctx, size);
		atomic_set(&pep->drq_ctx[qid]->head, 0);
		atomic_set(&pep->drq_ctx[qid]->tail, 0);
		si_minfo(pep, "Create RQE cache for %d entries", ctx->entries);
		size = ctx->entries * sizeof(struct si_rqe);
		pep->drqe_caches[qid] = dma_alloc_coherent(dev, size,
				&pep->p_drqe_caches[qid], GFP_KERNEL);
		if (pep->drqe_caches[qid] == NULL) {
			si_err(pep, "Failed to setup rqe caches");
			return -ENOMEM;
		}

		ret = si_pep_copy_from_host(pep, pep->p_drqe_caches[qid],
				ctx->qe_pb, size, DWQ_CACHE_DMA_CH);
		if (ret) {
			si_err(pep, "Failed to cache rqes");
			dma_free_coherent(dev, size, pep->drqe_caches[qid],
					pep->p_drqe_caches[qid]);
			return -EIO;
		}

		pep->drq_ctx[qid]->status = SI_ACTIVE;
		dq_set_rq_head(pep, qid, 0);
		break;
	default:
		si_err(pep, "Unknown queue type");
		return -EINVAL;
	}

	pep->drv_ops->queue_created_callback(pep, t, qid);
	si_minfo(pep, "%s queue %d successfully created", si_pep_qtype_str(t),
			qid);
	return 0;
}

void si_pep_flush_dreqs(struct si_pep_dev *pep, u32 qid)
{
	struct si_pep_req_cache_hdr *chdr = &pep->dchdr[qid];

	if (queue_is_bad(qid)) {
		si_err(pep, "Invalid queue");
		return;
	}

	mutex_lock(&pep->dreq_cache_mutex[qid]);
	chdr->head = 0;
	chdr->tail = 0;
	chdr->nreqs = 0;
	pep->dreq_cached[qid] = 0;
	mutex_unlock(&pep->dreq_cache_mutex[qid]);
}

static int si_pep_abort_io(struct si_pep_dev *pep, void *params)
{
	int qid;
	int idx;
	u16 head;
	u16 tail;
	struct si_qe_id *qidx;
	struct si_pep_qdev *qdev;
	struct si_wqe *rcache;
	struct si_pep_req_cache_hdr *rchdr;
	struct si_wqe wqe = { 0 };
	struct si_io io = { 0 };
	struct mutex *m;

	qidx = (struct si_qe_id *)params;
	if (qidx->qtype == SI_MWQ) {
		rcache = pep->mreq_cache;
		rchdr = &pep->mchdr;
		m = &pep->mreq_cache_mutex;
		qid = 0;
	} else if (qidx->qtype == SI_DWQ) {
		rcache = pep->dreq_caches[qidx->qid];
		rchdr = &pep->dchdr[qidx->qid];
		m = &pep->dreq_cache_mutex[qidx->qid];
		qid = qidx->qid + 1;
	} else {
		si_err(pep, "Invalid type for abort io");
		return -EINVAL;
	}

	idx = qidx->idx;
	qdev = &pep->qdev[qid];

	mutex_lock(m);
	head = rchdr->head;
	tail = rchdr->tail;
	while (head != tail) {
		if (rcache[head].qidx.idx != idx) {
			head++;
			if (head >= rchdr->maxreqs)
				head = 0;
			continue;
		}

		si_qerr(qdev, "Aborting unprocessed request %d. Seq: %u",
				idx, wqe.seqid);
		memcpy(&wqe, &rcache[head], sizeof(struct si_wqe));
		memset(&rcache[head], 0, sizeof(struct si_wqe));
		qdev->stats.abort_reqs_suc++;
		break;
	}
	mutex_unlock(m);

	if (wqe.qidx.qtype == 0)
		qdev->stats.abort_reqs_fail++;

	io.cmd = wqe.cmd;
	io.resp = SI_RES_ABORTED;
	return si_pep_send_resp(qdev, &io, NULL);
}

static int si_pep_handle_setup_soc_cmd(struct si_pep_dev *pep,
				       struct si_mcmd *mcmd)
{
	struct si_setup_soc_params *params =
		(struct si_setup_soc_params*)mcmd->params;

	pep->host_alert_addr = ((u64)(params->alert_addr_high) << 32) |
				params->alert_addr_low;

	return SI_RES_SUCCESS;
}

char * const prog = "/bin/sh";
char * const arg1 = "-c";
char *envp[] = {
	"HOME=/",
	"LANG=C",
	"PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
	NULL,
};

static void si_pep_teardown_netintf(struct si_pep_dev *pep)
{
	char intfname[IFNAMSIZ];
	char command[256];
	char *argv[] = {prog, arg1, command, NULL};

	if (pep->net_dev.created == 0)
		return;

	strncpy(&intfname[0], pep->net_dev.ndev->name, IFNAMSIZ);
	snprintf(command, sizeof(command), "ifconfig %s down", intfname);
	call_usermodehelper(prog, argv, envp, UMH_WAIT_PROC | UMH_KILLABLE);
	si_pep_net_cleanup(pep);
	pep->net_dev.net_addr = cpu_to_be32(0);
	pep->net_dev.net_mask = cpu_to_be32(0);
	pep->net_dev.created = 0;
}

static int si_pep_setup_neintf(struct si_pep_dev *pep)
{
	int ret;

	if (pep->card_num == -1) /* Host hasn't called setup yet */
		return -ENOENT;

	if (pep->net_dev.host_hwdata_p == 0)
		return -ENOENT;

	if (pep->net_dev.created) {
		si_err(pep, "Interface exists");
		return -EEXIST;
	}

	ret = si_pep_net_init(pep);
	if (ret) {
		si_err(pep, "Failed to setup network interface");
		return ret;
	}

	pep->net_dev.created = 1;

	return 0;
}

static int si_pep_set_neintf_ifaddr(struct si_pep_dev *pep,
		struct si_mcmd_set_ipaddr_params *ifaddr)
{
	int ret;
	struct si_pep_net_dev *netdev = &pep->net_dev;
	char addr[16] = { 0 };
	char mask[16] = { 0 };
	char intfname[IFNAMSIZ] = { 0 };
	char command[256];
	char *argv[] = {prog, arg1, command, NULL};

	if (pep->net_dev.created == 0) {
		si_err(pep, "Ethernet Interface not created");
		return -ENODEV;
	}

	if (ifaddr->net_addr == netdev->net_addr &&
			ifaddr->net_mask == netdev->net_mask)
		return -EEXIST;

	strncpy(&intfname[0], pep->net_dev.ndev->name, IFNAMSIZ);
	netdev->net_addr = ifaddr->net_addr;
	netdev->net_mask = ifaddr->net_mask;
	snprintf(&addr[0], sizeof(addr), "%pI4", &ifaddr->net_addr);
	snprintf(&mask[0], sizeof(mask), "%pI4", &ifaddr->net_mask);
	snprintf(command, sizeof(command), "ifconfig %s %s netmask %s up",
			intfname, addr, mask);
	ret = call_usermodehelper(prog, argv, envp, UMH_WAIT_PROC|UMH_KILLABLE);
	if (ret) {
		si_err(pep, "Failed to set IPv4 address for interface '%s'",
				pep->net_dev.ndev->name);
		si_err(pep, "Set Interface IP to %pI4/%pI4", &netdev->net_addr,
				&netdev->net_mask);
		return -EIO;
	}

	return 0;
}

void si_pep_handle_mcmd(struct si_pep_dev *pep, struct si_mcmd *mcmd_in)
{
	struct si_mcmd mcmd = { 0 };
	struct si_mcmd_res mcmd_res = { 0 };
	struct si_mcmd_timesync_params *time;
	struct timespec64 tp;
	struct timezone tz;
	struct si_qctx *ctx;
	enum si_qstatus cur_status;
	int res = SI_RES_SUCCESS;
	int ret;
	void *params;
	u32 qid;

	si_pep_iomem_read_rep(&mcmd, mcmd_in, sizeof(mcmd));
	params = &mcmd.params;

	si_minfo(pep, "Handle Management command 0x%04x", mcmd.cmd);
	
	if (mcmd.cmd == SI_MCMD_REBOOT_SOC) {
		goto mcmd_restart;
	}

	switch (mcmd.cmd) {
	case SI_MCMD_SETUP_SOC:
		res = si_pep_handle_setup_soc_cmd(pep, &mcmd);
		break;
	case SI_MCMD_CREATE_DWQS:
	case SI_MCMD_CREATE_DCQS:
	case SI_MCMD_CREATE_DRQS:
	case SI_MCMD_CREATE_MWQS:
	case SI_MCMD_CREATE_MCQS:
	case SI_MCMD_CREATE_MRQS:
		ctx = (struct si_qctx*)params;
		ret = si_pep_create_queue(pep, ctx);
		if (ret != 0) {
			si_err(pep, "Failed to create %s queue %d",
				si_pep_qtype_str(ctx->qtype), ctx->qid);
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_DELETE_DWQ:
		if (si_pep_destroy_queue(pep, SI_DWQ, params) != 0) {
			si_err(pep, "Failed to delete data work queues");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_DELETE_DCQ:
		if (si_pep_destroy_queue(pep, SI_DCQ, params) != 0) {
			si_err(pep, "Failed to delete data completion queues");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_DELETE_DRQ:
		if (si_pep_destroy_queue(pep, SI_DRQ, params) != 0) {
			si_err(pep, "Failed to delete data receive queues");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_DELETE_MWQ:
		if (si_pep_destroy_queue(pep, SI_MWQ, params) != 0) {
			si_err(pep, "Failed to delete management work queue");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_DELETE_MCQ:
		if (si_pep_destroy_queue(pep, SI_MCQ, params) != 0) {
			si_err(pep, "Failed to delete management completion queue");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_DELETE_MRQ:
		if (si_pep_destroy_queue(pep, SI_MRQ, params) != 0) {
			si_err(pep, "Failed to delete management receive queue");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_ABORT_REQ:
		if (si_pep_abort_io(pep, params) != 0) {
			si_err(pep, "Failed to abort io");
			res = SI_RES_FAILURE;
		}
		break;
	case SI_MCMD_FLUSH_REQS:
		qid = *((u32 *)params);
		cur_status = pep->dwq_ctx[qid]->status;
		pep->dwq_ctx[qid]->status = SI_FLUSHING;
		si_minfo(pep, "Flushing data queue %d", qid);
		si_pep_flush_dreqs(pep, qid);
		pep->dwq_ctx[qid]->status = cur_status;
		break;
	case SI_MCMD_SET_TIME:
		time = (struct si_mcmd_timesync_params *)params;
		si_minfo(pep, "Setting sytem time");
		tp.tv_sec = time->time_sec_hi;
		tp.tv_sec = (tp.tv_sec << 32) | time->time_sec_lo;
		tp.tv_nsec = time->time_nsec_hi;
		tp.tv_nsec = (tp.tv_nsec << 32) | time->time_nsec_lo;
		tz.tz_minuteswest = time->tz_minuteswest;
		tz.tz_dsttime = time->tz_dsttime;
		do_settimeofday64(&tp);
		break;
	case SI_MCMD_SET_CARD_NUM:
		pep->card_num = *((u32 *)params);
		break;
	case SI_MCMD_SET_NETHWADDR:
		pep->net_dev.host_hwdata_p =
			((struct si_mcmd_set_nethw_addr_params *)params)->
			nethw_addr_high;
		pep->net_dev.host_hwdata_p =
			(pep->net_dev.host_hwdata_p << 32) |
			((struct si_mcmd_set_nethw_addr_params *)params)->
			nethw_addr_low;
		if (pep->net_dev.host_hwdata_p != 0) {
			si_pep_setup_neintf(pep);
		} else {
			si_pep_teardown_netintf(pep);
		}
		break;
	case SI_MCMD_SET_NETIPADDR:
		ret = si_pep_set_neintf_ifaddr(pep,
				(struct si_mcmd_set_ipaddr_params*)params);
		if (ret)
			res = SI_RES_FAILURE;
		break;
	default:
		si_err(pep, "Unsupported command 0x%04x", mcmd.cmd);
		res = SI_RES_FAILURE;
	}

mcmd_restart:
	mcmd_res.cmd = mcmd.cmd;
	mcmd_res.res = res;
	mcmd_res.errno = ret;
	mcmd_res.seq_id = mcmd.seq_id;
	mcmd_res.param_size = 0;
	pep->drv_ops->resp_mcmd(pep, &mcmd_res);
	si_minfo(pep, "Command 0x%04x, Response 0x%x", mcmd_res.cmd,
			mcmd_res.res);

	if (mcmd.cmd == SI_MCMD_REBOOT_SOC) {
		msleep(500);
		si_err(pep, "Rebooting SoC");
		kernel_restart("PCIe Host requested reboot");
	}
}

static void si_pep_process_timestamp(struct si_pep_dev *pep, u8 action,
				     struct si_pep_timestamps *ts)
{
	ktime_t *entry_time;
	ktime_t *exit_time;
	ktime_t *min_time;
	ktime_t *max_time;
	u32 ts_index;
	ktime_t time_diff;

	ts_index = ts->ts_index;

	switch (action) {
	case RECORD_ENTRY_TS:
		entry_time = &ts->start[ts_index];
		*entry_time = ktime_get();
		break;
	case RECORD_EXIT_TS:
		exit_time = &ts->stop[ts_index];
		*exit_time = ktime_get();
		break;
	case COMPUTE_TS_METRICS:
		entry_time = &ts->start[ts_index];
		exit_time = &ts->stop[ts_index];
		min_time = &ts->min_time;
		max_time = &ts->max_time;
		time_diff = *exit_time - *entry_time;
		if (*min_time == 0 && *max_time == 0) {
			*min_time = time_diff;
			*max_time = time_diff;
		} else {
			if (time_diff < *min_time)
				*min_time = time_diff;
			else
				*max_time = time_diff;
		}
		(ts_index >= 10) ? ts_index = 0 : ts_index++;
		ts->ts_index = ts_index;
		break;
	default:
		si_err(pep, "Invalid action code: %d\n", action);
	}
	return;
}

void si_pep_recycle_qes(struct si_pep_dev *pep, enum si_qtype qtype, int qid)
{
	int tail;
	int ptail;
	struct si_qctx *ctx;

	switch (qtype) {
	case SI_MRQ:
		ctx = pep->mrq_ctx;
		tail = mq_get_rq_tail(pep);
		break;
	case SI_MCQ:
		ctx = pep->mcq_ctx;
		tail = mq_get_cq_tail(pep);
		break;
	case SI_DRQ:
		ctx = pep->drq_ctx[qid];
		tail = dq_get_rq_tail(pep, qid);
		break;
	case SI_DCQ:
		ctx = pep->dcq_ctx[qid];
		tail = dq_get_cq_tail(pep, qid);
		break;
	default:
		return;
	}

	ptail = atomic_read(&ctx->tail);
	if (tail != ptail) {
		si_dbg_l(pep, "%s queue %d, setting tail to %d. Was %d",
				si_pep_qtype_str(qtype), qid, tail, ptail);
		atomic_set(&ctx->tail, tail);
	}
}

static int si_pep_cache_wqe(struct si_pep_dev *pep, enum si_qtype qtype,
		u16 qid, u16 sidx, u16 nents)
{
	int i;
	int ch;
	int ret;
	int cachable;
	int cached = 0;
	int spaces;
	size_t size;
	char *qstr = si_pep_qtype_str(qtype);
	dma_addr_t pcache;
	dma_addr_t src, dst;
	u32 *req_cached;
	wait_queue_head_t *req_wait;
	struct si_qctx *ctx;
	struct si_pep_req_cache_hdr *chdr;
	struct si_pep_qdev *qdev = NULL;
	struct mutex *m;
	ktime_t curr_time;

	si_dbg(pep, "%s queue %u, Cache %u entries starting %u", qstr, qid,
			nents, sidx);

	if (qtype == SI_MWQ) {
		ctx = pep->mwq_ctx;
		chdr = &pep->mchdr;
		qdev = &pep->qdev[0];
		m = &pep->mreq_cache_mutex;
		req_cached = &pep->mreq_cached;
		req_wait = &pep->mreq_wait;
		pcache =pep->p_mreq_cache; 
		ch = MWQ_CACHE_DMA_CH;
	} else if (qtype == SI_DWQ) {
		ctx = pep->dwq_ctx[qid];
		chdr = &pep->dchdr[qid];
		qdev = &pep->qdev[qid + 1];
		m = &pep->dreq_cache_mutex[qid];
		req_cached = &pep->dreq_cached[qid];
		req_wait = &pep->dreq_wait[qid];
		pcache = pep->p_dreq_caches[qid];
		ch = DWQ_CACHE_DMA_CH;
	} else {
		si_err(pep, "Invalid queue type '%s'", qstr);
		return 0;
	}

	mutex_lock(m);
	if (chdr->nreqs >= chdr->maxreqs) {
		si_err(pep, "%s queue request cache full", qstr);
		goto end;
 	}

	si_qdbg(qdev, "%u entries in cache. Max: %u", chdr->nreqs,
			chdr->maxreqs);
	if (chdr->nreqs + nents >= chdr->maxreqs)
		cachable = chdr->maxreqs - chdr->nreqs - 1;
	else
		cachable = nents;
	si_qdbg(qdev, "Cachable entries: %u", cachable);

	while (cachable > 0) {
		if (chdr->head <= chdr->tail)
			spaces = chdr->maxreqs - chdr->tail;
		else
			spaces = chdr->head - chdr->tail;

		src = (ctx->qe_pb) + (sizeof(struct si_wqe) * sidx);
		dst = pcache + (sizeof(struct si_wqe) * chdr->tail);
		nents = (cachable > spaces)?spaces:cachable;
		size = sizeof(struct si_wqe) * nents;
		ret = si_pep_copy_from_host(pep, dst, src, size, ch);
		if (ret) {
			si_err(pep, "Failed to copy wqes. %s queue %u, Start: "
				"%u, Entries %u", qstr, qid, sidx, nents);
			goto end;
		}

		if (qdev && dump_queue_occupency == true) {
			curr_time = ktime_get_ns();
			for (i = sidx; i < (sidx + nents); i++) {
				qdev->timepoint_1[i] = curr_time;
				qdev->timepoint_2[i] = 0 ;
			}
		}

		if (qdev)
			qdev->stats.rx_reqs += nents;

		cached += nents;
		chdr->tail = chdr->tail + nents;
		if (chdr->tail >= chdr->maxreqs)
			chdr->tail = 0;
		sidx += nents;
		cachable -= nents;
 	}
end:
	chdr->nreqs += cached;
	*req_cached = 1;
	mutex_unlock(m);
	wake_up_interruptible(req_wait);
	si_qdbg(qdev, "Caching done. Copied %d entries. Head %d, Tail %d",
			cached, chdr->head, chdr->tail);
	return cached;
}

void si_pep_get_reqs_from_host(struct si_pep_dev *pep, enum si_qtype t, int qid)
{
	int cached;
	int to_cache;
	u16 head;
	u16 tail;
	u16 wq_phead;
	u16 nwqs;
	atomic_t *wq_phead_ptr;
	struct si_qctx *wqctx;

	if (t == SI_MWQ) {
		wqctx = pep->mwq_ctx;
		wq_phead_ptr = &pep->mwq_prev_head;
		head = mq_get_wq_head(pep);
		tail = mq_get_wq_tail(pep);
		qid = 0;
	} else if (t == SI_DWQ) {
		wqctx = pep->dwq_ctx[qid];
		wq_phead_ptr = &pep->dwq_prev_heads[qid];
		head = dq_get_wq_head(pep, qid);
		tail = dq_get_wq_tail(pep, qid);
	} else
		return;

	wq_phead = atomic_read(wq_phead_ptr);
	if (head == 0 || head == tail || wq_phead == head)
		return;

	si_dinfo(pep, "%s queue %d, head %u, tail %u, prev head %u",
			si_pep_qtype_str(t), qid, head, tail, wq_phead);

	if (wq_phead > head) {
		nwqs = (wqctx->entries - wq_phead) + head;
		to_cache = (wqctx->entries - wq_phead);
		if (to_cache) {
			cached = si_pep_cache_wqe(pep, t, qid, wq_phead, to_cache);
			if (cached < to_cache) {
				atomic_set(wq_phead_ptr, wq_phead + cached);
				return;
			}
		}
		to_cache = head;
		cached = si_pep_cache_wqe(pep, t, qid, 0, to_cache);
		atomic_set(wq_phead_ptr, cached);
	} else {
		nwqs = head - wq_phead;
		to_cache = nwqs;
		cached = si_pep_cache_wqe(pep, t, qid, wq_phead, to_cache);
		atomic_set(wq_phead_ptr, wq_phead + cached);
	}

	return;
}

static void si_pep_link_down_handler(struct work_struct *lwork)
{
	struct si_pep_dev *pep;
	struct si_alert alert;
	
	u8 __iomem *stat7;
	u32 reg;

	pep = link_down_work_to_pep_dev(lwork);
	if (unlikely(!pep)) {
		pr_err("%s: Invalid end point device pointer\n", __func__);
		return;
	}

	/* Immediately reading stat7 causes synchronous external abort */
	si_err(pep, "PCIe Link toggled");
	stat7 = ((u8*)pep->glue_logic_base) + 0x74;
	do {
		reg = readl(stat7);
	} while ((reg & 0x02) != 0);

	/* Give link and host some time to settle down before we notify */
	msleep(250);
	pep->drv_ops->set_soc_state(pep, SOC_STATE_INIT);
	alert.type = SI_ALERT_LINK_DOWN;
	alert.sub_type = 0;
	strncpy(alert.data, "PCIe Link Toggled\n", sizeof(alert.data) - 1);
	si_pep_alert_host(pep, &alert);
}

static void si_pep_dbg_dump_bufid(struct si_pep_dev *pep,
		struct si_buffer_id *bid, char *prefix)
{
	char *btype;

	switch (bid->btype) {
	case SI_BUF_TYPE_INVALID:
	case SI_BUF_TYPE_MAX:
		btype = "Invalid/Max";
		break;
	case SI_BUF_TYPE_TX:
		btype = "Transmit";
		break;
	case SI_BUF_TYPE_RX:
		btype = "Receive";
		break;
	default:
		btype = "Illegal";
	}

	si_dbg_dump(pep, "%s: Queue ID: %u, Index: %u, Type: %s", prefix,
			bid->qid, bid->idx, btype);
}

static void si_pep_dbg_dump_sge(struct si_pep_dev *pep, struct si_sge *sge,
		char *prefix)
{
	char *type;
	char *flags;
	u64 addr = sge->addr_h;

	addr <<= 32;
	addr |= sge->addr_l;

	switch (sge->type) {
	case SGE_TYPE_NONE:
	case SGE_TYPE_MAX:
		type = "None/Max";
		break;
	case SGE_TYPE_SINGLE:
		type = "Single";
		break;
	case SGE_TYPE_LIST:
		type = "List";
		break;
	default:
		type = "Illegal";
	}

	switch (sge->flags) {
	case SGE_FLAGS_NONE:
	case SGE_FLAGS_MAX:
		flags = "None/Max";
		break;
	case SGE_FLAGS_FIRSTSGE:
		flags = "First SGE";
		break;
	case SGE_FLAGS_LASTSGE:
		flags = "Last SGE";
		break;
	default:
		flags = "Illegal";
	}

	si_dbg_dump(pep, "%s: Type: %s, Flags: %s, Address: 0x%llx, Size: %u",
			prefix, type, flags, addr, sge->size);
}

static void si_pep_dbg_dump_buf(struct si_pep_dev *pep, struct si_buffer *buf,
		char *prefix)
{
	char *addrtype;

	switch (buf->addrtype) {
	case SI_BUF_ADDR_INVALID:
	case SI_BUF_ADDR_MAX:
		addrtype = "Invalid/Max";
		break;
	case SI_BUF_ADDR_NONE:
		addrtype = "No Address";
		break;
	case SI_BUF_ADDR_USER:
		addrtype = "User virtual";
		break;
	case SI_BUF_ADDR_PHYSICAL:
		addrtype = "Physical";
		break;
	case SI_BUF_ADDR_KVIRTUAL:
		addrtype = "Kernel Virtual";
		break;
	case SI_BUF_ADDR_SGE:
		addrtype = "SGE";
		break;
	case SI_BUF_ADDR_SGE_LIST:
		addrtype = "SGE List";
		break;
	default:
		addrtype = "Illegal";
		break;
	}

	si_pep_dbg_dump_bufid(pep, &buf->bid, prefix);
	si_dbg_dump(pep, "%s: Address Type: %s, Address: 0x%px, Size: %lu, "
			"Total Size: %u, Entries: %u", prefix, addrtype,
			buf->addr, buf->size, buf->tsize, buf->nents);
}

static int si_pep_get_rqe(struct si_pep_dev *pep, enum si_qtype t, u32 qid,
		struct si_rqe **rqe)
{
	int c;
	u32 nqueues;
	u32 head;
	u32 tail;
	struct si_qctx *ctx;
	struct si_rqe *rqe_cache;

	if (si_pep_is_data_queue(t)) {
		ctx = pep->drq_ctx[qid];
		nqueues = SI_MAX_DQS;
		rqe_cache = pep->drqe_caches[qid];
	} else if (si_pep_is_mgmt_queue(t)) {
		ctx = pep->mrq_ctx;
		nqueues = 1;
		rqe_cache = pep->mrqe_cache;
	} else {
		si_err(pep, "Invalid queue type %u, queue %u", t, qid);
		return -EINVAL;
	}

	if (qid >= nqueues) {
		si_err(pep, "%s Queue %u out of range. Have %u",
				si_pep_qtype_str(t), qid, nqueues);
		return -EINVAL;
	}

	if (ctx->status == SI_INVALID) {
		si_err(pep, "No context created for %s queue %d",
		       si_pep_qtype_str(t), qid);
		return -ENOENT;
	}

	tail = atomic_read(&ctx->tail);
	head = atomic_read(&ctx->head);
	c = CIRC_SPACE(head, tail, ctx->entries);
	if (c <= 1) {
		si_dbg(pep, "No free rq entries in %u. Head %u, "
			"Tail %u, Entries %u", qid, head, tail, ctx->entries);
		return -EAGAIN;
	}

	*rqe = &rqe_cache[head];
	(*rqe)->qidx.qid = qid;
	(*rqe)->qidx.idx = head;
	if (si_pep_is_data_queue(t))
		(*rqe)->qidx.qtype = SI_DRQ;
	else
		(*rqe)->qidx.qtype = SI_MRQ;

	si_dbg(pep, "%s queue %u, Tail %u, Head: %u",
		si_pep_qtype_str((*rqe)->qidx.qtype), qid, tail, head);
	head++;
	if (head >= ctx->entries)
		head = 0;
	atomic_set(&ctx->head, head);

	return 0;
}

static int si_pep_try_get_rqe(struct si_pep_dev *pep, enum si_qtype t, u32 qid,
		struct si_rqe **rqe)
{
	int ret;
	int retries = 20000;

	do {
		ret = si_pep_get_rqe(pep, t, qid, rqe);
		if (ret == -EAGAIN) {
			usleep_range(50, 60);
			retries--;
		} else
			break;
	} while (retries >= 0);

	if (ret) {
		si_err(pep, "Queue %u, Ran out of output buffers", qid);
		return ret;
	}

	return 0;
}

static int si_pep_set_rqe(struct si_pep_dev *pep, struct si_rqe *rqe)
{
	int ret;
	int ch;
	u32 qid;
	u16 index;
	dma_addr_t dest;
	dma_addr_t src;
	enum si_qtype t = rqe->qidx.qtype;
	struct si_qctx *ctx;

	qid = rqe->qidx.qid;
	index = rqe->qidx.idx;
	if (si_pep_is_data_queue(t)) {
		ctx = pep->drq_ctx[qid];
		src = pep->p_drqe_caches[qid] + (index * sizeof(*rqe));
		ch = DQ_DMA_CH(qid);
	} else if (si_pep_is_mgmt_queue(t)) {
		ctx = pep->mrq_ctx;
		src = pep->p_mrqe_cache + (index * sizeof(*rqe));
		ch = MQ_DMA_CH;
	} else {
		si_err(pep, "Invalid queue type %u, queue %u", t, qid);
		return -EINVAL;
	}

	dest = ctx->qe_pb + (index * sizeof(struct si_rqe));
	ret = si_pep_copy_to_host(pep, dest, src, sizeof(*rqe), ch, -1);
	if (ret) {
		si_err(pep, "%s queue %u, Index %u, Failed to update on host",
				si_pep_qtype_str(t), qid, index);
		return ret;
	}

	if (si_pep_is_data_queue(t))
		dq_set_rq_head(pep, qid, index + 1);
	else
		mq_set_rq_head(pep, index + 1);

	return 0;
}

static int si_pep_get_cqe(struct si_pep_dev *pep, enum si_qtype qtype,
		u32 qid, struct si_cqe *cqe)
{
	int c;
	u32 nqueues;
	u32 head;
	u32 tail;
	struct si_qctx *ctx;
	char *qstr;

	if (si_pep_is_data_queue(qtype)) {
		ctx = pep->dcq_ctx[qid];
		nqueues = SI_MAX_DQS;
		qstr = "Data";
	} else if (si_pep_is_mgmt_queue(qtype)) {
		ctx = pep->mcq_ctx;
		nqueues = 1;
		qstr = "Management";
	} else {
		si_err(pep, "Invalid queue type %u, queue %u", qtype, qid);
		return -EINVAL;
	}

	if (qid >= nqueues) {
		si_err(pep, "Queue %u out of range. Have %u", qid, nqueues);
		return -EINVAL;
	}

	if (ctx->status == SI_INVALID) {
		si_err(pep, "No context created for %s cqe %d",
		       si_pep_qtype_str(qtype), qid);
		return -ENOENT;
	}

	tail = atomic_read(&ctx->tail);
	head = atomic_read(&ctx->head);
	c = CIRC_SPACE(head, tail, ctx->entries);
	if (c <= 1) {
		si_dinfo(pep, "%s queue %u, No free entries. Head %u, Tail %u",
				qstr, qid, head, tail);
		return -EAGAIN;
	}

	cqe->qidx.qid = qid;
	cqe->qidx.idx = head;
	if (si_pep_is_data_queue(qtype))
		cqe->qidx.qtype = SI_DCQ;
	else
		cqe->qidx.qtype = SI_MCQ;
	cqe->qidx.inuse = true;

	si_dinfo(pep, "%s queue %u, Tail %u, Head: %u",
		si_pep_qtype_str(cqe->qidx.qtype), qid, tail, head);

	head++;
	if (head >= ctx->entries)
		head = 0;
	atomic_set(&ctx->head, head);

	return 0;
}

static int si_pep_set_cqe(struct si_pep_dev *pep, struct si_cqe *cqe)
{
	int ret;
	int msi;
	int ch;
	u32 qid;
	u16 index;
	dma_addr_t dest;
	phys_addr_t src;
	enum si_qtype t;
	struct si_cqe *lcqe;
	struct si_qctx *ctx;

	qid = cqe->qidx.qid;
	index = cqe->qidx.idx;
	t = cqe->qidx.qtype;
	if (si_pep_is_data_queue(t)) {
		ctx = pep->dcq_ctx[qid];
		lcqe = si_pep_cache_virt(pep, dcqe_tmps[qid]);
		src = si_pep_cache_phys(pep, dcqe_tmps[qid]);
		msi = SI_DCQ_MSI_START + qid;
		ch = DQ_DMA_CH(qid);
	} else if (si_pep_is_mgmt_queue(t)) {
		ctx = pep->mcq_ctx;
		lcqe = si_pep_cache_virt(pep, mcqe_tmp);
		src = si_pep_cache_phys(pep, mcqe_tmp);
		msi = SI_MCQ_MSI;
		ch = MQ_DMA_CH;
	} else {
		return -EINVAL;
	}

	si_dbg_l(pep, "%s queue %u[%u], channel: %u, msi: %u ,Sequence ID: %u, "
			"Cmd: %u, Response: 0x%x", si_pep_qtype_str(t), qid,
			index, ch, msi, cqe->seqid, cqe->cmd, cqe->resp);

	si_pep_iomem_write_rep(lcqe, cqe, sizeof(*cqe));
	dest = ctx->qe_pb + (index * sizeof(struct si_cqe));

	if (si_pep_is_data_queue(t))
		dq_set_cq_head(pep, qid, index + 1);
	else
		mq_set_cq_head(pep, index + 1);

	ret = si_pep_copy_to_host(pep, dest, src, sizeof(*cqe), ch, msi);
	if (ret) {
		si_err(pep, "Failed to update CQE on host. %s queue %u[%u], "
			"Sequence ID: %u, Response: 0x%x", si_pep_qtype_str(t),
			qid, index, cqe->seqid, cqe->resp);
		return ret;
	}

	si_dinfo(pep, "%s queue %u[%u], Sequence ID: %u, Response: 0x%x",
			si_pep_qtype_str(t), qid, index, cqe->seqid, cqe->resp);
	return 0;
}

static int si_pep_get_buff_arg(struct si_pep_dev *pep, struct si_buffer *buf,
		void **addrs, size_t **sizes)
{
	struct device *dev = pep->dev;
	int n = buf->nents;
	int ret;

	if (n <= 1 || n > SI_NUM_BUFS_MAX) {
		si_err(pep, "Invalid number of buffers, %u", n);
		return -E2BIG;
	}

	*addrs = devm_kmalloc_array(dev, n, sizeof(void __user*), GFP_KERNEL);
	if (*addrs == NULL) {
		si_err(pep, "Failed to allocate argument list");
		return -ENOMEM;
	}

	*sizes = devm_kmalloc_array(dev, n, sizeof(size_t), GFP_KERNEL);
	if (*sizes == NULL) {
		si_err(pep, "Failed to allocate argument list for sizes");
		ret = -ENOMEM;
		goto error_salloc;
	}

	if (copy_from_user(*addrs, buf->addrs, n * sizeof(void __user *))) {
		ret = -EFAULT;
		goto error_copy_from;
	}

	if (copy_from_user(*sizes, buf->sizes, n * sizeof(size_t))) {
		ret = -EFAULT;
		goto error_copy_from;
	}

	return 0;

error_copy_from:
	devm_kfree(dev, sizes);
error_salloc:
	devm_kfree(dev, addrs);

	return ret;
}

static void si_pep_free_dma_ll(struct si_pep_dev *pep,
		struct si_pep_dma_ll_ctx *ll)
{
	if (ll == NULL)
		return;

	if (ll->dma_ll == NULL)
		return;

	dma_free_coherent(pep->dev, ll->size, ll->dma_ll, ll->p_dma_ll);
	memset(ll, 0, sizeof(*ll));
}
	
static void si_pep_unmap_user_ptr_single(struct si_pep_dev *pep,
		struct si_pep_userbuf_map *ubmap)
{
	if (ubmap->nr_pages == 0)
		return;

	if (ubmap->nr_sgents > 0) {
		dma_unmap_sg(pep->dev, ubmap->sgt.sgl, ubmap->sgt.nents,
				DMA_BIDIRECTIONAL);
		ubmap->nr_sgents = 0;
	}

	sg_free_table(&ubmap->sgt);
	unpin_user_pages(ubmap->pages, ubmap->nr_pages);
	devm_kfree(pep->dev, ubmap->pages);
	ubmap->nr_pages = 0;
	ubmap->pages = NULL;
}

static void si_pep_unmap_user_ptrs(struct si_pep_dev *pep,
		struct si_pep_userbuf_ctx **bufctx)
{
	int idx;

	if (*bufctx == NULL)
		return;

	if ((*bufctx)->nr_bufs == 0)
		return;

	for (idx = 0; idx < (*bufctx)->nr_bufs; idx++)
		si_pep_unmap_user_ptr_single(pep, &(*bufctx)->userbuf_maps[idx]);

	devm_kfree(pep->dev, *bufctx);
	*bufctx = NULL;
}

static int si_pep_map_user_ptr_single(struct si_pep_dev *pep, void __user *ptr,
		size_t size, struct si_pep_userbuf_map *ubmap)
{
	int ret;
	ulong start = (ulong)ptr;
	ulong offset = offset_in_page(start);
	uint nr_pages;

	ubmap->total_size = size;
	nr_pages = DIV_ROUND_UP(offset + size, PAGE_SIZE);
	ubmap->pages = devm_kmalloc_array(pep->dev, nr_pages,
			sizeof(struct page *), GFP_KERNEL);
	if (ubmap->pages == NULL)
		return -ENOMEM;
	ret = pin_user_pages_fast(start & PAGE_MASK, nr_pages,
			FOLL_WRITE | FOLL_LONGTERM, ubmap->pages);
	if (ret < 0) {
		devm_kfree(pep->dev, ubmap->pages);
		ubmap->nr_pages = 0;
		return ret;
	}

	if (ret != nr_pages) {
		ubmap->nr_pages = ret;
		ret = -EFAULT;
		goto error_pin;
	}

	ubmap->nr_pages = nr_pages;
	ret = sg_alloc_table_from_pages(&ubmap->sgt, ubmap->pages, nr_pages,
			offset, size, GFP_KERNEL);
	if (ret)
		goto error_pin;
	ubmap->nr_sgents = dma_map_sg(pep->dev, ubmap->sgt.sgl, ubmap->sgt.nents,
			DMA_BIDIRECTIONAL);
	if (ubmap->nr_sgents == 0) {
		ret = -EIO;
		goto error_sg_alloc;
	}

	return 0;

error_sg_alloc:
	sg_free_table(&ubmap->sgt);
error_pin:
	unpin_user_pages(ubmap->pages, ubmap->nr_pages);
	devm_kfree(pep->dev, ubmap->pages);

	return ret;
}

static int si_pep_map_user_ptr(struct si_pep_dev *pep, struct si_buffer *buf,
		struct si_pep_userbuf_ctx **bufctx)
{
	int ret;
	int idx;
	int nents = buf->nents;
	u8 *ptrs = NULL;
	size_t *sizes = NULL;
	struct si_pep_userbuf_map *userbuf_maps;
	struct device *dev = pep->dev;

	if (buf->addrtype != SI_BUF_ADDR_USER)
		return -ENOTSUPP;

	if (nents > 1) {
		ret = si_pep_get_buff_arg(pep, buf, (void**)&ptrs, &sizes);
		if (ret)
			return ret;
	}

	userbuf_maps = devm_kcalloc(dev, nents,
			sizeof(struct si_pep_userbuf_map), GFP_KERNEL);
	if (userbuf_maps == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	*bufctx = devm_kzalloc(dev, sizeof(struct si_pep_userbuf_ctx),
			GFP_KERNEL);
	if (*bufctx == NULL) {
		ret = -ENOMEM;
		goto error_alloc_ctx;
	}

	(*bufctx)->userbuf_maps = userbuf_maps;
	(*bufctx)->nr_bufs = nents;

	if (nents == 1) {
		ret = si_pep_map_user_ptr_single(pep, buf->addr, buf->size,
				&userbuf_maps[idx]);
		if (ret)
			goto error_map_single;

		return 0;
	}

	for (idx = 0; idx < buf->nents; idx++) {
		ret = si_pep_map_user_ptr_single(pep, (void *)&ptrs[idx],
				sizes[idx], &userbuf_maps[idx]);
		if (ret)
			goto error_map_single;
	}

	ret = 0;
	goto done;

error_map_single:
	while (idx > 0) {
		idx--;
		si_pep_unmap_user_ptr_single(pep, &userbuf_maps[idx]);
	}
error_alloc_ctx:
	devm_kfree(dev, userbuf_maps);
done:
	if (sizes)
		devm_kfree(dev, sizes);
	if (ptrs)
		devm_kfree(dev, ptrs);

	return ret;
}

static int si_pep_prepare_dma_ll(struct si_pep_dev *pep,
		struct si_pep_dmabuf *src, int nsrc,
		struct si_pep_dmabuf *dst, int ndst,
		struct si_pep_dma_ll_ctx *ll, enum si_dma_dir dir)
{
	struct device *dev = pep->dev;
	struct si_pep_dma_ll_lelem *lelem;
	struct si_pep_dma_ll_delem *delem;
	dma_addr_t saddr, daddr;
	int sbi, sbo, dbi, dbo, lli;
	size_t stsz, dtsz, csz, srsz, drsz;
	size_t n_ll_ents = 0;

	stsz = 0;
	for (sbi = 0; sbi < nsrc; sbi++)
		stsz += src[sbi].size;

	dtsz = 0;
	for (dbi = 0; dbi < ndst; dbi++)
		dtsz += dst[dbi].size;

	ll->dir = dir;
	ll->size = (n_ll_ents + 1) * sizeof(struct si_pep_dma_ll_lelem);
	ll->dma_ll = dma_alloc_coherent(dev, ll->size, &ll->p_dma_ll,
			GFP_KERNEL);
	if (ll->dma_ll == NULL)
		return PTR_ERR(ll->dma_ll);
	ll->nents = n_ll_ents;

	sbi = 0;
	sbo = 0;
	dbi = 0;
	dbo = 0;
	lli = 0;
	while (sbi < nsrc && dbi < ndst) {
		srsz = src[sbi].size - sbo;
		drsz = dst[dbi].size - dbo;

		if (srsz == 0) {
			sbi++;
			sbo = 0;
			continue;
		}

		if (drsz == 0) {
			dbi++;
			dbo = 0;
			continue;
		}

		saddr = src[sbi].addr + sbo;
		daddr = dst[dbi].addr + dbo;
		delem = &ll->dma_ll[lli++];
		csz = (srsz < drsz)?srsz:drsz;
		delem->dar_low = lower_32_bits(daddr);
		delem->dar_high = upper_32_bits(daddr);
		delem->sar_low = lower_32_bits(saddr);
		delem->sar_high = upper_32_bits(saddr);
		delem->xfer_size = csz;
		delem->flags = CYCLE_BIT;
		sbo += csz;
		dbo += csz;
		ll->tsize += csz;

		if (sbo >= src[sbi].size) {
			sbi++;
			sbo = 0;
		}

		if (dbo >= dst[dbi].size) {
			dbi++;
			dbo = 0;
		}
	}

	delem->flags |= LWIE_BIT;
	lelem = (struct si_pep_dma_ll_lelem *)&ll->dma_ll[lli];
	memset(lelem, 0, sizeof(*lelem));
	lelem->flags = LLP_ELEMENT | TCB_BIT;
	lelem->ll_ptr_low = lower_32_bits(ll->p_dma_ll);
	lelem->ll_ptr_high = upper_32_bits(ll->p_dma_ll);
	si_dbg_l(pep, "Linked list prepeated. Total copy: %lu bytes", ll->tsize);

	return 0;
}

static int si_pep_dma_buflist_from_sge(struct si_pep_dev *pep,
		struct si_sge *sge, int ch, struct si_pep_dmabuf **db,
		int *nents, size_t *tsize)
{
	struct device *dev = pep->dev;
	struct si_sge *rsges;
	dma_addr_t p_rsges;
	dma_addr_t raddr;
	size_t ssz;
	int i;
	int n;

	if (sge->type == SGE_TYPE_SINGLE)
		n = 1;
	else if (sge->type == SGE_TYPE_LIST)
		n = sge->size;
	else
		return -EINVAL;

	*db = devm_kzalloc(dev, n * sizeof(struct si_pep_dmabuf), GFP_KERNEL);
	if (*db == NULL)
		return -ENOMEM;

	if (sge->type == SGE_TYPE_SINGLE) {
		(*db)->addr = (dma_addr_t)sge->addr_h << 32 | sge->addr_l;
		(*db)->size = sge->size;
		*nents = 1;
		*tsize = sge->size;
		return 0;
	}

	ssz = n * sizeof(struct si_sge);
	rsges = dma_alloc_coherent(pep->dev, ssz, &p_rsges, GFP_KERNEL);
	if (rsges == NULL) {
		si_err(pep, "Failed to allocate mem for remote sges");
		devm_kfree(dev, *db);
		return -ENOMEM;
	}

	raddr = sge->addr_l | ((dma_addr_t)sge->addr_h << 32);
	if (si_pep_copy_from_host(pep, p_rsges, raddr, ssz, ch)) {
		si_err(pep, "Failed to copy sge list from host");
		devm_kfree(dev, *db);
		dma_free_coherent(dev, ssz, rsges, p_rsges);
		return -EIO;
	}

	*tsize = 0;
	for (i = 0; i < n; i++) {
		(*db)[i].addr = ((dma_addr_t)rsges[i].addr_h << 32) |
			rsges[i].addr_l;
		(*db)[i].size = rsges[i].size;
		(*tsize) += rsges[i].size;
	}

	*nents = n;
	return 0;
}

static int si_pep_dma_buflist_from_physptr(struct si_pep_dev *pep,
		struct si_buffer *buf, struct si_pep_dmabuf **db, int *nents,
		size_t *tsize)
{
	struct device *dev = pep->dev;
	void *addrs;
	size_t *sizes;
	int ret;
	int i;

	if (buf->addrtype != SI_BUF_ADDR_PHYSICAL)
		return -EINVAL;

	*db = devm_kzalloc(dev, buf->nents * sizeof(struct si_pep_dmabuf),
			GFP_KERNEL);
	if (*db == NULL)
		return -ENOMEM;

	if (buf->nents == 1) {
		(*db)->addr = (dma_addr_t)buf->addr;
		(*db)->size = buf->size;
		*tsize = buf->size;
		*nents = 1;
		return 0;
	}

	ret = si_pep_get_buff_arg(pep, buf, &addrs, &sizes);
	if (ret) {
		devm_kfree(dev, *db);
		return ret;
	}

	*tsize = 0;
	for (i = 0; i < buf->nents; i++) {
		(*db)[i].addr = ((dma_addr_t*)addrs)[i];
		(*db)[i].size = sizes[i];
		(*tsize) += sizes[i];
	}

	devm_kfree(dev, addrs);
	devm_kfree(dev, sizes);

	return 0;
}

static int si_pep_dma_buflist_from_uptr(struct si_pep_dev *pep, 
		struct si_pep_userbuf_ctx *bufctx, struct si_pep_dmabuf **db,
		int *nents, size_t *tsize)
{
	struct si_pep_userbuf_map *ubmap = bufctx->userbuf_maps;
	struct device *dev = pep->dev;
	struct sg_table *sgt;
	struct scatterlist *sl;
	int n = 0;
	int i, j, k = 0;

	for (i = 0; i < bufctx->nr_bufs; i++) {
		n += ubmap[i].nr_sgents;
	}

	*db = devm_kzalloc(dev, sizeof(struct si_pep_dmabuf) * n, GFP_KERNEL);
	if (*db == NULL)
		return -ENOMEM;

	*tsize = 0;
	for (i = 0; i < bufctx->nr_bufs; i++) {
		sgt = &ubmap[i].sgt;
		for_each_sg (sgt->sgl, sl, sgt->nents, j) {
			(*db)[k].addr = sg_dma_address(sl);
			(*db)[k].size = sg_dma_len(sl);
			(*tsize) += sg_dma_len(sl);
			k++;
		}
	}
	*nents = n;

	return 0;
}

static int si_pep_buf_prepare_dmabuf_list(struct si_pep_qdev *qdev,
		struct si_buffer *buf, struct si_pep_dmabuf **db, int *nents,
		size_t *tsize, struct si_pep_userbuf_ctx **bufctx)
{
	struct si_pep_dev *pep = qdev->pep;
	struct si_sge sge;
	int ch = Q_DMA_CH(qdev);
	int ret;

	*tsize = 0;
	if (buf->addrtype == SI_BUF_ADDR_USER) {
		ret = si_pep_map_user_ptr(pep, buf, bufctx);
		if (ret) {
			si_err(pep, "Failed to map user pointer");
			return ret;
		}

		ret = si_pep_dma_buflist_from_uptr(pep, *bufctx, db, nents,
				tsize);
		if (ret) {
			si_err(pep, "Failed to prepare source list");
			si_pep_unmap_user_ptrs(pep, bufctx);
			return ret;
		}
		return 0;
	} else if (buf->addrtype == SI_BUF_ADDR_PHYSICAL) {
		return si_pep_dma_buflist_from_physptr(pep, buf, db, nents,
				tsize);
	} else if (buf->addrtype == SI_BUF_ADDR_SGE) {
		sge.type = SGE_TYPE_SINGLE;
		sge.size = buf->size;
	} else if (buf->addrtype == SI_BUF_ADDR_SGE_LIST) {
		sge.type = SGE_TYPE_LIST;
		sge.size = buf->nents;
	} else
		return -EINVAL;

	sge.addr_h = upper_32_bits((u64)buf->addr);
	sge.addr_l = lower_32_bits((u64)buf->addr);
	return si_pep_dma_buflist_from_sge(pep, &sge, ch, db, nents, tsize);
}

static int si_pep_send_data_single(struct si_pep_qdev *qdev,
		struct si_buffer *buf, struct si_sge *sge)
{
	struct si_pep_dev *pep = qdev->pep;
	dma_addr_t src;
	dma_addr_t dst;
	size_t size;
	int ch = Q_DMA_CH(qdev);

	if (buf->nents != 1 || (buf->addrtype != SI_BUF_ADDR_SGE &&
			buf->addrtype != SI_BUF_ADDR_PHYSICAL)) {
		si_err(pep, "Invalid buffer type for transfer");
		return -EINVAL;
	}

	if (sge->type != SGE_TYPE_SINGLE) {
		si_err(pep, "Invalid sge type for transfer");
		return -EINVAL;
	}

	if (sge->size < buf->tsize) {
		si_err(pep, "Truncating source buffer. Not enough space");
		size = sge->size;
	} else if (sge->size >= buf->tsize)
		size = buf->tsize;

	if (size == 0) {
		si_err(pep, "Invalid send buffer size");
		return -EINVAL;
	}

	src = (dma_addr_t)buf->addr;
	dst = ((dma_addr_t)sge->addr_h << 32) | sge->addr_l;
	if (si_pep_copy_to_host(pep, dst, src, size, ch, -1)) {
		si_err(pep, "Send to host failed");
		return -EIO;
	}

	buf->tsize = size;

	return 0;
}

static int si_pep_send_data(struct si_pep_qdev *qdev, struct si_buffer *buf,
		struct si_qe_id *rqidx)
{
	struct si_rqe *rqe;
	struct si_sge *sge;
	struct si_pep_dmabuf *destb = NULL;
	struct si_pep_dmabuf *srcb = NULL;
	struct si_pep_dma_ll_ctx ll = { 0 };
	struct si_pep_userbuf_ctx *bufctx = NULL;
	struct si_pep_dev *pep = qdev->pep;
	struct device *dev = pep->dev;
	enum si_dma_dir dir = DIR_WRITE;
	size_t tsize;
	int ndest, nsrc;
	int ch = Q_DMA_CH(qdev);
	int llmode = 0;
	int ret;

	ret = si_pep_try_get_rqe(pep, qdev->qtype, qdev->qid, &rqe);
	if (ret)
		return ret;
	
	memcpy(rqidx, &rqe->qidx, sizeof(struct si_qe_id));
	sge = &rqe->sge;
	if (sge->type == SGE_TYPE_LIST || buf->nents > 1 ||
			buf->addrtype == SI_BUF_ADDR_USER ||
			buf->addrtype == SI_BUF_ADDR_SGE_LIST) {
		llmode = 1;
	}

	if (llmode) {
		ret = si_pep_dma_buflist_from_sge(pep, sge, ch, &destb, &ndest,
				&tsize);
		if (ret) {
			si_err(pep, "Unable to prepare destination list");
			return ret;
		}

		ret = si_pep_buf_prepare_dmabuf_list(qdev, buf, &srcb, &nsrc,
				&tsize, &bufctx);
		if (ret) {
			si_err(pep, "Unable to prepare source buffer list");
			goto err_clean_send_data;
		}

		ret = si_pep_prepare_dma_ll(pep, srcb, nsrc, destb, ndest, &ll,
				dir);
		if (ret) {
			si_err(pep, "Unable to prepare source buffer list");
			goto err_clean_send_data;
		}

		ret = si_pep_copy_do_ll(pep, &ll, ch, -1);
		if (!ret)
			buf->tsize = ll.tsize;
	} else
		ret = si_pep_send_data_single(qdev, buf, sge);

	if (ret) {
		rqe->data_size = 0;
		si_err(pep, "Failed to send data");
	} else
		rqe->data_size = buf->tsize;
	
	ret = si_pep_set_rqe(pep, rqe);

err_clean_send_data:
	if (destb)
		devm_kfree(dev, destb);
	if (srcb)
		devm_kfree(dev, srcb);
	if (ll.dma_ll)
		si_pep_free_dma_ll(pep, &ll);
	if (bufctx)
		si_pep_unmap_user_ptrs(pep, &bufctx);

	return ret;
}

static int si_pep_send_resp(struct si_pep_qdev *qdev, struct si_io *io,
		struct si_qe_id *rqidx)
{
	struct si_pep_dev *pep = qdev_to_pep_dev(qdev);
	struct si_qctx *qctx;
	struct si_cqe cqe = { 0 };
	u32 index;
	int qid;
	int ret;

	qid = qdev->qid;
	if (queue_is_data(qdev))
		qctx = pep->dwq_ctx[qid];
	else
		qctx = pep->mwq_ctx;

	ret = si_pep_get_cqe(pep, qctx->qtype, qctx->qid, &cqe);
	if (ret)
		return ret;
	
	index = cqe.qidx.idx;
	if (rqidx)
		memcpy(&cqe.rqidx, rqidx, sizeof(*rqidx));
	else
		cqe.rqidx = (struct si_qe_id){ 0 };
	cqe.cmd = io->cmd;
	cqe.resp = io->resp;
	cqe.seqid = atomic_inc_return(&qdev->seqid);
	if (cqe.seqid == 0)
		cqe.seqid = atomic_inc_return(&qdev->seqid);
	memcpy(&cqe.params, &io->desc, sizeof(cqe.params));

	si_qdbg_l(qdev, "index %u, Response: 0x%x, Seq: %u", index, cqe.resp,
			cqe.seqid);

	ret = si_pep_set_cqe(pep, &cqe);
	if (ret)
		return ret;

	qdev->stats.tx_resps++;
	qdev->stats.last_resp = ktime_get_ns();

	return 0;
}

static int si_pep_send_io(struct si_pep_qdev *qdev, struct si_io *io)
{
	struct si_buffer *buff;
	struct si_qctx *qctx;
	struct si_pep_dev *pep = qdev_to_pep_dev(qdev);
	struct si_qe_id rqidx = { 0 };
	int ret;
	int qid;

	qid = qdev->qid;
	if (queue_is_data(qdev))
		qctx = pep->dwq_ctx[qid];
	else
		qctx = pep->mwq_ctx;

	if (qctx->status == SI_FLUSHING)
		return -EAGAIN;

	buff = &io->buffer;
	if (buff->tsize != 0) {
		ret = si_pep_send_data(qdev, buff, &rqidx);
		if (ret) {
			io->resp = SI_RES_FAILURE;
			io->desc.resp = SI_RES_FAILURE;
			io->desc.err = -EIO;
			memset(&io->buffer, 0, sizeof(struct si_buffer));
		}
	}

	return si_pep_send_resp(qdev, io, &rqidx);
}

static int si_pep_recv_data_single(struct si_pep_qdev *qdev,
		struct si_buffer *buf, struct si_sge *sge)
{
	struct si_pep_dev *pep = qdev->pep;
	dma_addr_t src;
	dma_addr_t dst;
	size_t size;
	int ch = Q_DMA_CH(qdev);

	if (buf->nents != 1 && buf->addrtype != SI_BUF_ADDR_PHYSICAL) {
		si_err(pep, "Invalid buffer type for transfer");
		return -EINVAL;
	}

	if (sge->type != SGE_TYPE_SINGLE) {
		si_err(pep, "Invalid sge type for transfer");
		buf->tsize = 0;
		return -EINVAL;
	}

	if (sge->size > buf->tsize) {
		si_err(pep, "Truncating source buffer. Not enough space");
		size = buf->tsize;
	} else if (sge->size <= buf->tsize)
		size = sge->size;

	if (size == 0) {
		si_err(pep, "Invalid send buffer size");
		buf->tsize = 0;
		return -EINVAL;
	}

	dst = (dma_addr_t)buf->addr;
	src = ((dma_addr_t)sge->addr_h << 32) | sge->addr_l;
	if (si_pep_copy_from_host(pep, dst, src, size, ch)) {
		si_err(pep, "Receive from host failed");
		buf->tsize = 0;
		return -EIO;
	}

	return 0;
}

/*
 * Receive data from host. The source buffer should always be host buffer, with
 * address type of either SGE or SGE_LIST. The destination buffer can be either
 * USER or PHYSICAL. There is no current use-case that would require an SoC
 * buffer to be of SGE type. If needed, add buffer allocation support in driver
 * or start supporting standard sg_table for better compatibility
 */
static int si_pep_recv_data(struct si_pep_qdev *qdev,
		struct si_data_param *dparam)
{
	struct si_sge sge;
	struct si_buffer *src;
	struct si_buffer *dst;
	struct si_pep_dmabuf *dstb = NULL;
	struct si_pep_dmabuf *srcb = NULL;
	struct si_pep_dma_ll_ctx ll = { 0 };
	struct si_pep_userbuf_ctx *bufctx = NULL;
	struct si_pep_dev *pep = qdev->pep;
	size_t tsize;
	int nsrc, ndst;
	int llmode = 0;
	int ret;
	int ch = Q_DMA_CH(qdev);

	src = &dparam->src;
	dst = &dparam->dst;

	if (src->tsize > dst->tsize) {
		si_err(pep, "Not enough space for %u bytes. Have %u. Truncate",
				src->tsize, dst->tsize);
		src->tsize = dst->tsize;
		src->size = dst->tsize;
	} else {
		dst->tsize = src->size;
	}

	if (src->addrtype == SI_BUF_ADDR_SGE) {
		sge.type = SGE_TYPE_SINGLE;
		sge.size = src->size;
		si_dbg_l(pep, "Single SGE, Source: 0x%llx, Size: 0x%lu",
				(dma_addr_t)src->addr, src->size);
	} else if (src->addrtype == SI_BUF_ADDR_SGE_LIST) {
		sge.type = SGE_TYPE_LIST;
		sge.size = src->nents;
		llmode = 1;
		si_dbg_l(pep, "SGE List, Source: 0x%llx, Size: 0x%lu",
				(dma_addr_t)src->addr, src->size);
	} else {
		si_err(pep, "Invalid source buffer type");
		return -EINVAL;
	}

	sge.addr_h = upper_32_bits((dma_addr_t)src->addr);
	sge.addr_l = lower_32_bits((dma_addr_t)src->addr);

	if (dst->addrtype == SI_BUF_ADDR_USER)
		llmode = 1;
	else if (dst->addrtype == SI_BUF_ADDR_PHYSICAL) {
		if (dst->nents > 1)
			llmode = 1;
	} else {
		si_err(pep, "Invalid destination buffer type");
		return -EINVAL;
	}

	if (llmode) {
		ret = si_pep_dma_buflist_from_sge(pep, &sge, ch, &srcb, &nsrc,
				&tsize);
		if (ret) {
			si_err(pep, "Unable to prepare source list");
			return ret;
		}

		ret = si_pep_buf_prepare_dmabuf_list(qdev, dst, &dstb, &ndst,
				&tsize, &bufctx);
		if (ret) {
			si_err(pep, "Unable to prepare destination list");
			goto err_clean_recv_data;
		}

		ret = si_pep_prepare_dma_ll(pep, srcb, nsrc, dstb, ndst, &ll,
				DIR_READ);
		if (ret) {
			si_err(pep, "Unable to prepare source buffer list");
			goto err_clean_recv_data;
		}

		ret = si_pep_copy_do_ll(pep, &ll, ch, -1);
		if (ret)
			dst->tsize = 0;
	} else
		ret = si_pep_recv_data_single(qdev, dst, &sge);

err_clean_recv_data:
	if (dstb)
		devm_kfree(pep->dev, dstb);
	if (srcb)
		devm_kfree(pep->dev, srcb);
	if (ll.dma_ll)
		si_pep_free_dma_ll(pep, &ll);
	if (bufctx)
		si_pep_unmap_user_ptrs(pep, &bufctx);

	return ret;
}

/* File Operations */
static int si_pep_open(struct inode *inode, struct file *filp)
{
	struct si_pep_qdev *qdev;
	int minor = iminor(inode);

	if (minor > (SI_MAX_DQS + 1)) {
		si_err(g_pep_dev, "Invalid device file");
		return -ENOENT;
	}

	qdev = &g_pep_dev->qdev[minor];
	si_qdbg_l(qdev, "Opening device %s", qdev->devname);
	if (queue_is_data(qdev)) {
		if (use_driver_loopback) {
			return -ENOENT;
		}
	}

	filp->private_data = (void*)qdev;

	return 0;
}

static int si_pep_release(struct inode *inode, struct file *filp)
{
	struct si_pep_qdev *qdev = filp->private_data;

	if (qdev == NULL) {
		si_err(g_pep_dev, "Invalid driver private data");
		return -EINVAL;
	}

	if (queue_is_data(qdev)) {
		if (use_driver_loopback) {
			return -ENOENT;
		}
	}

	return 0;
}

static __poll_t si_pep_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct si_pep_dev *pep;
	struct si_pep_qdev *qdev;
	struct si_pep_req_cache_hdr *chdr;
	wait_queue_head_t *req_wait;

	qdev = filp->private_data;
	pep = qdev_to_pep_dev(qdev);

	si_qdbg_l(qdev, "Polling %s", qdev->devname);

	if (pep->state != SOC_STATE_RUN)
		return -EAGAIN;

	if (!queue_is_data(qdev)) {
		req_wait = &pep->mreq_wait;
		chdr = &pep->mchdr;
	} else {
		req_wait = &pep->dreq_wait[qdev->qid];
		chdr = &pep->dchdr[qdev->qid];
	}

	if (use_driver_loopback)
		return -ENOENT;

	if (chdr->nreqs != 0) {
		mask |= EPOLLIN | EPOLLPRI;
		return mask;
	}

	poll_wait(filp, req_wait, wait);
	if (chdr->nreqs != 0) {
		mask |= EPOLLIN | EPOLLPRI;
	} else
		mask = 0;

	return mask;
}

static int si_pep_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static long si_pep_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct si_pep_dev *pep;
	struct si_pep_qdev *qdev;
	struct si_data_param param;
	struct si_buffer *buf;
	u64 time;
	int idx;
	int qid;
	int ret;

	qdev = filp->private_data;
	pep = qdev_to_pep_dev(qdev);
	qid = qdev->qid;

	switch (cmd) {
	case SI_GET_DATA:
		ret = copy_from_user(&param, (void __user*)arg, sizeof(param));
		if (ret) {
			si_err(pep, "Failed to copy arguments");
			return -EFAULT;
		}

		buf = &param.src;
		if (buf->bid.qid != qid) {
			si_err(pep, "Invalid buffer for queue %d", qid);
			return -EINVAL;
		}

		if (buf->bid.btype != SI_BUF_TYPE_TX) {
			si_err(pep, "Invalid buffer type");
			return -EINVAL;
		}

		idx = ((struct si_qe_id*)&buf->qidx)->idx;
		if (dump_queue_occupency == true) {
			time = ktime_get_ns();
			qdev->timepoint_3[idx] = time;
			qdev->timepoint_4[idx] = 0;
		}

		ret = si_pep_recv_data(qdev, &param);
		buf = &param.dst;
		if (queue_is_data(qdev))
			dq_set_wq_tail(pep, qid, idx + 1);
		else
			mq_set_wq_tail(pep, idx + 1);
		
		if (!ret)
			qdev->stats.rx_bytes += buf->tsize;

		ret = copy_to_user((void __user *)arg, &param, sizeof(param));
		if (dump_queue_occupency == true) {
			time = ktime_get_ns();
			qdev->timepoint_4[idx] = time;
		}
		return ret;
		break;
	default:
		si_err(pep, "Invalid ioctl for device");
	}
	
	return -ENOTTY;
}

static int si_pep_fasync(int fd, struct file *filp, int mode)
{
	struct si_pep_dev *pep;
	struct si_pep_qdev *qdev;

	qdev = filp->private_data;
	pep = qdev_to_pep_dev(qdev);

	return fasync_helper(fd, filp, mode, &pep->fasync);
}

static int si_pep_read_request(struct si_pep_qdev *qdev, struct si_io *io,
		unsigned int f_flags)
{
	u32 qid;
	wait_queue_head_t *wait;
	struct si_pep_dev *pep = qdev->pep;
	struct si_pep_req_cache_hdr *chdr;
	struct si_wqe wqe;
	struct si_wqe *req_cache;
	struct mutex *m;

	qid = qdev->qid;
	if (!queue_is_data(qdev)) {
		chdr = &pep->mchdr;
		m = &pep->mreq_cache_mutex;
		req_cache = pep->mreq_cache;
		wait = &pep->mreq_wait;
	} else {
		if (pep->state != SOC_STATE_RUN)
			return -EAGAIN;

		if (pep->dwq_ctx[qid]->status == SI_FLUSHING)
			return -EAGAIN;

		chdr = &pep->dchdr[qid];
		m = &pep->dreq_cache_mutex[qid];
		req_cache = pep->dreq_caches[qid];
		wait = &pep->dreq_wait[qid];
	}

	/* Check for break frequency parameter and if set, skip a request */
	if (breakfreq) {
		if (breakcntr == breakfreq) {
			breakcntr = 0;
			mutex_lock(m);
			chdr->nreqs = chdr->nreqs - 1;
			chdr->head = chdr->head + 1;
			if (chdr->head == chdr->maxreqs)
				chdr->head = 0;
			mutex_unlock(m);
		} else {
			breakcntr++;
		}
	}

	/* Consume WQE */
	mutex_lock(m);
	/*
	 * qidx.inuse in cache dreq will be 0 only if that dreq is aborted
	 * by the host. So check if the dreq was aborted and skip current dreq
	 * and memcpy next dreq to user.
	 */
	si_qdbg_l(qdev, "Available requests: %d. Head %d, tail: %d",
			chdr->nreqs, chdr->head, chdr->tail);

	while (req_cache[chdr->head].qidx.inuse == 0) {
		chdr->nreqs--;
		chdr->head++;
		if (chdr->head == chdr->maxreqs)
			chdr->head = 0;
		if (chdr->nreqs == 0)
			break;
	}

	si_qdbg(qdev, "Available requests: %d. Head %d, tail: %d",
			chdr->nreqs, chdr->head, chdr->tail);
	if (chdr->nreqs == 0) {
		if (f_flags & O_NONBLOCK) {
			mutex_unlock(m);
			return -EAGAIN;
		}
		mutex_unlock(m);
		if (wait_event_interruptible(*wait, (chdr->nreqs != 0)))
			return -ERESTARTSYS;
		mutex_lock(m);
	}

	memcpy(&wqe, &req_cache[chdr->head], sizeof(struct si_wqe));
	chdr->nreqs--;
	chdr->head++;
	if (chdr->head == chdr->maxreqs)
		chdr->head = 0;
	si_qdbg(qdev, "Index: %d, Seq: %u. Cache Head %d, tail: %d",
			wqe.qidx.idx, wqe.seqid, chdr->head, chdr->tail);
	mutex_unlock(m);

	io->cmd = wqe.cmd;
	memcpy(&io->desc, &wqe.params, sizeof(wqe.params));
	si_qdbg_l(qdev, "Command: 0x%x, Type: %u, Sequence ID: %u",
			io->cmd, io->desc.type, io->desc.frame_id_l);
	if (wqe.data_size != 0 && (wqe.sge.type == SGE_TYPE_SINGLE ||
				wqe.sge.type == SGE_TYPE_LIST)) {
		io->buffer.bid_int = wqe.sge.bidx;
		io->buffer.addrtype = (wqe.sge.type == SGE_TYPE_SINGLE)?
			SI_BUF_ADDR_SGE:SI_BUF_ADDR_SGE_LIST;
		io->buffer.nents = (wqe.sge.type == SGE_TYPE_SINGLE)?
			1:wqe.sge.size;
		io->buffer.addr = (void *)(((u64)wqe.sge.addr_h << 32) |
				wqe.sge.addr_l);
		io->buffer.size = wqe.data_size;
		io->buffer.tsize = wqe.data_size;
		io->buffer.qidx = *((u32*)&wqe.qidx);
	} else {
		memset(&io->buffer, 0, sizeof(struct si_buffer));
		if (queue_is_data(qdev))
			dq_set_wq_tail(pep, qid, wqe.qidx.idx + 1);
		else
			mq_set_wq_tail(pep, wqe.qidx.idx + 1);
	}

	return 0;
}

static ssize_t si_pep_read(struct file *filp, char __user *buf, size_t sz,
			   loff_t *off)
{
	int ret;
	struct si_pep_dev *pep;
	struct si_pep_qdev *qdev;
	struct si_io io = { 0 };

	if (use_driver_loopback)
		return -ENOENT;

	qdev = filp->private_data;
	pep = qdev_to_pep_dev(qdev);

	if (sz < sizeof(io)) {
		si_err(pep, "Buffer too small");
		return -ENOSPC;
	}

	si_pep_process_timestamp(pep, RECORD_ENTRY_TS, &qdev->read_ts);
	ret = si_pep_read_request(qdev, &io, filp->f_flags);
	if (ret)
		return ret;

	if (copy_to_user(buf, &io, sizeof(struct si_io)))
		sz = -EFAULT;

	if (dump_queue_occupency == true) {
		qdev->timepoint_2[io.buffer.bid.idx] = ktime_get_ns();
		qdev->timepoint_3[io.buffer.bid.idx] = 0;
	}
	si_pep_process_timestamp(pep, RECORD_EXIT_TS, &qdev->read_ts);
	si_pep_process_timestamp(pep, COMPUTE_TS_METRICS, &qdev->read_ts);

	return sz;
}

static ssize_t si_pep_write(struct file *filp, const char __user *buff,
			    size_t sz, loff_t *off)
{
	struct si_pep_dev *pep;
	struct si_pep_qdev *qdev;
	struct si_io io;
        size_t size = 0;
	int ret;
	u32 qid;

	if (use_driver_loopback)
		return -ENOENT;

	if (sz < sizeof(io)) {
		si_err(pep, "Invalid write size");
		return -EINVAL;
	}

	qdev = filp->private_data;
	pep = qdev_to_pep_dev(qdev);
	qid = qdev->qid;
	
	if (copy_from_user(&io, buff, sizeof(struct si_io)))
		return -EFAULT;

	si_pep_process_timestamp(pep, RECORD_ENTRY_TS, &qdev->write_ts);
	ret = si_pep_send_io(qdev, &io);
	if (!ret)
		size = io.buffer.tsize;
	si_pep_process_timestamp(pep, RECORD_EXIT_TS, &qdev->write_ts);
	si_pep_process_timestamp(pep, COMPUTE_TS_METRICS, &qdev->write_ts);

	return size;
}

#define LPBACK_BUF_SIZ	(4 * 1024 * 1024)
static int si_pep_data_loopback_handler(void *arg)
{
	struct si_pep_thread_param *param = (struct si_pep_thread_param *)arg;
	struct si_pep_dev *pep = param->pep;
	struct si_data_param dparam = { 0 };
	struct si_buffer *srcb = &dparam.src;
	struct si_buffer *dstb = &dparam.dst;
	struct si_io io;
	struct si_pep_qdev *qdev;
	u32 qid = param->qid;
	void *data;
	dma_addr_t pdata;
	int ret;

	data = dma_alloc_coherent(pep->dev, LPBACK_BUF_SIZ, &pdata, GFP_KERNEL);
	if (data == NULL) {
		si_err(pep, "Failed to allocate loopback buffer. Qid: %u", qid);
		return 0;
	}

	qdev = &pep->qdev[param->qid + 1];
	dstb->addrtype = SI_BUF_ADDR_PHYSICAL;
	dstb->nents = 1;
	dstb->tsize = LPBACK_BUF_SIZ;
	dstb->size = LPBACK_BUF_SIZ;
	dstb->addr = (void*)pdata;

	while (1) {
		if (kthread_should_stop()) {
			si_info(pep, "Stop request, loopback handler %d", qid);
			break;
		}

		ret = si_pep_read_request(qdev, &io, 0);
		if (ret) {
			si_err(pep, "Failed to read request: %d", ret);
			continue;
		}

		memcpy(srcb, &io.buffer, sizeof(struct si_buffer));
		si_pep_recv_data(qdev, &dparam);
		dq_set_wq_tail(pep, param->qid, io.buffer.bid.idx + 1);
		memcpy(&io.buffer, dstb, sizeof(struct si_buffer));
		si_pep_send_io(qdev, &io);
	}

	dma_free_coherent(pep->dev, LPBACK_BUF_SIZ, data, pdata);
	return 0;
}

static void si_pep_dump_devinfo(struct si_pep_dev *pep)
{
	u32 t1;
	u32 t2;

	t1 = ioread16(pep->dbi_base + PCI_VENDOR_ID);
	t2 = ioread16(pep->dbi_base + PCI_DEVICE_ID);
	si_info(pep, "Vendor ID: 0x%x, Device ID: 0x%x", t1, t2);

	t1 = ioread32(pep->dbi_base + SI_PCI_PF0_PORT_LOGIC +
		      PL_PCIE_VERSION_NUMBER);
	t2 = ioread32(pep->dbi_base + SI_PCI_PF0_PORT_LOGIC +
		      PL_PCIE_VERSION_TYPE);
	si_info(pep, "PCIe IP Version Number: 0x%x, PCIe Version Type: 0x%x",
		     t1, t2);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 12, 0)
static void oops_do_dump(struct kmsg_dumper *dumper,
			 struct kmsg_dump_detail *detail)
#else
static void oops_do_dump(struct kmsg_dumper *dumper,
			 enum kmsg_dump_reason reason)
#endif
{
	struct si_pep_dev *pep = kmsg_dumper_to_pep_dev(dumper);
	struct si_alert alert;
	char *message = "SoC Kernel Crashed ";
	
	pep->drv_ops->set_soc_state(pep, SOC_STATE_INVALID);

	alert.type = SI_ALERT_KERNEL_CRASH;
	alert.sub_type = 0;
	strncpy(alert.data, message, sizeof(alert.data) - 1);
	si_pep_alert_host(pep, &alert);
}

static const struct file_operations si_pep_fops = {
	.owner = THIS_MODULE,
	.open = si_pep_open,
	.release = si_pep_release,
	.poll = si_pep_poll,
	.mmap = si_pep_mmap,
	.unlocked_ioctl = si_pep_ioctl,
	.read = si_pep_read,
	.write = si_pep_write,
	.fasync = si_pep_fasync,
	.llseek = noop_llseek,
};

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 2, 0)
static char *si_pep_devnode(struct device *dev, umode_t *mode)
#else
static char *si_pep_devnode(const struct device *dev, umode_t *mode)
#endif
{
	struct si_pep_qdev *qdev = dev_get_drvdata(dev);

	if (mode)
		*mode = 0666;

	return kstrdup(qdev->devname, GFP_KERNEL);
}

static int si_pep_create_device(struct si_pep_dev *pep)
{
	int ret;
	static int created = 0;
	unsigned int major;

	if (created)
		return 0;

	ret = alloc_chrdev_region(&pep->devt, 0, SI_MAX_DQS + 1,
				  DRV_MODULE_NAME);
	if (ret) {
		si_err(pep, "Failed to allocate char dev region");
		return ret;
	}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(6, 4, 0)
	pep->class = class_create(THIS_MODULE, DRV_MODULE_NAME);
#else
	pep->class = class_create(DRV_MODULE_NAME);
#endif
	pep->class->devnode = si_pep_devnode;
	cdev_init(&pep->cdev, &si_pep_fops);
	ret = cdev_add(&pep->cdev, pep->devt, SI_MAX_DQS + 1);
	if (ret) {
		si_err(pep, "Failed to add cdev");
		return ret;
	}

	major = MAJOR(pep->devt);
	snprintf(pep->qdev[0].devname, sizeof(pep->qdev[0].devname),
		 "sima_pep");
	pep->qdev[0].qid = 0;
	pep->qdev[0].qtype = SI_MWQ;
	pep->qdev[0].pep = pep;
	device_create(pep->class, NULL, MKDEV(major, 0), &pep->qdev[0],
		      &pep->qdev[0].devname[0]);
	return 0;
}

static void si_pep_alert_host(struct si_pep_dev *pep, struct si_alert *alert)
{
	void __iomem *lalert;
	
	if (pep->host_alert_addr == 0) {
		si_err(pep, "Asynchronous notifications are not configured");
		return;
	}

	lalert = si_pep_cache_virt(pep, alert);
	si_pep_iomem_write_rep(lalert, alert, sizeof(struct si_alert));

	if (si_pep_copy_to_host(pep, pep->host_alert_addr,
				(si_pep_cache_phys(pep, alert)),
				sizeof(struct si_alert),
				ALERT_DMA_CH, SI_ALERT_MSI)) {
		si_err(pep, "Failed to alert host");
	}
}

static int si_pep_restart_handler(struct notifier_block *this,
				  unsigned long mode, void *cmd)
{
	struct si_pep_dev *pep = reboot_notifier_to_pep_dev(this);
	struct si_alert alert;
	char *message = "SoC is Rebooting\n";

	pep->drv_ops->set_soc_state(pep, SOC_STATE_INVALID);

	alert.type = SI_ALERT_REBOOT_SOC;
	alert.sub_type = 0;
	strncpy(alert.data, message, sizeof(alert.data) - 1);
	si_pep_alert_host(pep, &alert);

	return 0;
}

static int si_pep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct si_pep_dev *pep;
	struct device_node *node;
	struct si_pep_qdev *qd;
	const struct of_device_id *match;
	const struct si_pep_of_data *matched_data;
	int val;
	int ret;
	int tpsize;
	int i;

	match = of_match_device(si_pep_of_match, dev);
	if (match == NULL) {
		dev_err(dev, "No SiMa PEP devices found\n");
		return -EINVAL;
	}

	pep = devm_kzalloc(dev, sizeof(struct si_pep_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pep)) {
		dev_err(dev, "Failed to allocate driver private data\n");
		return PTR_ERR(pep);
	}

	g_pep_dev = pep;
	matched_data = (struct si_pep_of_data*)match->data;
	pep->dev = dev;
	pep->pdev = pdev;
	platform_set_drvdata(pdev, pep);

	si_info(pep, "Found %s silicon", matched_data->name);
	if (!strncmp(matched_data->name, "davinci", 7)) {
		pep->drv_ops = &davinci_drv_ops;
		pep->modalix = false;
	} else if (!strncmp(matched_data->name, "modalix", 7)) {
		pep->drv_ops = &modalix_drv_ops;
		pep->modalix = true;
	} else {
		si_err(pep, "Invalid SiMa PEP device");
		return -EINVAL;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res == NULL)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pep->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pep->dbi_base)) {
		si_err(pep, "Failed to map 'dbi' resources");
		return PTR_ERR(pep->dbi_base);
	}
	pep->dbi_size = resource_size(res);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi2");
	if (res == NULL)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	pep->dbi2_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pep->dbi2_base)) {
		si_err(pep, "Failed to map 'dbi2' resources");
		return PTR_ERR(pep->dbi2_base);
	}
	pep->dbi2_size = resource_size(res);

	node = of_parse_phandle(dev->of_node, "ocm-region", 0);
	if (!node) {
		si_err(pep, "No OCM region in device tree");
		return -EINVAL;
	} else {
		struct resource tmp_res;
		val = of_address_to_resource(node, 0, &tmp_res);
		if (val) {
			si_err(pep, "Can't parse OCM region in DT");
			return -EINVAL;
		}

		pep->ocm_size = resource_size(&tmp_res);
		pep->ocm_phys_base = tmp_res.start;
		pep->ocm_base = devm_ioremap(
			dev, tmp_res.start, resource_size(&tmp_res));
		if (IS_ERR(pep->ocm_base)) {
			val = PTR_ERR(pep->ocm_base);
			return -EINVAL;
		}

		pep->cache_base = (struct si_pep_cache *)pep->ocm_base;
		pep->cache_pbase = pep->ocm_phys_base;
	}
	of_node_put(node);

	pep->drv_ops->platform_init(pep);
	si_pep_get_msi_prams(pep, &pep->msg_addr_lower, &pep->msg_addr_upper);
	si_pep_dump_devinfo(pep);

	if (pep->drv_ops->set_mlsoc_caps) {
		struct si_mod_mlsoc_caps_0 cap;
		u32 val;
		memset(&cap, 0, sizeof(struct si_mod_mlsoc_caps_0));
		cap.tops = 2;
		cap.max_mqs = 1;
		cap.max_dqs = SI_MAX_DQS - 1;
		cap.max_cameras = SI_MAX_DQS - 1;
		memcpy(&val, &cap, sizeof(struct si_mod_mlsoc_caps_0));
		pep->drv_ops->set_mlsoc_caps(pep, SI_MLSOC_CAPABILITIES_0, val);
	}

	INIT_DELAYED_WORK(&pep->link_down_handler, si_pep_link_down_handler);
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		spin_lock_init(&pep->dma_write_lock[i]);
		spin_lock_init(&pep->dma_read_lock[i]);
	}

	mutex_init(&pep->qdev[0].ioctl_mutex);
	init_waitqueue_head(&pep->mreq_wait);

	pep->mwq_ctx = si_pep_cache_virt(pep, mwq_ctx);
	pep->p_mwq_ctx = si_pep_cache_phys(pep, mwq_ctx);
	si_pep_iomem_memset(pep->mwq_ctx, 0, sizeof(struct si_qctx));
	pep->mcq_ctx = si_pep_cache_virt(pep, mcq_ctx);
	pep->p_mcq_ctx = si_pep_cache_phys(pep, mcq_ctx);
	si_pep_iomem_memset(pep->mcq_ctx, 0, sizeof(struct si_qctx));
	pep->mrq_ctx = si_pep_cache_virt(pep, mrq_ctx);
	pep->p_mrq_ctx = si_pep_cache_phys(pep, mrq_ctx);
	si_pep_iomem_memset(pep->mrq_ctx, 0, sizeof(struct si_qctx));

	pep->mreq_cache = (struct si_wqe *)si_pep_cache_virt(pep, mreq_cache);
	pep->p_mreq_cache = si_pep_cache_phys(pep, mreq_cache);
	si_pep_iomem_memset(pep->mreq_cache, 0, MREQ_CACHE_SIZE);

	memset(&pep->mchdr, 0, sizeof(struct si_pep_req_cache_hdr));
	pep->mchdr.maxreqs = SI_MWQ_CACHE_SIZE;
	mutex_init(&pep->mreq_cache_mutex);

	for (i = 0; i < SI_MAX_DQS; i++) {
		pep->dwq_ctx[i] = si_pep_cache_virt(pep, dwq_ctx[i]);
		pep->p_dwq_ctx[i] = si_pep_cache_phys(pep, dwq_ctx[i]);
		si_pep_iomem_memset(pep->dwq_ctx[i], 0, sizeof(struct si_qctx));
		pep->dcq_ctx[i] = si_pep_cache_virt(pep, dcq_ctx[i]);
		pep->p_dcq_ctx[i] = si_pep_cache_phys(pep, dcq_ctx[i]);
		si_pep_iomem_memset(pep->dcq_ctx[i], 0, sizeof(struct si_qctx));
		pep->drq_ctx[i] = si_pep_cache_virt(pep, drq_ctx[i]);
		pep->p_drq_ctx[i] = si_pep_cache_phys(pep, drq_ctx[i]);
		si_pep_iomem_memset(pep->drq_ctx[i], 0, sizeof(struct si_qctx));
		
		pep->dreq_caches[i] = (struct si_wqe *)si_pep_cache_virt(pep,
				dreq_caches[i]);
		pep->p_dreq_caches[i] = si_pep_cache_phys(pep, dreq_caches[i]);
		si_pep_iomem_memset(pep->dreq_caches[i], 0, DREQ_CACHE_SIZE);
		memset(&pep->dchdr[i], 0, sizeof(struct si_pep_req_cache_hdr));
		pep->dchdr[i].maxreqs = SI_DWQ_CACHE_SIZE;

		init_waitqueue_head(&pep->dreq_wait[i]);
		mutex_init(&pep->dreq_cache_mutex[i]);

		if (use_driver_loopback) {
			t_params[i].pep = pep;
			t_params[i].qid = i;
			pep->loopback_hdlr[i] =
				kthread_create(si_pep_data_loopback_handler,
						&t_params[i],
						"sima-loopback-hdlr-%d", i);
		}

		qd = &pep->qdev[i+1];
		qd->qid = i;
		qd->qtype = SI_DWQ;
		qd->pep = pep;
		mutex_init(&qd->ioctl_mutex);
		snprintf(qd->devname, sizeof(qd->devname), "sima_pep.%d", i);

		if (dump_queue_occupency == true) {
			tpsize = sizeof(ktime_t) * MAX_TIMEPOINTS;
			qd->timepoint_1 = devm_kzalloc(dev, tpsize, GFP_KERNEL);
			qd->timepoint_2 = devm_kzalloc(dev, tpsize, GFP_KERNEL);
			qd->timepoint_3 = devm_kzalloc(dev, tpsize, GFP_KERNEL);
			qd->timepoint_4 = devm_kzalloc(dev, tpsize, GFP_KERNEL);
			
			if (IS_ERR_OR_NULL(qd->timepoint_1) ||
					IS_ERR_OR_NULL(qd->timepoint_2) ||
					IS_ERR_OR_NULL(qd->timepoint_3) ||
					IS_ERR_OR_NULL(qd->timepoint_4)) {
				si_err(pep, "Failed to allocate memory for"
						"time points");
				dump_queue_occupency = false;
				devm_kfree(dev, qd->timepoint_1);
				devm_kfree(dev, qd->timepoint_2);
				devm_kfree(dev, qd->timepoint_3);
				devm_kfree(dev, qd->timepoint_4);
			}
		}
	}

	pep->kdump.dump = oops_do_dump;
	ret = kmsg_dump_register(&pep->kdump);
	if (ret)
		si_err(pep, "Registering kmsg dumper failed with %d", ret);

	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64)))
		si_err(pep, "Failed to set DMA mask");

	/* Set SoC Status as Fully initialized */
	pep->drv_ops->local_init(pep);
	si_info(pep, "SoC State Initialized");

	pep->reboot_handler.notifier_call = si_pep_restart_handler;
	pep->reboot_handler.priority = 255;
	register_restart_handler(&pep->reboot_handler);

	pep->kobj_stats = kobject_create_and_add("statistics", &pep->dev->kobj);
	if (pep->kobj_stats == NULL) {
		si_err(pep, "Cannot create folder in sysfs");
	} else {
		if (sysfs_create_file(pep->kobj_stats, &sys_timestamp.attr))
			si_err(pep, "Cannot create sysfs file");
		if (sysfs_create_file(pep->kobj_stats, &sys_mgmt_stats.attr))
			si_err(pep, "Cannot create mgmt stats sysfs file");
		if (sysfs_create_file(pep->kobj_stats, &sys_data_stats.attr))
			si_err(pep, "Cannot create data stats sysfs file");
	}

	if (dump_queue_occupency == true) {
		if (sysfs_create_file(pep->kobj_stats, &sys_queue_occup.attr))
			si_err(pep, "Cannot create queue occupency sysfs file");
	}

	if (pep->modalix)
		pep->drv_ops->set_soc_state(pep, SOC_STATE_RUN);

	if (si_pep_create_device(pep)) {
		si_err(pep, "Failed to register character device");
		return -ENOMEM;
	}

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int si_pep_remove(struct platform_device *pdev)
#else
static void si_pep_remove(struct platform_device *pdev)
#endif
{
	int i;
	struct si_pep_dev *pep;

	pep = platform_get_drvdata(pdev);
	si_pep_teardown_netintf(pep);
	wake_up_interruptible(&pep->mreq_wait);
	pep->drv_ops->local_deinit(pep);
	pep->drv_ops->platform_deinit(pep);
	sysfs_remove_file(pep->kobj_stats, &sys_timestamp.attr);
	sysfs_remove_file(pep->kobj_stats, &sys_mgmt_stats.attr);
	sysfs_remove_file(pep->kobj_stats, &sys_data_stats.attr);
	if (dump_queue_occupency)
		sysfs_remove_file(pep->kobj_stats, &sys_queue_occup.attr);
	kobject_put(pep->kobj_stats);

	if (use_driver_loopback) {
		for (i = 0; i < SI_MAX_DQS; i++) {
			if (pep->loopback_hdlr[i]) {
				kthread_stop(pep->loopback_hdlr[i]);
				wake_up_interruptible(&pep->dreq_wait[i]);
			}
		}
	}

	for (i = 0; i < SI_MAX_DQS; i++) {
		if (dump_queue_occupency == true) {
			kfree(pep->qdev[i+1].timepoint_1);
			kfree(pep->qdev[i+1].timepoint_2);
			kfree(pep->qdev[i+1].timepoint_3);
			kfree(pep->qdev[i+1].timepoint_4);
		}
	}

	device_destroy(pep->class, pep->devt);
	class_destroy(pep->class);
	cdev_del(&pep->cdev);
	unregister_chrdev_region(pep->devt, SI_MAX_DQS + 1);
	unregister_restart_handler(&pep->reboot_handler);
	kmsg_dump_unregister(&pep->kdump);
	devm_kfree(&pdev->dev, pep);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
	return 0;
#endif
}

static const struct si_pep_of_data davinci_of_data = {
	.data = 0xabcd,
	.name = "davinci"
};

static const struct si_pep_of_data modalix_of_data = {
	.data = 0x0001,
	.name = "modalix"
};

static const struct of_device_id si_pep_of_match[] = {
	{ .compatible = "simaai,davinci-pcie-ep-1.0",
		.data = &davinci_of_data },
	{ .compatible = "simaai,modalix-pcie-ep-1.0",
		.data = &modalix_of_data },
	{}
};
MODULE_DEVICE_TABLE(of, si_pep_of_match);

static struct platform_driver si_pep_platform_drv = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = si_pep_of_match,
	},
	.probe  = si_pep_probe,
	.remove = si_pep_remove,
};

static int __init si_pep_drv_init(void)
{
	return platform_driver_register(&si_pep_platform_drv);
}

static void __exit si_pep_drv_exit(void)
{
	platform_driver_unregister(&si_pep_platform_drv);
}

module_init(si_pep_drv_init);
module_exit(si_pep_drv_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SiMa.ai SoC PCIe Endpoint Driver");
MODULE_AUTHOR("SiMa.ai");
MODULE_ALIAS("platform:sima_pep_drv");
