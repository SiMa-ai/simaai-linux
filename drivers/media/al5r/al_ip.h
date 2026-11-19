/******************************************************************************
*
* Copyright (C) 2019 Allegro DVT2.  All rights reserved.
*
******************************************************************************/

#pragma once

#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include "al_ioctl.h"
#include "al_alloc.h"

#define AL5R_NR_DEVS 4

#define AXI_ADDR_OFFSET_IP(codec) ((codec)->regs_min_addr + 0x1208)
#define AL5_INTERRUPT_MASK(codec) ((codec)->regs_min_addr + 0x14)
#define AL5_INTERRUPT(codec) ((codec)->regs_min_addr + 0x18)

#define al5_writel(val, reg) iowrite32(val, codec->regs + reg)
#define al5_readl(reg) ioread32(codec->regs + reg)

#define al5_dbg(format, ...) \
	dev_dbg(codec->device, format, ## __VA_ARGS__)

#define al5_info(format, ...) \
	dev_info(codec->device, format, ## __VA_ARGS__)

#define al5_err(format, ...) \
	dev_err(codec->device, format, ## __VA_ARGS__)

struct al5r_codec_desc;
struct al5_cmd_engine;
struct dma_buf_info {
	struct al5_dma_buffer *buffer;
	struct al5r_codec_desc *codec;
};

struct r_irq {
	struct list_head list;
	u32 bitfield;
};

struct al5r_codec_desc {
	struct device *device;
	void __iomem *regs;             /* base addr of the ip hw registers */
	unsigned long regs_size;
	u32 regs_min_addr;              /* first regs that can be addressed in the mmio.*/
	dma_addr_t dma_offset;          /* hw ip adds this offset to the dma address it is given */
	struct cdev cdev;
	/* exclusive owner of the raw register interface, if any */
	struct al5r_codec_chan *legacy_chan;
	/* exclusive holder of the MVD (AV1/VP9) engine, if any */
	struct al5r_codec_chan *mvd_chan;
	struct al5_cmd_engine *engine;
	spinlock_t i_lock;
	struct kmem_cache *cache;
	int minor;
	bool dma_mask_64_bit; /* sima specific parameter for 64-bit dma mask */
};

struct al5_dma_buf_mmap {
	struct list_head list;
	struct al5_dma_buffer *buf;
	int buf_id;
};

struct al5r_codec_chan {
	wait_queue_head_t irq_queue;
	int unblock;
	spinlock_t lock;
	struct list_head mem;
	int num_bufs;
	/* DMA-BUFs this channel resolved to a bus address.  Refcounted: the
	 * channel holds one reference, and any command still on the hardware
	 * when the channel closes holds another, so a mapping outlives both
	 * the address it produced and the channel that asked for it.
	 */
	struct al5_import_set *import_set;
	struct al5r_codec_desc *codec;
	/* interrupt events routed to this channel (legacy owner / MVD
	 * holder), protected by codec->i_lock
	 */
	struct list_head irq_masks;
	/* Commands admitted on this channel and not yet retired, protected
	 * by engine->lock.  64-bit: the count cannot wrap, whatever the
	 * reporting field can hold.
	 */
	u64 queued_cmds;
	/* command-arbiter completions, protected by engine->lock */
	struct list_head done_list;
	wait_queue_head_t done_wait;
	bool holds_mvd;
};

int al5r_codec_read_register(struct al5r_codec_chan *chan,
			     struct al5_reg *reg);
void al5r_codec_write_register(struct al5r_codec_chan *chan,
			       struct al5_reg *reg);
irqreturn_t al5r_irq_handler(int irq, void *data);
irqreturn_t al5r_hardirq_handler(int irq, void *data);
