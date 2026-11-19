/******************************************************************************
*
* Copyright (C) 2019 Allegro DVT2.  All rights reserved.
*
******************************************************************************/

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "al_ip.h"
#include "al_cmd.h"

int al5r_codec_read_register(struct al5r_codec_chan *chan,
			     struct al5_reg *reg)
{
	struct al5r_codec_desc *codec = chan->codec;

	if (!chan->codec->regs) {
		al5_err("Registers not mapped\n");
		return -EINVAL;
	}
	reg->value = ioread32(chan->codec->regs + reg->id);

	return 0;
}

void al5r_codec_write_register(struct al5r_codec_chan *chan,
			       struct al5_reg *reg)
{
	struct al5r_codec_desc *codec = chan->codec;

	if (!chan->codec->regs) {
		al5_err("Registers not mapped\n");
		return;
	}
	iowrite32(reg->value, chan->codec->regs + reg->id);

}

irqreturn_t al5r_hardirq_handler(int irq, void *data)
{
	struct al5r_codec_desc *codec = (struct al5r_codec_desc *)data;
	u32 unmasked_irq_bitfield, irq_bitfield;
	u32 mask;
	unsigned long flags;
	struct r_irq *i_callback;
	int i = 0;

	mask = ioread32(codec->regs + AL5_INTERRUPT_MASK(codec));
	unmasked_irq_bitfield = ioread32(codec->regs + AL5_INTERRUPT(codec));
	irq_bitfield = unmasked_irq_bitfield & mask;
	if (irq_bitfield == 0) {
		al5_dbg("bitfield is 0\n");
		return IRQ_NONE;
	}
	iowrite32(unmasked_irq_bitfield, codec->regs + AL5_INTERRUPT(codec));
	ioread32(codec->regs + AL5_INTERRUPT(codec));

	al5_dbg("got irq. bitfield:%x, mask:%x", unmasked_irq_bitfield, mask);

	for (i = 0; i < 32; ++i) {
		struct al5r_codec_chan *target;

		if (!(irq_bitfield & (1U << i)))
			continue;

		/* completion of a command submitted through the arbiter */
		if (al5_cmd_engine_irq(codec, i))
			continue;

		/* raw event for the legacy owner or the MVD holder */
		spin_lock_irqsave(&codec->i_lock, flags);
		if (((1U << i) & AL5_MVD_IRQ_MASK) && codec->mvd_chan)
			target = codec->mvd_chan;
		else
			target = codec->legacy_chan;
		if (!target) {
			spin_unlock_irqrestore(&codec->i_lock, flags);
			continue;
		}
		i_callback = kmem_cache_alloc(codec->cache, GFP_ATOMIC);
		if (!i_callback) {
			spin_unlock_irqrestore(&codec->i_lock, flags);
			al5_dbg("ENOMEM: Missed interrupt\n");
			return IRQ_NONE;
		}
		i_callback->bitfield = i;
		list_add_tail(&i_callback->list, &target->irq_masks);
		wake_up_interruptible(&target->irq_queue);
		spin_unlock_irqrestore(&codec->i_lock, flags);
	}

	return IRQ_HANDLED;
}
