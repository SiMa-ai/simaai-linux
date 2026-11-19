// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 SiMa.ai
 *
 * Codec-agnostic command arbiter for the Allegro register IP.
 *
 * The hardware is a set of independent sub-modules (encoder stages,
 * decoder parser/pixel stages, JPEG, start-code detector), each driven by
 * the same pattern: program a register window, kick a start register, get
 * a completion interrupt.  Every submission fully programs its module, so
 * commands from different processes can be interleaved freely on one
 * module.  The arbiter owns everything global: IP reset, per-module reset
 * and clock gating sequences, interrupt enables, and hang recovery.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "al_ip.h"
#include "al_cmd.h"
#include "al_dmabuf.h"

/* Recovery work discarded because its originating command had already
 * retired.  Counted per engine and logged, so "a retired generation's
 * recovery work is a no-op" is observable on a production kernel.
 */
#define AL5_STALE_RECOVERY_LOG_RATE	10

/* Load sampling window: busy time decays with a ~1s half life. */
#define AL5_LOAD_WINDOW_NS	(1000ull * NSEC_PER_MSEC)

static int al5r_cmd_timeout_ms = 2000;
module_param(al5r_cmd_timeout_ms, int, 0644);
MODULE_PARM_DESC(al5r_cmd_timeout_ms,
		 "default command completion timeout in milliseconds");

/* TOP bank registers, relative to the IP base */
#define AL5_TOP_RESET		0x10
#define AL5_TOP_IRQ_EN		0x14
#define AL5_TOP_IRQ		0x18
#define AL5_TOP_AOM		0x1C	/* decoder: MVD IRQ rearm / clock */
#define AL5_TOP_WDG_CTRL	0x24

#define AL5_RESET_COMMON	BIT(0)
#define AL5_WDT_STOP		0

/* Per-module control registers, relative to the module window */
#define AL5_WIN_RUN		0x1C
#define AL5_WIN_RESET		0x28
#define AL5_WIN_CLOCK		0x2C

#define AL5_RUN_FRAME		BIT(1)
#define AL5_RESET_MODULE	BIT(0)
#define AL5_RESET_MODULE_TOP	BIT(1)
#define AL5_GCLK_AUTO		0
#define AL5_GCLK_FORCE_ON	1
#define AL5_GCLK_FORCE_OFF	2

/* AV1/VP9 engine register aperture */
#define AL5_MVD_WIN_BASE	0x40000
#define AL5_MVD_WIN_END		0x60000

static const struct al5_hw_desc al5_hw_enc_v200 = {
	.type = "enc-v200",
	.common_size = 0x1000,
	.core_size = 0x1000,
	.mods = {
		[AL5_MOD_ENC1] = {
			.name = "enc1", .num_cores = 2, .num_subs = 1,
			.bank = 0x000, .span = 0x800,
			.irq_base = 0, .irq_core_shift = 2,
		},
		[AL5_MOD_ENC2] = {
			.name = "enc2", .num_cores = 2, .num_subs = 1,
			.bank = 0x800, .span = 0x800,
			.irq_base = 1, .irq_core_shift = 2,
		},
		[AL5_MOD_ENC_JPEG] = {
			.name = "encjpeg", .num_cores = 2, .num_subs = 1,
			.first_phys_core = 2,
			.bank = 0x000, .span = 0x800,
			.irq_base = 0, .irq_core_shift = 2,
		},
	},
};

static const struct al5_hw_desc al5_hw_dec_v300 = {
	.type = "dec-v300",
	.common_size = 0x1000,
	.core_size = 0x1000,
	.axi_burst_reg = 0x80C,
	.has_mvd = true,
	.mods = {
		[AL5_MOD_SCD] = {
			.name = "scd", .num_cores = 1, .num_subs = 1,
			.bank = 0x900, .span = 0x300,
			.irq_base = 3, .irq_core_shift = 4,
		},
		[AL5_MOD_DEC1] = {
			.name = "dec1", .num_cores = 1, .num_subs = 2,
			.bank = 0x000, .span = 0x300, .sub_stride = 0x300,
			.irq_base = 0, .irq_core_shift = 4, .irq_sub_shift = 1,
		},
		[AL5_MOD_DEC2] = {
			.name = "dec2", .num_cores = 1, .num_subs = 1,
			.bank = 0x600, .span = 0x300,
			.irq_base = 2, .irq_core_shift = 4,
		},
		[AL5_MOD_DEC_JPEG] = {
			.name = "decjpeg", .num_cores = 1, .num_subs = 1,
			.bank = 0xC00, .span = 0x300,
			.irq_base = 18, .irq_core_shift = 1,
		},
	},
};

static u32 al5_rd(struct al5r_codec_desc *codec, u32 reg)
{
	return ioread32(codec->regs + codec->regs_min_addr + reg);
}

static void al5_wr(struct al5r_codec_desc *codec, u32 reg, u32 val)
{
	iowrite32(val, codec->regs + codec->regs_min_addr + reg);
}

static void al5_apply_writes(struct al5r_codec_desc *codec,
			     const struct al5_reg_write *w, u32 n)
{
	u32 i, v;

	for (i = 0; i < n; ++i, ++w) {
		if (w->mask == ~0u) {
			al5_wr(codec, w->reg, w->value);
		} else {
			v = al5_rd(codec, w->reg);
			al5_wr(codec, w->reg,
			       (v & ~w->mask) | (w->value & w->mask));
		}
	}
}

static void al5_snapshot_status(struct al5r_codec_desc *codec,
				struct al5_pending_cmd *p)
{
	u32 i;

	for (i = 0; i < p->cmd.num_status; ++i)
		p->done.status_values[i] = al5_rd(codec, p->cmd.status_regs[i]);
}

/* engine->lock held.  Retires p one way or the other, and always wakes
 * retire_wait: a closing channel is waiting for exactly this.
 */
static void al5_deliver(struct al5_cmd_engine *e, struct al5_pending_cmd *p)
{
	struct al5r_codec_chan *chan = p->chan;

	if (!chan) {
		/* Orphaned: drop the buffer mappings this command was still
		 * keeping alive on the closed channel's behalf.
		 */
		al5_dmabuf_put_import_set(p->imports);
		kfree(p);	/* channel closed while the command ran */
	} else {
		if (chan->queued_cmds)
			--chan->queued_cmds;
		list_add_tail(&p->node, &chan->done_list);
		wake_up_interruptible(&chan->done_wait);
	}
	wake_up_all(&e->retire_wait);
}

/* engine->lock held.  Halve every slot's accumulated busy time until the
 * sampling window is fresh again.  Called from the activity path as well
 * as from AL5_GET_LOAD, so the window keeps decaying while the hardware
 * is busy even when nobody reads the load: otherwise the first reader
 * after a quiet period sees minutes of idle history diluting the current
 * load to zero, which is exactly when a scheduler asks.
 */
static void al5_load_fold(struct al5_cmd_engine *e, u64 now)
{
	u64 elapsed;
	int i;

	if (!e->load_win_start)
		e->load_win_start = now;

	elapsed = now - e->load_win_start;
	while (elapsed > 2 * AL5_LOAD_WINDOW_NS) {
		for (i = 0; i < e->num_slots; ++i)
			e->slots[i].busy_ns /= 2;
		e->load_win_start = now - elapsed / 2;
		elapsed = now - e->load_win_start;
	}
}

/* engine->lock held; the slot's command just stopped running */
static void al5_slot_account_busy(struct al5_slot *slot)
{
	if (slot->busy_since) {
		u64 now = ktime_get_ns();

		slot->busy_ns += now - slot->busy_since;
		slot->busy_since = 0;
		al5_load_fold(slot->engine, now);
	}
}

/* engine->lock held */
static void al5_slot_start(struct al5_slot *slot, struct al5_pending_cmd *p)
{
	struct al5r_codec_desc *codec = slot->engine->codec;
	unsigned int timeout = p->cmd.timeout_ms ?: al5r_cmd_timeout_ms;

	slot->running = p;
	slot->busy_since = ktime_get_ns();
	if (p->cmd.flags & AL5_CMD_F_RESET_MODULE) {
		/* the per-frame reset pulses only the module reset bit; a
		 * reset-top here would tear down the shared inter-module
		 * pipeline interface under a concurrently running stage
		 */
		al5_wr(codec, slot->base + AL5_WIN_CLOCK, AL5_GCLK_FORCE_OFF);
		al5_wr(codec, slot->base + AL5_WIN_RESET, AL5_RESET_MODULE);
		al5_wr(codec, slot->base + AL5_WIN_CLOCK, AL5_GCLK_AUTO);
		/* atomic context: give the module time to leave reset */
		udelay(100);
	}
	al5_wr(codec, slot->base + AL5_WIN_CLOCK, AL5_GCLK_FORCE_ON);
	al5_apply_writes(codec, p->cmd.writes, p->cmd.num_writes);
	/* make sure the program lands before the kick */
	wmb();
	al5_apply_writes(codec, &p->cmd.kick, 1);
	slot->deadline = jiffies + msecs_to_jiffies(timeout);
	mod_timer(&slot->timer, slot->deadline);
}

/* engine->lock held; completes the running command (if any) */
static void al5_slot_complete_reason(struct al5_slot *slot, int result,
				    const char *reason)
{
	struct al5_pending_cmd *p = slot->running;

	if (!p)
		return;
	slot->running = NULL;
	al5_slot_account_busy(slot);
	timer_delete(&slot->timer);
	/* Drop a timeout marker owned by this command: it has retired, and
	 * the marker must not be inherited by whatever occupies the slot next.
	 */
	if (slot->timed_out && slot->timeout_gen == p->gen) {
		slot->timed_out = false;
		slot->timeout_gen = 0;
	}
	p->done.result = result;
	if (!result)
		al5_snapshot_status(slot->engine->codec, p);
	al5_deliver(slot->engine, p);
}

static void al5_slot_complete(struct al5_slot *slot, int result)
{
	al5_slot_complete_reason(slot, result, result ? "error" : "irq");
}

/* engine->lock held */
static void al5_slot_kick_next(struct al5_slot *slot)
{
	struct al5_pending_cmd *p;

	if (slot->paused || slot->running)
		return;
	if (list_empty(&slot->queue)) {
		al5_wr(slot->engine->codec, slot->base + AL5_WIN_CLOCK,
		       AL5_GCLK_AUTO);
		return;
	}
	p = list_first_entry(&slot->queue, struct al5_pending_cmd, node);
	list_del(&p->node);
	al5_slot_start(slot, p);
}

bool al5_cmd_engine_irq(struct al5r_codec_desc *codec, int bit)
{
	struct al5_cmd_engine *e = codec->engine;
	struct al5_slot *slot;
	int i;

	if (!e || !e->hw || !READ_ONCE(e->hw_ready))
		return false;
	if (bit >= 32 || !(e->irq_mask & BIT(bit)))
		return false;

	spin_lock(&e->lock);
	for (i = 0; i < e->num_slots; ++i) {
		slot = &e->slots[i];
		if (slot->irq_bit != bit)
			continue;
		al5_slot_complete(slot, 0);
		al5_slot_kick_next(slot);
		break;
	}
	spin_unlock(&e->lock);
	return true;
}

static void al5_slot_timeout(struct timer_list *t)
{
	struct al5_slot *slot = container_of(t, struct al5_slot, timer);
	struct al5_cmd_engine *e = slot->engine;
	unsigned long flags;

	spin_lock_irqsave(&e->lock, flags);
	if (slot->running && time_after_eq(jiffies, slot->deadline)) {
		slot->timed_out = true;
		slot->timeout_gen = slot->running->gen;
		queue_work(e->recover_wq, &e->recover_work);
	}
	spin_unlock_irqrestore(&e->lock, flags);
}

/* Reset-top pulse, then module reset with the clock forced off, as done by
 * the reference scheduler at init and hang recovery.
 */
static void al5_module_reset(struct al5r_codec_desc *codec, u32 base)
{
	al5_wr(codec, base + AL5_WIN_RESET, AL5_RESET_MODULE_TOP);
	al5_wr(codec, base + AL5_WIN_CLOCK, AL5_GCLK_FORCE_OFF);
	al5_wr(codec, base + AL5_WIN_RESET, AL5_RESET_MODULE);
	al5_wr(codec, base + AL5_WIN_CLOCK, AL5_GCLK_AUTO);
}

/* Full IP init: common reset, per-module reset, interrupt setup.
 * Process context; serialized by owner_mutex (or recovery, which runs
 * with all slots paused).
 */
static void al5_hw_init(struct al5_cmd_engine *e)
{
	struct al5r_codec_desc *codec = e->codec;
	unsigned long flags;
	u32 v;
	int i;

	al5_wr(codec, AL5_TOP_RESET, AL5_RESET_COMMON);
	al5_rd(codec, 0);	/* flush the posted write */
	usleep_range(2000, 4000);

	for (i = 0; i < e->num_slots; ++i)
		al5_module_reset(codec, e->slots[i].base);
	usleep_range(1000, 2000);

	/* stop the hardware watchdog: timeouts are handled by the driver */
	al5_wr(codec, AL5_TOP_WDG_CTRL, AL5_WDT_STOP);

	/* SiMa NoC compatibility: limit AXI bursts to 256 bytes */
	if (e->hw->axi_burst_reg) {
		u32 reg = e->hw->axi_burst_reg;

		al5_wr(codec, reg, al5_rd(codec, reg) & ~0x3);
		usleep_range(1000, 2000);
		if (al5_rd(codec, reg) & 0x3)
			dev_warn(codec->device,
				 "AXI max burst size setup failed\n");
	}

	/* acknowledge anything pending, then enable the module interrupts;
	 * the MVD bits belong to the MVD holder and are preserved
	 */
	al5_wr(codec, AL5_TOP_IRQ, e->irq_mask);
	spin_lock_irqsave(&codec->i_lock, flags);
	v = al5_rd(codec, AL5_TOP_IRQ_EN);
	al5_wr(codec, AL5_TOP_IRQ_EN, (v & AL5_MVD_IRQ_MASK) | e->irq_mask);
	spin_unlock_irqrestore(&codec->i_lock, flags);
}

static int al5_engine_ensure_ready(struct al5_cmd_engine *e)
{
	int ret = 0;

	if (READ_ONCE(e->hw_ready))
		return 0;

	mutex_lock(&e->owner_mutex);
	if (!e->hw_ready) {
		if (e->codec->legacy_chan) {
			ret = -EBUSY;
		} else {
			al5_hw_init(e);
			WRITE_ONCE(e->hw_ready, true);
			dev_info(e->codec->device,
				 "command arbiter active (%s)\n", e->hw->type);
		}
	}
	mutex_unlock(&e->owner_mutex);
	return ret;
}

static void al5_global_reset(struct al5_cmd_engine *e)
{
	struct al5r_codec_desc *codec = e->codec;
	unsigned long flags;
	int i;

	dev_err(codec->device,
		"module reset failed, resetting the whole IP\n");

	spin_lock_irqsave(&e->lock, flags);
	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];

		slot->paused = true;
		slot->timed_out = false;
		timer_delete(&slot->timer);
		al5_slot_complete(slot, -EIO);
	}
	spin_unlock_irqrestore(&e->lock, flags);

	al5_hw_init(e);

	spin_lock_irqsave(&e->lock, flags);
	for (i = 0; i < e->num_slots; ++i) {
		e->slots[i].paused = false;
		al5_slot_kick_next(&e->slots[i]);
	}
	spin_unlock_irqrestore(&e->lock, flags);
}

/* True when the recovery scheduled for this slot no longer refers to the
 * command occupying it: the originating generation has retired and the slot
 * has been reused, or is empty.  Such work must be discarded, never applied.
 */
bool al5_recovery_is_stale(const struct al5_slot *slot)
{
	return !slot->running || slot->running->gen != slot->timeout_gen;
}

static void al5_recover_work(struct work_struct *work)
{
	struct al5_cmd_engine *e =
		container_of(work, struct al5_cmd_engine, recover_work);
	struct al5r_codec_desc *codec = e->codec;
	bool global_reset = false;
	unsigned long flags;
	int i;

	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];
		struct al5_pending_cmd *p;


		spin_lock_irqsave(&e->lock, flags);
		if (!slot->timed_out) {
			spin_unlock_irqrestore(&e->lock, flags);
			continue;
		}
		p = slot->running;
		if (al5_recovery_is_stale(slot)) {
			/* The command whose timeout scheduled this recovery
			 * has already retired; the slot now holds an
			 * unrelated command, or none.  Discard the stale work.
			 */
			slot->timed_out = false;
			slot->timeout_gen = 0;
			++e->stale_recovery;
			spin_unlock_irqrestore(&e->lock, flags);
			dev_warn_ratelimited(codec->device,
					     "discarded recovery for a retired command on %s core %u.%u (%llu so far)\n",
					     slot->mod->name, slot->core,
					     slot->sub, e->stale_recovery);
			continue;
		}
		slot->timed_out = false;
		slot->timeout_gen = 0;
		slot->paused = true;
		slot->running = NULL;
		slot->recovering = p;
		al5_slot_account_busy(slot);
		spin_unlock_irqrestore(&e->lock, flags);


		dev_warn(codec->device,
			 "command timeout on %s core %u.%u (run %#x irq %#x/%#x), resetting module\n",
			 slot->mod->name, slot->core, slot->sub,
			 al5_rd(codec, slot->base + AL5_WIN_RUN),
			 al5_rd(codec, AL5_TOP_IRQ),
			 al5_rd(codec, AL5_TOP_IRQ_EN));

		al5_module_reset(codec, slot->base);
		usleep_range(1000, 2000);

		spin_lock_irqsave(&e->lock, flags);
		if (p) {
			p->done.result = -ETIMEDOUT;
			slot->recovering = NULL;
			al5_deliver(e, p);
		}
		if (al5_rd(codec, slot->base + AL5_WIN_RUN) & AL5_RUN_FRAME) {
			global_reset = true;
		} else {
			slot->paused = false;
			al5_slot_kick_next(slot);
		}
		spin_unlock_irqrestore(&e->lock, flags);
	}

	if (global_reset)
		al5_global_reset(e);
}

static struct al5_slot *al5_find_slot(struct al5_cmd_engine *e, u32 module,
				      u32 core, u32 sub)
{
	int i;

	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];

		if (slot->module == module && slot->core == core &&
		    slot->sub == sub)
			return slot;
	}
	return NULL;
}

static bool al5_reg_in_window(const struct al5_slot *slot, u32 reg)
{
	return !(reg & 3) && reg >= slot->base &&
	       reg + sizeof(u32) <= slot->base + slot->mod->span;
}

static int al5_validate_cmd(struct al5_cmd_engine *e,
			    const struct al5_cmd *cmd,
			    struct al5_slot **slotp)
{
	const struct al5_mod_desc *mod;
	struct al5_slot *slot;
	u32 i;

	if (cmd->module >= AL5_MOD_COUNT || cmd->module == AL5_MOD_MVD)
		return -EINVAL;
	if (cmd->flags & ~(AL5_CMD_F_NO_KICK | AL5_CMD_F_RESET_MODULE))
		return -EINVAL;
	if ((cmd->flags & AL5_CMD_F_NO_KICK) &&
	    (cmd->flags & AL5_CMD_F_RESET_MODULE))
		return -EINVAL;
	if (cmd->reserved)
		return -EINVAL;
	/* An unbounded timeout lets a caller hold an execution slot for as
	 * long as it likes, and close has to wait for the slot to retire.
	 */
	if (cmd->timeout_ms &&
	    (cmd->timeout_ms < AL5_MIN_TIMEOUT_MS ||
	     cmd->timeout_ms > AL5_MAX_TIMEOUT_MS))
		return -EINVAL;
	if (cmd->num_writes > AL5_CMD_MAX_WRITES ||
	    cmd->num_status > AL5_CMD_MAX_STATUS)
		return -EINVAL;

	mod = &e->hw->mods[cmd->module];
	if (!mod->num_cores || cmd->core >= mod->num_cores ||
	    cmd->sub >= mod->num_subs)
		return -EINVAL;

	slot = al5_find_slot(e, cmd->module, cmd->core, cmd->sub);
	if (WARN_ON(!slot))
		return -EINVAL;

	for (i = 0; i < cmd->num_writes; ++i)
		if (!al5_reg_in_window(slot, cmd->writes[i].reg))
			return -EINVAL;
	if (!(cmd->flags & AL5_CMD_F_NO_KICK) &&
	    !al5_reg_in_window(slot, cmd->kick.reg))
		return -EINVAL;
	for (i = 0; i < cmd->num_status; ++i)
		if (!al5_reg_in_window(slot, cmd->status_regs[i]))
			return -EINVAL;

	*slotp = slot;
	return 0;
}

long al5_cmd_ioctl_submit(struct al5r_codec_chan *chan, unsigned long arg)
{
	struct al5r_codec_desc *codec = chan->codec;
	struct al5_cmd_engine *e = codec->engine;
	struct al5_pending_cmd *p;
	struct al5_slot *slot;
	unsigned long flags;
	int ret;

	if (!e || !e->hw)
		return -ENOTTY;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	if (copy_from_user(&p->cmd, (void __user *)arg, sizeof(p->cmd))) {
		ret = -EFAULT;
		goto err_free;
	}

	ret = al5_validate_cmd(e, &p->cmd, &slot);
	if (ret)
		goto err_free;

	ret = al5_engine_ensure_ready(e);
	if (ret)
		goto err_free;

	p->chan = chan;
	p->done.user_data = p->cmd.user_data;
	p->done.core = p->cmd.core;

	spin_lock_irqsave(&e->lock, flags);
	if (chan->queued_cmds >= AL5_MAX_CHAN_QUEUED) {
		spin_unlock_irqrestore(&e->lock, flags);
		ret = -EAGAIN;
		goto err_free;
	}
	++chan->queued_cmds;
	p->gen = ++e->next_gen;
	if (p->cmd.flags & AL5_CMD_F_NO_KICK) {
		/* live register update: applied now, ahead of the queue */
		al5_apply_writes(codec, p->cmd.writes, p->cmd.num_writes);
		p->done.result = 0;
		al5_snapshot_status(codec, p);
		al5_deliver(e, p);
	} else if (slot->running || slot->paused ||
		   !list_empty(&slot->queue)) {
		list_add_tail(&p->node, &slot->queue);
	} else {
		al5_slot_start(slot, p);
	}
	spin_unlock_irqrestore(&e->lock, flags);
	return 0;

err_free:
	kfree(p);
	return ret;
}

long al5_cmd_ioctl_wait(struct al5r_codec_chan *chan, unsigned long arg,
			bool nonblock)
{
	struct al5_cmd_engine *e = chan->codec->engine;
	struct al5_pending_cmd *p = NULL;
	unsigned long flags;
	int ret;

	if (!e || !e->hw)
		return -ENOTTY;

	for (;;) {
		spin_lock_irqsave(&e->lock, flags);
		if (!list_empty(&chan->done_list)) {
			p = list_first_entry(&chan->done_list,
					     struct al5_pending_cmd, node);
			list_del(&p->node);
		}
		spin_unlock_irqrestore(&e->lock, flags);
		if (p)
			break;
		if (nonblock)
			return -EAGAIN;
		ret = wait_event_interruptible(chan->done_wait,
					!list_empty(&chan->done_list));
		if (ret)
			return ret;
	}

	ret = copy_to_user((void __user *)arg, &p->done, sizeof(p->done)) ?
		-EFAULT : 0;
	kfree(p);
	return ret;
}

__poll_t al5_cmd_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct al5r_codec_chan *chan = filp->private_data;

	poll_wait(filp, &chan->done_wait, wait);
	if (!list_empty(&chan->done_list))
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

long al5_cmd_ioctl_hw_info(struct al5r_codec_chan *chan, unsigned long arg)
{
	struct al5_cmd_engine *e = chan->codec->engine;
	struct al5_hw_info info = { 0 };
	unsigned long flags;
	int i;

	if (!e || !e->hw)
		return -ENOTTY;

	for (i = 0; i < AL5_MOD_COUNT; ++i) {
		info.num_cores[i] = e->hw->mods[i].num_cores;
		info.num_subs[i] = e->hw->mods[i].num_subs;
	}
	if (e->hw->has_mvd) {
		info.num_cores[AL5_MOD_MVD] = 1;
		info.num_subs[AL5_MOD_MVD] = 1;
	}

	spin_lock_irqsave(&e->lock, flags);
	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];
		struct al5_pending_cmd *p;

		if (slot->running)
			info.queue_depth[slot->module]++;
		list_for_each_entry(p, &slot->queue, node)
			info.queue_depth[slot->module]++;
	}
	spin_unlock_irqrestore(&e->lock, flags);

	info.uid = al5_rd(chan->codec, 0x0);
	info.arch = al5_rd(chan->codec, 0x4);
	info.date = al5_rd(chan->codec, 0x8);
	info.version = al5_rd(chan->codec, 0xC);

	if (copy_to_user((void __user *)arg, &info, sizeof(info)))
		return -EFAULT;
	return 0;
}

long al5_cmd_ioctl_load(struct al5r_codec_chan *chan, unsigned long arg)
{
	struct al5_cmd_engine *e = chan->codec->engine;
	struct al5_load load = { 0 };
	unsigned long flags;
	u64 now, elapsed;
	int i;

	if (!e || !e->hw)
		return -ENOTTY;

	spin_lock_irqsave(&e->lock, flags);
	now = ktime_get_ns();
	al5_load_fold(e, now);
	elapsed = now - e->load_win_start;
	if (!elapsed)
		elapsed = 1;
	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];
		struct al5_pending_cmd *p;
		u64 busy = slot->busy_ns;
		u32 sum;

		if (slot->busy_since)
			busy += now - slot->busy_since;
		if (slot->core < AL5_LOAD_MAX_CORES) {
			__u16 *cell =
				&load.busy_permille[slot->module][slot->core];

			sum = *cell;
			if (elapsed)
				sum += min_t(u64,
					     div64_u64(busy * 1000, elapsed),
					     1000);
			*cell = min(sum, 1000u);
			/* Count in 64 bits, then saturate: a __u16 cell must
			 * never wrap back to zero while work is outstanding.
			 */
			u64 queued = load.queued[slot->module][slot->core];

			if (slot->running)
				++queued;
			list_for_each_entry(p, &slot->queue, node)
				++queued;
			load.queued[slot->module][slot->core] =
				min_t(u64, queued, U16_MAX);
		}
	}
	spin_unlock_irqrestore(&e->lock, flags);

	if (copy_to_user((void __user *)arg, &load, sizeof(load)))
		return -EFAULT;
	return 0;
}

long al5_mvd_acquire(struct al5r_codec_chan *chan)
{
	struct al5r_codec_desc *codec = chan->codec;
	struct al5_cmd_engine *e = codec->engine;
	unsigned long flags;
	int ret;

	if (!e || !e->hw || !e->hw->has_mvd)
		return -ENOTTY;

	/* initializing the IP also puts the MVD cores in a known state */
	ret = al5_engine_ensure_ready(e);
	if (ret)
		return ret;

	mutex_lock(&e->owner_mutex);
	if (codec->mvd_chan && codec->mvd_chan != chan) {
		ret = -EBUSY;
	} else {
		spin_lock_irqsave(&codec->i_lock, flags);
		codec->mvd_chan = chan;
		chan->holds_mvd = true;
		spin_unlock_irqrestore(&codec->i_lock, flags);
		ret = 0;
	}
	mutex_unlock(&e->owner_mutex);
	return ret;
}

long al5_mvd_release(struct al5r_codec_chan *chan)
{
	struct al5r_codec_desc *codec = chan->codec;
	struct al5_cmd_engine *e = codec->engine;
	unsigned long flags;
	int ret = -EPERM;

	if (!e)
		return -ENOTTY;

	mutex_lock(&e->owner_mutex);
	if (codec->mvd_chan == chan) {
		spin_lock_irqsave(&codec->i_lock, flags);
		codec->mvd_chan = NULL;
		chan->holds_mvd = false;
		spin_unlock_irqrestore(&codec->i_lock, flags);
		ret = 0;
	}
	mutex_unlock(&e->owner_mutex);
	return ret;
}

static int al5_legacy_claim(struct al5r_codec_chan *chan)
{
	struct al5r_codec_desc *codec = chan->codec;
	struct al5_cmd_engine *e = codec->engine;
	unsigned long flags;
	int ret = 0;

	if (codec->legacy_chan == chan)
		return 0;

	mutex_lock(&e->owner_mutex);
	if (codec->legacy_chan && codec->legacy_chan != chan)
		ret = -EBUSY;
	else if (e->hw_ready || codec->mvd_chan)
		ret = -EBUSY;
	else {
		spin_lock_irqsave(&codec->i_lock, flags);
		codec->legacy_chan = chan;
		spin_unlock_irqrestore(&codec->i_lock, flags);
		dev_info(codec->device, "legacy exclusive owner claimed\n");
	}
	mutex_unlock(&e->owner_mutex);
	return ret;
}

int al5_legacy_reg_access(struct al5r_codec_chan *chan, u32 reg)
{
	struct al5r_codec_desc *codec = chan->codec;
	u32 rel = reg - codec->regs_min_addr;

	if (chan->holds_mvd) {
		if ((rel >= AL5_MVD_WIN_BASE && rel < AL5_MVD_WIN_END) ||
		    rel == AL5_TOP_RESET || rel == AL5_TOP_IRQ_EN ||
		    rel == AL5_TOP_IRQ || rel == AL5_TOP_AOM)
			return 0;
		return -EPERM;
	}
	return al5_legacy_claim(chan);
}

int al5_legacy_event_access(struct al5r_codec_chan *chan)
{
	if (chan->holds_mvd)
		return 0;
	return al5_legacy_claim(chan);
}

bool al5_mvd_shared_reg_write(struct al5r_codec_chan *chan, u32 rel, u32 val)
{
	struct al5r_codec_desc *codec = chan->codec;
	unsigned long flags;
	u32 v;

	if (!chan->holds_mvd)
		return false;

	switch (rel) {
	case AL5_TOP_IRQ_EN:
		/* Interrupt-enable register shared with the arbiter: keep the
		 * arbiter's bits and set only the MVD bits, read-modify-write
		 * under codec->i_lock -- the same lock the engine takes when it
		 * rewrites this register (preserving the MVD bits) at init.
		 */
		spin_lock_irqsave(&codec->i_lock, flags);
		v = al5_rd(codec, AL5_TOP_IRQ_EN);
		v = (v & ~AL5_MVD_IRQ_MASK) | (val & AL5_MVD_IRQ_MASK);
		al5_wr(codec, AL5_TOP_IRQ_EN, v);
		spin_unlock_irqrestore(&codec->i_lock, flags);
		return true;
	case AL5_TOP_IRQ:
		/* Write-1-to-clear: mask to the MVD bits so the holder cannot
		 * acknowledge the arbiter's pending interrupts.
		 */
		al5_wr(codec, AL5_TOP_IRQ, val & AL5_MVD_IRQ_MASK);
		return true;
	default:
		return false;
	}
}



void al5_cmd_chan_init(struct al5r_codec_chan *chan)
{
	INIT_LIST_HEAD(&chan->irq_masks);
	INIT_LIST_HEAD(&chan->done_list);
	init_waitqueue_head(&chan->done_wait);
}

void al5_cmd_chan_release(struct al5r_codec_chan *chan)
{
	struct al5r_codec_desc *codec = chan->codec;
	struct al5_cmd_engine *e = codec->engine;
	struct al5_pending_cmd *p, *pn;
	struct list_head *pos, *n;
	unsigned long flags;
	int i;

	if (!e)
		return;

	/* drop exclusive ownerships */
	mutex_lock(&e->owner_mutex);
	spin_lock_irqsave(&codec->i_lock, flags);
	if (codec->legacy_chan == chan)
		codec->legacy_chan = NULL;
	if (codec->mvd_chan == chan) {
		codec->mvd_chan = NULL;
		chan->holds_mvd = false;
	}
	spin_unlock_irqrestore(&codec->i_lock, flags);
	mutex_unlock(&e->owner_mutex);


	/* drop queued commands, orphan the in-flight ones */
	spin_lock_irqsave(&e->lock, flags);
	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];

		list_for_each_entry_safe(p, pn, &slot->queue, node) {
			if (p->chan != chan)
				continue;
			list_del(&p->node);
			kfree(p);
		}
		/* Orphan anything still on the hardware, and hand it a
		 * reference to the channel's buffer mappings: the IP may
		 * still be reading or writing them, so they outlive the
		 * channel and are released when the command retires.
		 */
		if (slot->running && slot->running->chan == chan) {
			slot->running->imports =
				al5_dmabuf_get_import_set(chan);
			slot->running->chan = NULL;
		}
		if (slot->recovering && slot->recovering->chan == chan) {
			slot->recovering->imports =
				al5_dmabuf_get_import_set(chan);
			slot->recovering->chan = NULL;
		}
	}
	list_for_each_entry_safe(p, pn, &chan->done_list, node) {
		list_del(&p->node);
		kfree(p);
	}
	/* Drain invariant: once close has run, nothing the hardware or
	 * recovery can still reach may point at this channel.  A command
	 * that outlives it must have been orphaned and given its own
	 * reference to the buffer mappings.
	 */
	for (i = 0; i < e->num_slots; ++i) {
		struct al5_slot *slot = &e->slots[i];

		WARN_ON(slot->running && slot->running->chan == chan);
		WARN_ON(slot->recovering && slot->recovering->chan == chan);
	}
	spin_unlock_irqrestore(&e->lock, flags);

	/* free pending legacy/MVD interrupt events */
	spin_lock_irqsave(&codec->i_lock, flags);
	list_for_each_safe(pos, n, &chan->irq_masks) {
		struct r_irq *cb = list_entry(pos, struct r_irq, list);

		list_del(pos);
		kmem_cache_free(codec->cache, cb);
	}
	spin_unlock_irqrestore(&codec->i_lock, flags);
}

int al5_cmd_engine_setup(struct al5r_codec_desc *codec,
			 struct device_node *np)
{
	const struct al5_hw_desc *hw = NULL;
	struct al5_cmd_engine *e;
	struct al5_slot *slot;
	const char *type;
	int m, core, sub;

	e = devm_kzalloc(codec->device, sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;

	e->codec = codec;
	spin_lock_init(&e->lock);
	mutex_init(&e->owner_mutex);
	init_waitqueue_head(&e->retire_wait);
	INIT_WORK(&e->recover_work, al5_recover_work);
	/* WQ_MEM_RECLAIM: hang recovery must make progress under memory
	 * pressure, so the queue needs its own rescuer thread.
	 */
	e->recover_wq = alloc_ordered_workqueue("al5r_recover/%s",
						WQ_MEM_RECLAIM,
						dev_name(codec->device));
	if (!e->recover_wq)
		return -ENOMEM;
	codec->engine = e;

	if (!np || of_property_read_string(np, "al,ip-type", &type))
		return 0;	/* no property: legacy-only device */

	if (!strcmp(type, al5_hw_enc_v200.type))
		hw = &al5_hw_enc_v200;
	else if (!strcmp(type, al5_hw_dec_v300.type))
		hw = &al5_hw_dec_v300;
	if (!hw) {
		dev_warn(codec->device,
			 "unknown al,ip-type \"%s\", command mode disabled\n",
			 type);
		return 0;
	}

	for (m = 0; m < AL5_MOD_COUNT; ++m)
		e->num_slots += hw->mods[m].num_cores * hw->mods[m].num_subs;

	e->slots = devm_kcalloc(codec->device, e->num_slots,
				sizeof(*e->slots), GFP_KERNEL);
	if (!e->slots)
		return -ENOMEM;

	slot = e->slots;
	for (m = 0; m < AL5_MOD_COUNT; ++m) {
		const struct al5_mod_desc *mod = &hw->mods[m];

		for (core = 0; core < mod->num_cores; ++core) {
			for (sub = 0; sub < mod->num_subs; ++sub, ++slot) {
				u32 phys = mod->first_phys_core + core;

				slot->engine = e;
				slot->mod = mod;
				slot->module = m;
				slot->core = core;
				slot->sub = sub;
				slot->base = hw->common_size +
					phys * hw->core_size + mod->bank +
					sub * mod->sub_stride;
				slot->irq_bit = mod->irq_base +
					mod->irq_core_shift * phys +
					mod->irq_sub_shift * sub;
				e->irq_mask |= BIT(slot->irq_bit);
				INIT_LIST_HEAD(&slot->queue);
				timer_setup(&slot->timer, al5_slot_timeout, 0);
			}
		}
	}
	e->hw = hw;

	dev_info(codec->device,
		 "command arbiter available: %s, %d slots, irq mask %#x\n",
		 hw->type, e->num_slots, e->irq_mask);
	return 0;
}

void al5_cmd_engine_teardown(struct al5r_codec_desc *codec)
{
	struct al5_cmd_engine *e = codec->engine;
	int i;

	if (!e)
		return;
	if (e->hw) {
		cancel_work_sync(&e->recover_work);
		for (i = 0; i < e->num_slots; ++i)
			timer_delete_sync(&e->slots[i].timer);
	}
	if (e->recover_wq) {
		destroy_workqueue(e->recover_wq);
		e->recover_wq = NULL;
	}
}
