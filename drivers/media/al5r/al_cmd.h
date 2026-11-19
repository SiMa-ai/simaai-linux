/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 SiMa.ai
 *
 * Codec-agnostic command arbiter for the Allegro register IP.
 * See al_cmd_ioctl.h for the user API and semantics.
 */

#pragma once

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>

#include "al_cmd_ioctl.h"

struct al5r_codec_desc;
struct al5r_codec_chan;

/* MVD (AV1/VP9) engine bits in the shared TOP interrupt registers */
#define AL5_MVD_IRQ_MASK	(0xFu << 22)

/* Geometry of one sub-module class, offsets relative to the IP base
 * (i.e. on top of regs_min_addr).
 */
struct al5_mod_desc {
	const char *name;
	u8 num_cores;
	u8 num_subs;		/* sub-units per core (DEC1 parsers); 0 = absent */
	u8 first_phys_core;	/* encoder JPEG cores follow the video cores */
	u16 bank;		/* window offset within the core region */
	u16 span;		/* window size */
	u16 sub_stride;		/* window stride per sub-unit */
	u8 irq_base;		/* completion IRQ bit, plus ... */
	u8 irq_core_shift;	/* ... this per physical core, plus ... */
	u8 irq_sub_shift;	/* ... this per sub-unit */
};

struct al5_hw_desc {
	const char *type;	/* matches the al,ip-type DT property */
	u32 common_size;	/* TOP bank size; core 0 region starts here */
	u32 core_size;
	u32 axi_burst_reg;	/* clear its low bits at init (SiMa NoC
				 * needs 256-byte AXI bursts); 0 = none
				 */
	bool has_mvd;
	struct al5_mod_desc mods[AL5_MOD_COUNT];
};

struct al5_pending_cmd {
	struct list_head node;	/* slot queue, then channel done list */
	struct al5r_codec_chan *chan;	/* NULL once the channel is gone */
	/* Immutable identity assigned at admission, never zero.  A slot is
	 * reused, so (slot, cookie) does not identify a command; this does.
	 */
	u64 gen;
	/* Set only when the owning channel closed while this command was
	 * still on the hardware: keeps its buffer mappings alive until it
	 * retires.  NULL in the normal case.
	 */
	struct al5_import_set *imports;
	struct al5_cmd cmd;
	struct al5_cmd_done done;
};

/* One execution unit: (module class, core, sub-unit). */
struct al5_slot {
	const struct al5_mod_desc *mod;
	struct al5_cmd_engine *engine;
	u32 module;		/* enum al5_mod_class */
	u32 core;
	u32 sub;
	u32 base;		/* absolute window offset */
	u32 irq_bit;
	struct list_head queue;
	struct al5_pending_cmd *running;
	/* Detached by recovery: no longer "running", but still owned by the
	 * driver and still referencing its channel.  Close must be able to
	 * find it here, or it delivers into freed channel state.
	 */
	struct al5_pending_cmd *recovering;
	unsigned long deadline;
	bool timed_out;		/* recovery requested */
	u64 timeout_gen;	/* generation that requested it; 0 = none */
	bool paused;		/* held quiescent during recovery */
	struct timer_list timer;
	u64 busy_ns;		/* accumulated in the current load window */
	u64 busy_since;		/* start of the running command, 0 if idle */
};

struct al5_cmd_engine {
	const struct al5_hw_desc *hw;	/* NULL: command mode unavailable */
	struct al5r_codec_desc *codec;
	spinlock_t lock;		/* slots, queues, completions */
	struct mutex owner_mutex;	/* hw init, legacy/MVD ownership */
	struct al5_slot *slots;
	int num_slots;
	u32 irq_mask;			/* IRQ bits owned by the engine */
	bool hw_ready;			/* engine initialized the IP */
	bool need_global_reset;
	/* Woken whenever a command retires, so close can wait for hardware to
	 * stop touching the buffers a closing channel owns.
	 */
	wait_queue_head_t retire_wait;
	struct work_struct recover_work;
	/* Recovery runs a module reset with settle delays, and under the QA
	 * hooks it can block on a checkpoint.  Keep it off the shared system
	 * workqueue.  Ordered: one recovery at a time per device.
	 */
	struct workqueue_struct *recover_wq;
	u64 load_win_start;		/* load sampling window epoch */
	u64 next_gen;			/* monotonic command generation */
	u64 stale_recovery;		/* recoveries discarded as retired */
};

int al5_cmd_engine_setup(struct al5r_codec_desc *codec,
			 struct device_node *np);
void al5_cmd_engine_teardown(struct al5r_codec_desc *codec);

/* Called from the hard IRQ handler for each pending interrupt bit.
 * Returns true when the bit was consumed by the command engine.
 */
bool al5_cmd_engine_irq(struct al5r_codec_desc *codec, int bit);

void al5_cmd_chan_init(struct al5r_codec_chan *chan);
void al5_cmd_chan_release(struct al5r_codec_chan *chan);

long al5_cmd_ioctl_submit(struct al5r_codec_chan *chan, unsigned long arg);
long al5_cmd_ioctl_wait(struct al5r_codec_chan *chan, unsigned long arg,
			bool nonblock);
long al5_cmd_ioctl_hw_info(struct al5r_codec_chan *chan, unsigned long arg);
long al5_cmd_ioctl_load(struct al5r_codec_chan *chan, unsigned long arg);
long al5_mvd_acquire(struct al5r_codec_chan *chan);
long al5_mvd_release(struct al5r_codec_chan *chan);
__poll_t al5_cmd_poll(struct file *filp, struct poll_table_struct *wait);

/* Legacy raw register interface ownership.  The first legacy register/IRQ
 * ioctl claims exclusive ownership of the device; this fails once the
 * command engine has initialized the hardware (and vice versa).  An MVD
 * holder gets register access restricted to the MVD window.
 */
int al5_legacy_reg_access(struct al5r_codec_chan *chan, u32 reg);
int al5_legacy_event_access(struct al5r_codec_chan *chan);

/* Apply an MVD holder's write to one of the shared TOP interrupt
 * registers, masked to the MVD bits so the arbiter's enc/dec/jpeg bits
 * survive.  Returns true when the write was handled here (the caller
 * must then skip the raw register write); false for any other register.
 */
bool al5_mvd_shared_reg_write(struct al5r_codec_chan *chan, u32 rel, u32 val);

/* Exposed for unit testing: recovery must be discarded when the slot no
 * longer holds the generation that asked for it.
 */
bool al5_recovery_is_stale(const struct al5_slot *slot);

