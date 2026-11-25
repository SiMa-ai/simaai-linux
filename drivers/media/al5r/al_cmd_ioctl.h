/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (C) 2026 SiMa.ai
 *
 * Command-arbiter user API for the Allegro codec register driver (al5r).
 *
 * Each open() of the codec device is an independent channel.  A channel
 * submits self-contained register programs ("commands") for one hardware
 * sub-module (ENC1, DEC2, JPEG, ...).  The driver serializes commands from
 * all channels per sub-module, applies the register writes, kicks the module,
 * and completes the command back to its channel when the completion
 * interrupt fires, optionally with a snapshot of module status registers.
 *
 * Register offsets are the same device-relative offsets used with
 * AL_CMD_IP_READ_REG/AL_CMD_IP_WRITE_REG.  A command may only touch its
 * target sub-module's register window; global registers (reset, clock
 * gating, interrupt enable, watchdog) are owned by the kernel.
 */

#pragma once

#include <linux/types.h>

/* Sub-module classes.  Availability depends on the device (encoder or
 * decoder); AL5_GET_HW_INFO reports the number of cores of each class.
 */
enum al5_mod_class {
	AL5_MOD_ENC1 = 0,	/* video encoder, reconstruction stage */
	AL5_MOD_ENC2,		/* video encoder, entropy stage */
	AL5_MOD_ENC_JPEG,	/* JPEG encoder */
	AL5_MOD_SCD,		/* start-code detector */
	AL5_MOD_DEC1,		/* video decoder, parser (sub = parser index) */
	AL5_MOD_DEC2,		/* video decoder, pixel stage */
	AL5_MOD_DEC_JPEG,	/* JPEG decoder */
	AL5_MOD_MVD,		/* AV1/VP9 engine: exclusive hold only */
	AL5_MOD_COUNT
};

/* Bounds on al5_cmd.timeout_ms (0 still means "driver default"). */
#define AL5_MIN_TIMEOUT_MS	10u
#define AL5_MAX_TIMEOUT_MS	60000u

#define AL5_CMD_MAX_WRITES	64
#define AL5_CMD_MAX_STATUS	32

/* Apply (value & mask) | (reg & ~mask); mask == 0xffffffff is a plain
 * write.  Masked writes are atomic with respect to other channels.
 */
struct al5_reg_write {
	__u32 reg;
	__u32 value;
	__u32 mask;
};

/* Command has no kick and completes as soon as its writes are applied.
 * Intended for live synchronization registers (e.g. entropy sync bits).
 */
#define AL5_CMD_F_NO_KICK	(1u << 0)

/* Reset the sub-module (with the required clock sequence and settle time)
 * before applying the command's writes.
 */
#define AL5_CMD_F_RESET_MODULE	(1u << 1)

struct al5_cmd {
	__u32 module;		/* enum al5_mod_class */
	__u32 core;		/* core index within the class */
	__u32 sub;		/* sub-unit (DEC1 parser index), else 0 */
	__u32 flags;		/* AL5_CMD_F_* */
	__u32 num_writes;
	__u32 num_status;
	/* Completion timeout.  0 selects the driver default; any other value
	 * must be within [AL5_MIN_TIMEOUT_MS, AL5_MAX_TIMEOUT_MS] or the
	 * submission is rejected with -EINVAL.  The bound matters because a
	 * running command holds its execution slot, and close waits for it.
	 */
	__u32 timeout_ms;
	__u32 reserved;		/* must be 0 */
	__u64 user_data;	/* returned in al5_cmd_done */
	struct al5_reg_write writes[AL5_CMD_MAX_WRITES];
	struct al5_reg_write kick;	/* applied last */
	__u32 status_regs[AL5_CMD_MAX_STATUS];	/* read back on completion */
};

struct al5_cmd_done {
	__u64 user_data;
	__s32 result;		/* 0, -ETIMEDOUT (module reset), -EIO (IP reset) */
	__u32 core;
	__u32 status_values[AL5_CMD_MAX_STATUS];
};

struct al5_hw_info {
	__u32 num_cores[AL5_MOD_COUNT];
	__u32 num_subs[AL5_MOD_COUNT];	/* sub-units per core (DEC1 parsers) */
	__u32 queue_depth[AL5_MOD_COUNT]; /* currently queued + running */
	__u32 arch;		/* architecture/HW-ID register snapshot */
	__u32 uid;		/* IP identification register */
	__u32 date;		/* IP date register */
	__u32 version;		/* IP version register */
	__u32 reserved[4];
};

/* Enqueue a command; returns immediately.  Completions are delivered in
 * per-channel order via AL5_CMD_WAIT (the fd is also poll()able).
 */
#define AL5_CMD_SUBMIT		_IOW('q', 20, struct al5_cmd)
#define AL5_CMD_WAIT		_IOR('q', 21, struct al5_cmd_done)
#define AL5_GET_HW_INFO		_IOR('q', 22, struct al5_hw_info)

/* Exclusive hold of the MVD (AV1/VP9) engine.  While held, this channel
 * may use AL_CMD_IP_READ_REG/WRITE_REG/WAIT_IRQ restricted to the MVD
 * register window and the MVD bits of the shared interrupt registers.
 * Released explicitly or on close.
 */
#define AL5_MVD_ACQUIRE		_IO('q', 23)
#define AL5_MVD_RELEASE		_IO('q', 24)

#define AL5_LOAD_MAX_CORES	4

/* Cross-process hardware load, for channel placement.  Unlike
 * AL5_GET_HW_INFO this is a live query: schedulers ask right before
 * binding a new channel to a core.
 */
struct al5_load {
	/* Busy time of each (module, core) execution unit as per-mille of
	 * wall time over the driver's decaying sampling window (~1s).
	 * Sub-unit slots (DEC1 parsers) are summed and clamped to 1000.
	 */
	__u16 busy_permille[AL5_MOD_COUNT][AL5_LOAD_MAX_CORES];
	/* Commands queued plus running right now, saturating at U16_MAX.
	 * The admission limit below keeps the true count well inside that,
	 * so saturation is a backstop, not an expected value.  A consumer
	 * that needs an exact count must not rely on this field alone.
	 */
	__u16 queued[AL5_MOD_COUNT][AL5_LOAD_MAX_CORES];
	__u32 reserved[8];
};

#define AL5_GET_LOAD		_IOR('q', 25, struct al5_load)

/* Admission limit for AL5_CMD_SUBMIT, counted per open channel (per file
 * descriptor), not per device and not per module.  A submit that would take
 * a channel past this many commands queued-plus-running is rejected with
 * -EAGAIN; the caller should reap completions with AL5_CMD_WAIT and retry.
 * The driver counts in 64 bits internally, so the count itself cannot wrap.
 */
#define AL5_MAX_CHAN_QUEUED	4096u
