/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2021 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

/*
 * Per-ctx calibration-storage bridge.
 *
 * Originally this file backed the /dev/isp_control_<N> char devices the
 * IPA used for MODALIX_ISP_IOC_SET_CALIBRATION pushes and a
 * kernel→user-space command/event RPC channel. Both roles moved off
 * the chardev:
 *
 *   - calibration push is now a VIDIOC_S_EXT_CTRLS on the per-ctx
 *     meta-stats node (MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB → s_ctrl
 *     → modalix_isp_install_user_calibrations()),
 *   - the kernel→IPA command/event dispatch (ctrl_channel_handle_api_*)
 *     was never wired to an in-tree caller — grep showed zero kernel
 *     producers — so it's deleted outright.
 *
 * What remains here:
 *   - the per-ctx slot the firmware's calib_mgr read path consumes
 *     (struct ctrl_channel_dev_context.user_calibrations);
 *   - the cbs[]/custom_get_calib_ctxN dispatch the framework wires
 *     into acamera_settings.get_calibrations at boot — the dispatcher
 *     prefers the user-pushed blob and falls back to the old kernel-
 *     baked get_calibrations_*() helper if no blob has been pushed
 *     yet (a later change retires that fallback);
 *   - the shared modalix_isp_install_user_calibrations() helper that
 *     the meta-stats V4L2 ctrl handler calls.
 *
 * The .c name is preserved to minimise Makefile churn — the file now
 * acts as a small "calib bridge" connecting the meta-stats V4L2
 * control to the firmware's calibration read path. No chardev is
 * registered.
 */

#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include "system_stdlib.h"
#include "acamera_ctrl_channel.h"
#include "acamera_logger.h"
#include "acamera_calib_mgr.h"
#include "acamera_calib_mgr_slot_types.h"
#include "acamera_isp_config.h" /* system_isp_read_32/_write_32, reg 0xe040 */
#include "acamera_isp_ctx.h"    /* acamera_settings */
#include <uapi/linux/media/simaai/modalix_isp_v4l2.h> /* WDR layer limits, sensor modes */

/* ISP pipeline bypass register (Mali "Pipeline" group).
 *
 * Bits 0, 1, 23, 25 and 26 are reserved and keep their power-on value.
 * Bits 2 and 30 - the input formatter and its linear-data-source mux - are
 * held back for a different reason: the decompander FSM owns them and
 * programs them from MODALIX_ISP_CALIB_DECOMPANDER_CONTROL. The IPA resolves
 * both from the same calibration, so letting the generic word write them too
 * would only make the result depend on which arrived last. */
#define MODALIX_ISP_PIPELINE_BYPASS_OFFSET   0xe040u
#define MODALIX_ISP_PIPELINE_BYPASS_RESERVED \
	( BIT(0) | BIT(1) | BIT(2) | BIT(23) | BIT(25) | BIT(26) | BIT(30) )

extern acamera_settings *get_settings_by_id( u8 ctx_id );

/* How long the FSM read path waits for the IPA to push the calibration
 * blob before giving up with -ETIMEDOUT. The IPA pushes during
 * IPAModalix::init() (well before any stream is started), so the wait
 * normally returns immediately. The timeout exists so a missing or
 * broken IPA surfaces a clean errno instead of hanging the FSM thread
 * forever. */
#define CALIB_PUSH_WAIT_MS  5000u

struct ctrl_channel_dev_context {
    uint8_t              dev_inited;
    uint8_t              ctx_id;
    struct mutex         lock;
    wait_queue_head_t    wq;
    bool                 calib_ready;
    bool                 shutting_down;
    /* The effective calibration for the mode the IPA has selected. The IPA
     * resolves its tuning file's base and per-mode overrides itself and
     * pushes the result, so there is exactly one set here and nothing to
     * choose between. */
    ACameraCalibrations *calibrations;
    /* The set the calibration manager last copied slot pointers out of, if
     * it is not @calibrations any more.
     *
     * The manager keeps those pointers until its next update, so a replaced
     * set cannot be freed at the moment it is replaced. It can be freed as
     * soon as the manager has taken pointers from a newer one, which is
     * what the next hand-out does - and until then it is exactly one set,
     * so a userspace that pushes in a loop cannot grow this without bound.
     *
     * One store serves one ISP context, which has one calibration manager
     * entry, so "the last hand-out" is unambiguous. */
    ACameraCalibrations *borrowed;
    /*
     * The set published one cycle before `borrowed`. The manager copies the
     * overlay into its slots only after the hand-out callback returns, so the
     * set its slots still point into cannot be freed there; it is retired on
     * the next update, by which time a newer set has been published.
     */
    ACameraCalibrations *retiring;
    /* Callbacks currently inside the hand-out path. Teardown waits for
     * these to leave before the firmware contexts are torn down, so a
     * callback cannot still be filling a calibration manager entry that
     * acamera_deinit() has already reset. */
    unsigned int         in_flight;
};

static struct ctrl_channel_dev_context ctrl_channel_ctx[FIRMWARE_CONTEXT_NUMBER];

/* Block until the IPA has pushed a calibration blob for @p_ctx (or
 * until the wait times out). Returns 0 when calibrations are ready,
 * -ETIMEDOUT if the IPA never pushed inside CALIB_PUSH_WAIT_MS, or
 * -ERESTARTSYS if interrupted. Safe to call from a kernel thread. */
static int wait_for_calibrations( struct ctrl_channel_dev_context *p_ctx )
{
    long ret;

    if ( READ_ONCE( p_ctx->shutting_down ) )
        return -ENODEV;
    if ( READ_ONCE( p_ctx->calib_ready ) )
        return 0;

    /* Teardown has to end the wait as well, or a waiter woken by deinit
     * finds its predicate still false and sleeps again until the timeout. */
    ret = wait_event_interruptible_timeout(
        p_ctx->wq,
        READ_ONCE( p_ctx->calib_ready ) || READ_ONCE( p_ctx->shutting_down ),
        msecs_to_jiffies( CALIB_PUSH_WAIT_MS ) );

    if ( ret == 0 ) {
        LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR, "ctx %u: timed out waiting for IPA to push calibrations",
             p_ctx->ctx_id );
        return -ETIMEDOUT;
    }
    if ( ret < 0 ) {
        LOG_CAT( ALOG_CAT_CONTROL, LOG_INFO, "ctx %u: interrupted while waiting for calibration push",
             p_ctx->ctx_id );
        return -ERESTARTSYS;
    }
    if ( READ_ONCE( p_ctx->shutting_down ) ) {
        return -ENODEV;
    }
    return 0;
}

/* What the kernel expects of slot @idx, or NULL if nothing reads it. */
static const struct modalix_calib_slot_type *calib_slot_type( unsigned int idx )
{
    unsigned int i;

    for ( i = 0; i < ARRAY_SIZE( modalix_calib_slot_types ); i++ ) {
        if ( modalix_calib_slot_types[i].slot == idx ) {
            return &modalix_calib_slot_types[i];
        }
    }
    return NULL;
}

/* Is a LUT descriptor's shape one the calibration manager can serve?
 *
 * Bounds alone are not enough. Consumers size their reads in elements -
 * calib_mgr_lut_len() is rows * cols and the typed getters multiply by the
 * width they expect - so a descriptor whose declared extent is zero passes
 * every bounds check and still sends an FSM walking off the end. A zero
 * width is the sharp edge: rows * cols * 0 is nothing at all, whatever
 * rows and cols claim.
 *
 * The IPA emits a descriptor only for a slot it fills, so "present but
 * empty" is not a thing on this wire - absent is calibrations[i] == 0.
 *
 * The width also has to be the one the FSM that reads this slot will use.
 * Checking that here rather than at the getter is what lets the whole blob
 * be refused as a unit: a getter can only decline the slot it was asked
 * for, and its caller has already been told by calib_mgr_lut_exists() that
 * the slot is there. Nothing downstream then has to cope with a LUT that
 * exists but cannot be read.
 *
 * Rows are deliberately not policed. A mode legitimately reshapes the
 * modulation tables - the FSM walks however many rows the descriptor
 * declares - so the column count is the part that has to hold for them.
 */
static bool calib_lut_shape_ok( unsigned int idx, const LookupTable *t )
{
    const struct modalix_calib_slot_type *type;
    uint32_t elems;

    if ( t->_reserved ) {
        return false;
    }
    if ( !t->ptr || !t->rows || !t->cols ) {
        return false;
    }
    /* the only element widths calib_mgr's typed getters cast to */
    if ( t->width != 1 && t->width != 2 && t->width != 4 ) {
        return false;
    }

    type = calib_slot_type( idx );
    if ( !type ) {
        /* no kernel consumer, so no width to disagree with */
        return true;
    }

    switch ( t->width ) {
    case 1:
        if ( !( type->flags & MODALIX_CALIB_SLOT_W1 ) )
            return false;
        break;
    case 2:
        if ( !( type->flags & MODALIX_CALIB_SLOT_W2 ) )
            return false;
        break;
    case 4:
        if ( !( type->flags & MODALIX_CALIB_SLOT_W4 ) )
            return false;
        break;
    }

    /* modulation entries are (x, y) pairs, one per row */
    if ( ( type->flags & MODALIX_CALIB_SLOT_MOD ) && t->cols != 2 ) {
        return false;
    }

    /* A consumer that reads a fixed position needs the LUT to reach it.
     * rows * cols is bounded by the blob, so this cannot overflow. */
    elems = (uint32_t)t->rows * t->cols;
    if ( elems < type->min_elems ) {
        return false;
    }

    /* Custom settings are walked in groups of four words, so a partial
     * group would leave the last read reaching past the LUT. */
    if ( ( type->flags & MODALIX_CALIB_SLOT_QUAD ) && ( elems % 4u ) ) {
        return false;
    }

    /* Some LUTs are copied straight into a fixed hardware register array,
     * indexed by the descriptor's own length. Too long does not over-read
     * the blob - it writes past the array into the registers beyond it. */
    if ( type->max_elems && elems > type->max_elems ) {
        return false;
    }

    return true;
}

/* ca_correction line offset is a 13-bit register field. */
#define CAC_MESH_DIM_MAX 0x1fffu

/**
 * calib_lut_values_ok() - check what a slot's contents mean, not just their
 * shape. Runs once @t->ptr is a bounded kernel pointer, because these read
 * the data.
 *
 * Shape is not the whole contract: a slot can be perfectly well-formed and
 * still carry a value the kernel then divides by.
 */
static bool calib_lut_values_ok( unsigned int idx, const LookupTable *t )
{
    const uint32_t elems = (uint32_t)t->rows * t->cols;

    /* The CAC mesh width and height divide the frame width, so a zero
     * survives every shape check and then divides by zero in the kernel.
     * The minimum correction, element 0, is legitimately zero. The upper
     * bound is the ca_correction line-offset register field, which is 13
     * bits: a mesh the hardware cannot address is not a mesh. */
    if ( idx == MODALIX_ISP_CALIB_CA_CORRECTION ) {
        const uint16_t *cfg = (const uint16_t *)(uintptr_t)t->ptr;

        if ( elems < 3 || !cfg[1] || !cfg[2] ||
             cfg[1] > CAC_MESH_DIM_MAX || cfg[2] > CAC_MESH_DIM_MAX ) {
            return false;
        }
    }

    return true;
}

/**
 * copy_and_patch_blob() - take a private copy of a wire blob and turn its
 * offsets into kernel pointers.
 *
 * The blob crosses the V4L2 boundary so it is untrusted: every descriptor is
 * checked for a shape a consumer can actually read, every offset is
 * bounds-checked against @size, and a malformed blob is rejected rather than
 * dereferenced. Wire form holds a byte offset in each __aligned_u64 slot;
 * after patching the same storage holds a real kernel pointer.
 *
 * Shared by the base document and every WDR layer - they use one encoding,
 * and a layer differs only in how many slots it leaves at zero.
 *
 * Return: the patched copy (caller owns it), or an ERR_PTR.
 */
static ACameraCalibrations *copy_and_patch_blob( const void *data, size_t size )
{
    ACameraCalibrations *kcalibs;
    const size_t         hdr = sizeof( *kcalibs );
    int                  i;
    int                  bad_slot = -1;

    if ( size < hdr || size > MODALIX_ISP_V4L2_CALIBRATION_BLOB_MAX ) {
        return ERR_PTR( -EINVAL );
    }

    kcalibs = kmalloc( size, GFP_KERNEL );
    if ( !kcalibs ) {
        return ERR_PTR( -ENOMEM );
    }
    memcpy( kcalibs, data, size );

    for ( i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++ ) {
        uintptr_t    table_offset = (uintptr_t)kcalibs->calibrations[i];
        uintptr_t    data_offset;
        size_t       data_bytes;
        LookupTable *table;

        /* zero means "this slot is not provided" - the whole basis of a
         * sparse WDR layer, and already true of a partial base document */
        if ( !table_offset ) {
            continue;
        }

        /* descriptor must sit after the root header, be 8-byte aligned,
         * and fit wholly inside the blob */
        if ( table_offset < hdr || ( table_offset & 0x7 ) ||
             table_offset > size - sizeof( LookupTable ) ) {
            bad_slot = i;
            goto bad_blob;
        }

        table = (LookupTable *)( (uint8_t *)kcalibs + table_offset );

        if ( !calib_lut_shape_ok( i, table ) ) {
            bad_slot = i;
            goto bad_blob;
        }

        data_offset = (uintptr_t)table->ptr;
        /* saturating, so a shape that would overflow size_t on a 32-bit
         * build fails the bounds test below rather than wrapping past it */
        data_bytes  = size_mul( size_mul( table->rows, table->cols ), table->width );

        /* LUT data must lie after the header, be aligned for its element
         * width, and fit wholly in the blob */
        if ( data_offset < hdr || data_offset > size ||
             ( data_offset & ( table->width - 1 ) ) ||
             data_bytes > size - data_offset ) {
            bad_slot = i;
            goto bad_blob;
        }
    }

    /*
     * Nothing has been written yet, and nothing may be until the layout is
     * known to be disjoint. Patching rewrites a descriptor's ptr in place,
     * so a payload covering another slot's descriptor would change bytes
     * that slot was already validated on - which is how a blob gets past
     * the value checks. Two slots sharing descriptor bytes do the same.
     */
    for ( i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++ ) {
        uintptr_t    off_i = (uintptr_t)kcalibs->calibrations[i];
        uintptr_t    data_i;
        size_t       bytes_i;
        LookupTable *ti;
        int          j;

        if ( !off_i ) {
            continue;
        }

        ti      = (LookupTable *)( (uint8_t *)kcalibs + off_i );
        data_i  = (uintptr_t)ti->ptr;
        bytes_i = size_mul( size_mul( ti->rows, ti->cols ), ti->width );

        for ( j = 0; j < MODALIX_ISP_CALIB_TOTAL_SIZE; j++ ) {
            uintptr_t off_j = (uintptr_t)kcalibs->calibrations[j];

            if ( !off_j ) {
                continue;
            }

            /* Both ends are bounded by size above, so neither sum wraps. */
            if ( bytes_i && off_j < data_i + bytes_i &&
                 data_i < off_j + sizeof( LookupTable ) ) {
                bad_slot = i;
                goto bad_blob;
            }

            if ( j > i && off_j < off_i + sizeof( LookupTable ) &&
                 off_i < off_j + sizeof( LookupTable ) ) {
                bad_slot = i;
                goto bad_blob;
            }
        }
    }

    /* Disjoint and bounded, so patching cannot disturb anything read. */
    for ( i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++ ) {
        uintptr_t    table_offset = (uintptr_t)kcalibs->calibrations[i];
        LookupTable *table;

        if ( !table_offset ) {
            continue;
        }

        table = (LookupTable *)( (uint8_t *)kcalibs + table_offset );
        kcalibs->calibrations[i] = (__u64)(uintptr_t)table;
        table->ptr = (__u64)(uintptr_t)( (uint8_t *)kcalibs + (uintptr_t)table->ptr );
    }

    /* Values read through the patched pointers, on bytes nothing can still
     * rewrite. */
    for ( i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++ ) {
        LookupTable *table;

        if ( !kcalibs->calibrations[i] ) {
            continue;
        }

        table = (LookupTable *)(uintptr_t)kcalibs->calibrations[i];
        if ( !calib_lut_values_ok( i, table ) ) {
            bad_slot = i;
            goto bad_blob;
        }
    }

    /* Everything the kernel reads unguarded has to be here. A blob that omits
     * one is not merely incomplete: the FSM that reads it has no way to ask
     * whether it arrived. */
    for ( i = 0; i < (int)ARRAY_SIZE( modalix_calib_slot_types ); i++ ) {
        if ( !( modalix_calib_slot_types[i].flags & MODALIX_CALIB_SLOT_REQ ) ) {
            continue;
        }
        if ( kcalibs->calibrations[modalix_calib_slot_types[i].slot] == 0 ) {
            LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR,
                 "calibration blob omits slot %u, which the kernel reads unconditionally",
                 modalix_calib_slot_types[i].slot );
            kfree( kcalibs );
            return ERR_PTR( -EINVAL );
        }
    }

    return kcalibs;

bad_blob:
    /* The root is a dense array of slots, so its size is part of the wire
     * format: a producer built against a different MODALIX_ISP_CALIB_TOTAL_SIZE
     * places every descriptor somewhere this kernel does not look. libcamera
     * and the kernel ship together, so that is a deployment mistake rather
     * than a case to accommodate - say so alongside the slot. */
    LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR,
         "calibration slot %d is malformed or is not the shape the kernel reads; rejecting the blob. If the whole blob is from a different release, libcamera and the kernel must be deployed together",
         bad_slot );
    kfree( kcalibs );
    return ERR_PTR( -EINVAL );
}

/* Copy every slot @src actually provides over @dst. Absent slots keep
 * whatever the previous pass left, which is what makes a layer sparse. */
static void overlay_calibrations( ACameraCalibrations *dst,
                                  const ACameraCalibrations *src )
{
    int i;

    for ( i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++ ) {
        const LookupTable *lut = (const LookupTable *)(uintptr_t)src->calibrations[i];

        if ( src->calibrations[i] != 0 && lut != NULL && lut->ptr != 0 ) {
            dst->calibrations[i] = src->calibrations[i];
        }
    }
}

#define REPEAT_16(x) x(0) x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10) x(11) x(12) x(13) x(14) x(15)
#define ASSIGN_CALIB_CALLBACK(CTX) custom_get_calib_ctx##CTX,
#define DEFINE_CALIB_CALLBACK(CTX) \
static int32_t custom_get_calib_ctx##CTX(uint32_t wdr_mode, void *param) { \
	struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[CTX]; \
	ACameraCalibrations *c, *user_c; \
	int rc; \
	if (param == NULL) \
		return -EINVAL; \
	rc = wait_for_calibrations(p_ctx); \
	if (rc) \
		return rc; \
	mutex_lock(&p_ctx->lock); \
	/* Teardown may have started while we were waiting; the flag is set \
	 * under this lock, so checking it here is the last word. */ \
	if (p_ctx->shutting_down) { \
		mutex_unlock(&p_ctx->lock); \
		return -ENODEV; \
	} \
	user_c = p_ctx->calibrations; \
	if (user_c == NULL) { \
		mutex_unlock(&p_ctx->lock); \
		return -EAGAIN; \
	} \
	p_ctx->in_flight++; \
	c = (ACameraCalibrations *)param; \
	overlay_calibrations(c, user_c); \
	/* The manager publishes by copying the overlay into its slots AFTER \
	 * this returns, so until then those slots still point into the set it \
	 * borrowed last. Free only what was retired a cycle ago - nothing \
	 * points there any more - and hand the current one on to be retired \
	 * by the next update. */ \
	if (p_ctx->retiring != NULL && p_ctx->retiring != user_c && \
	    p_ctx->retiring != p_ctx->borrowed) \
		kfree(p_ctx->retiring); \
	p_ctx->retiring = p_ctx->borrowed; \
	p_ctx->borrowed = user_c; \
	p_ctx->in_flight--; \
	mutex_unlock(&p_ctx->lock); \
	wake_up_interruptible(&p_ctx->wq); \
	LOG_CAT( ALOG_CAT_CONTROL, LOG_INFO, "ctx %d: applied the IPA's calibration (WDR mode %u)", \
		 CTX, wdr_mode); \
	return 0; \
}

REPEAT_16(DEFINE_CALIB_CALLBACK)

int32_t (*cbs[])( uint32_t wdr_mode, void *param ) = {
    REPEAT_16(ASSIGN_CALIB_CALLBACK)
};

/**
 * modalix_isp_install_user_calibrations() - V4L2-path entry point used
 * by the meta-stats node's s_ctrl handler when the IPA writes
 * MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB. Takes a copy of @data, patches
 * the offset-form __aligned_u64 fields to kernel pointers in place,
 * and atomically swaps the result into the per-ctx slot the
 * cbs[]/custom_get_calib_ctxN dispatcher reads from. Returns 0 on
 * success or a negative errno.
 *
 * The data is already in kernel memory when this is called (V4L2
 * has done the user-space copy), so no copy_from_user is needed.
 */
int modalix_isp_install_user_calibrations( uint32_t ctx_id,
                                           const void *data,
                                           size_t      size )
{
    struct ctrl_channel_dev_context *p_ctx;
    ACameraCalibrations             *kcalibs, *old;

    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER || data == NULL || size == 0 ) {
        return -EINVAL;
    }

    p_ctx = &ctrl_channel_ctx[ctx_id];
    if ( !p_ctx->dev_inited ) {
        return -EAGAIN;
    }

    kcalibs = copy_and_patch_blob( data, size );
    if ( IS_ERR( kcalibs ) ) {
        LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR, "ctx %u: rejected malformed calibration blob (%zu bytes)",
             ctx_id, size );
        return PTR_ERR( kcalibs );
    }

    mutex_lock( &p_ctx->lock );
    if ( !p_ctx->dev_inited || p_ctx->shutting_down ) {
        mutex_unlock( &p_ctx->lock );
        kfree( kcalibs );
        return -EAGAIN;
    }
    /* The outgoing set can go now unless the calibration manager is still
     * holding pointers into it - either as the set it was last handed
     * (`borrowed`) or as the one awaiting retirement (`retiring`, freed by
     * the next hand-out once something newer has been published). */
    old = p_ctx->calibrations;
    p_ctx->calibrations = kcalibs;
    if ( old != NULL && old != p_ctx->borrowed && old != p_ctx->retiring ) {
        kfree( old );
    }
    WRITE_ONCE( p_ctx->calib_ready, true );
    mutex_unlock( &p_ctx->lock );

    /* Wake any FSM thread that hit wait_for_calibrations before this
     * push arrived. The kernel no longer has a baked-in
     * default to fall back to, so a missing push would otherwise hang
     * the read path. */
    wake_up_interruptible( &p_ctx->wq );

    LOG_CAT( ALOG_CAT_CONTROL, LOG_DEBUG, "ctx %u: V4L2 install of calibration blob (%zu bytes)",
         ctx_id, size );
    return 0;
}
EXPORT_SYMBOL_GPL( modalix_isp_install_user_calibrations );


/* Apply the IPA's ISP pipeline bypass word to context @ctx_id's config
 * space. Masked RMW so reserved bits keep their power-on value. The
 * context's isp_base is the per-context CDMA config slot — the same base
 * the ISP sensor drivers and FSMs use for pipeline writes. */
int modalix_isp_apply_pipeline_bypass( uint32_t ctx_id, uint32_t bypass_word )
{
	acamera_settings *s;
	uint32_t          base, cur;

	if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER ) {
		return -EINVAL;
	}

	s = get_settings_by_id( (u8)ctx_id );
	if ( !s ) {
		return -ENODEV;
	}

	base = (uint32_t)s->isp_base;
	cur  = system_isp_read_32( base + MODALIX_ISP_PIPELINE_BYPASS_OFFSET );
	cur  = ( cur & MODALIX_ISP_PIPELINE_BYPASS_RESERVED ) |
	       ( bypass_word & ~MODALIX_ISP_PIPELINE_BYPASS_RESERVED );
	system_isp_write_32( base + MODALIX_ISP_PIPELINE_BYPASS_OFFSET, cur );

	LOG_CAT( ALOG_CAT_CONTROL, LOG_DEBUG, "ctx %u: applied ISP bypass 0x%08x (reg now 0x%08x)",
	     ctx_id, bypass_word, cur );
	return 0;
}
EXPORT_SYMBOL_GPL( modalix_isp_apply_pipeline_bypass );

/* Read back context @ctx_id's live ISP pipeline bypass word. */
int modalix_isp_read_pipeline_bypass( uint32_t ctx_id, uint32_t *out )
{
	acamera_settings *s;

	if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER || !out ) {
		return -EINVAL;
	}

	s = get_settings_by_id( (u8)ctx_id );
	if ( !s ) {
		return -ENODEV;
	}

	*out = system_isp_read_32( (uint32_t)s->isp_base +
	                           MODALIX_ISP_PIPELINE_BYPASS_OFFSET );
	return 0;
}
EXPORT_SYMBOL_GPL( modalix_isp_read_pipeline_bypass );

int32_t ctrl_channel_init( acamera_settings *settings, uint8_t num_of_contexts )
{
    uint32_t i;

    if ( num_of_contexts > FIRMWARE_CONTEXT_NUMBER ) {
        LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR, "num_of_contexts (%u) exceeds FIRMWARE_CONTEXT_NUMBER (%u).",
             num_of_contexts, FIRMWARE_CONTEXT_NUMBER );
        return -EINVAL;
    }

    for ( i = 0; i < num_of_contexts; i++ ) {
        struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[i];

        p_ctx->ctx_id = i;
        mutex_init( &p_ctx->lock );
        init_waitqueue_head( &p_ctx->wq );
        p_ctx->calib_ready = false;
        p_ctx->shutting_down = false;
        p_ctx->calibrations = NULL;
        p_ctx->borrowed = NULL;
        p_ctx->retiring = NULL;
        p_ctx->in_flight = 0;

        /* Route the firmware's get_calibrations read path through the
         * cbs[i] dispatcher; cbs[i] blocks on wait_for_calibrations
         * until the IPA pushes a blob via
         * MODALIX_ISP_V4L2_CID_CALIBRATION_BLOB. The kernel no longer
         * carries a baked-in fallback — sensor/calibrations/*.o was
         * removed from the Makefile and the runtime_initialization_
         * settings.h .get_calibrations field is NULL. */
        settings[i].get_calibrations = cbs[i];

        p_ctx->dev_inited = 1;
    }

    /* A slot id outside the table's range would silently stop policing that
     * slot, so say so once at init rather than never. */
    for ( i = 0; i < ARRAY_SIZE( modalix_calib_slot_types ); i++ ) {
        if ( modalix_calib_slot_types[i].slot >= MODALIX_ISP_CALIB_TOTAL_SIZE ) {
            LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR,
                 "calib slot-type table entry %u names slot %u, past TOTAL_SIZE %u",
                 i, modalix_calib_slot_types[i].slot, MODALIX_ISP_CALIB_TOTAL_SIZE );
        }
    }

    LOG_CAT( ALOG_CAT_CONTROL, LOG_INFO, "calib bridge: initialised %u ctxs", num_of_contexts );
    return 0;
}

void ctrl_channel_process( void )
{
    /* No-op. Retained for callsite stability in the user-space
     * firmware port (which still ticks this from its main loop).
     * The chardev's kfifo-out drain that used to live here was
     * unused — zero in-tree producers — and was deleted with the
     * rest of the chardev infrastructure. */
}

/* Stop handing calibrations out and release every waiter, without freeing
 * anything: the ISP contexts are still up at this point and their
 * calibration managers still hold slot pointers into the live set. */
void ctrl_channel_shutdown( void )
{
    uint32_t i;

    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[i];

        if ( !p_ctx->dev_inited )
            continue;

        /* Say we are going away under the lock a callback takes, so no
         * callback can be past its own check and still about to run. Then
         * wake every waiter, so they exit rather than sleeping out the
         * timeout on a predicate that will never come true. */
        mutex_lock( &p_ctx->lock );
        WRITE_ONCE( p_ctx->calib_ready, false );
        WRITE_ONCE( p_ctx->shutting_down, true );
        mutex_unlock( &p_ctx->lock );
        wake_up_interruptible_all( &p_ctx->wq );

        /* A callback that got in before the flag is still holding pointers
         * it is about to hand the calibration manager. Let it finish: the
         * contexts are torn down straight after this returns. */
        if ( wait_event_timeout( p_ctx->wq,
                                 READ_ONCE( p_ctx->in_flight ) == 0,
                                 msecs_to_jiffies( CALIB_PUSH_WAIT_MS ) ) == 0 ) {
            LOG_CAT( ALOG_CAT_CONTROL, LOG_ERR,
                 "ctx %u: a calibration hand-out did not finish before teardown",
                 p_ctx->ctx_id );
        }
    }
}

/* Free the calibration storage. Must run after every calibration manager
 * entry is gone - see acamera_deinit() - because until then the firmware
 * still has pointers into these allocations. */
void ctrl_channel_deinit( void )
{
    uint32_t i;

    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[i];

        if ( !p_ctx->dev_inited )
            continue;

        mutex_lock( &p_ctx->lock );
        if ( p_ctx->retiring != NULL && p_ctx->retiring != p_ctx->borrowed &&
             p_ctx->retiring != p_ctx->calibrations ) {
            kfree( p_ctx->retiring );
        }
        if ( p_ctx->borrowed != p_ctx->calibrations ) {
            kfree( p_ctx->borrowed );
        }
        kfree( p_ctx->calibrations );
        p_ctx->retiring = NULL;
        p_ctx->borrowed = NULL;
        p_ctx->calibrations = NULL;
        p_ctx->dev_inited = 0;
        mutex_unlock( &p_ctx->lock );
    }
}
