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
 *     yet (SOCSW-5579 will retire that fallback);
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
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include "system_stdlib.h"
#include "acamera_ctrl_channel.h"
#include "acamera_logger.h"
#include "acamera_calib_mgr.h"
#include "acamera_isp_config.h" /* system_isp_read_32/_write_32, reg 0xe040 */
#include "acamera_isp_ctx.h"    /* acamera_settings */

/* ISP pipeline bypass register (Mali "Pipeline" group). Reserved bits
 * are never driven by the IPA and kept at their power-on value. */
#define MODALIX_ISP_PIPELINE_BYPASS_OFFSET   0xe040u
#define MODALIX_ISP_PIPELINE_BYPASS_RESERVED \
	( BIT(0) | BIT(1) | BIT(23) | BIT(25) | BIT(26) )

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
    ACameraCalibrations *calibrations;
};

static struct ctrl_channel_dev_context ctrl_channel_ctx[FIRMWARE_CONTEXT_NUMBER];

/* Block until the IPA has pushed a calibration blob for @p_ctx (or
 * until the wait times out). Returns 0 when calibrations are ready,
 * -ETIMEDOUT if the IPA never pushed inside CALIB_PUSH_WAIT_MS, or
 * -ERESTARTSYS if interrupted. Safe to call from a kernel thread. */
static int wait_for_calibrations( struct ctrl_channel_dev_context *p_ctx )
{
    long ret;

    if ( READ_ONCE( p_ctx->calib_ready ) )
        return 0;

    ret = wait_event_interruptible_timeout(
        p_ctx->wq,
        READ_ONCE( p_ctx->calib_ready ),
        msecs_to_jiffies( CALIB_PUSH_WAIT_MS ) );

    if ( ret == 0 ) {
        LOG( LOG_ERR, "ctx %u: timed out waiting for IPA to push calibrations",
             p_ctx->ctx_id );
        return -ETIMEDOUT;
    }
    if ( ret < 0 ) {
        LOG( LOG_INFO, "ctx %u: interrupted while waiting for calibration push",
             p_ctx->ctx_id );
        return -ERESTARTSYS;
    }
    return 0;
}

#define REPEAT_16(x) x(0) x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10) x(11) x(12) x(13) x(14) x(15)
#define ASSIGN_CALIB_CALLBACK(CTX) custom_get_calib_ctx##CTX,
#define DEFINE_CALIB_CALLBACK(CTX) \
static int32_t custom_get_calib_ctx##CTX(uint32_t wdr_mode, void *param) { \
	struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[CTX]; \
	ACameraCalibrations *c, *user_c; \
	int rc, i; \
	if (param == NULL) \
		return -EINVAL; \
	rc = wait_for_calibrations(p_ctx); \
	if (rc) \
		return rc; \
	mutex_lock(&p_ctx->lock); \
	user_c = p_ctx->calibrations; \
	if (user_c == NULL) { \
		mutex_unlock(&p_ctx->lock); \
		return -EAGAIN; \
	} \
	c = (ACameraCalibrations *)param; \
	for (i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++) { \
		LookupTable *u_lut = (LookupTable *)(uintptr_t)user_c->calibrations[i]; \
		if (user_c->calibrations[i] != 0 && u_lut != NULL && u_lut->ptr != 0) { \
			c->calibrations[i] = user_c->calibrations[i]; \
		} \
	} \
	mutex_unlock(&p_ctx->lock); \
	LOG(LOG_INFO, "Wrote user calibrations for CTX %d (WDR Mode: %u)", CTX, wdr_mode); \
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
    ACameraCalibrations             *kcalibs;
    size_t                           hdr;
    int                              i;

    if ( ctx_id >= FIRMWARE_CONTEXT_NUMBER || data == NULL || size == 0 ) {
        return -EINVAL;
    }

    p_ctx = &ctrl_channel_ctx[ctx_id];
    if ( !p_ctx->dev_inited ) {
        return -EAGAIN;
    }

    kcalibs = kmalloc( size, GFP_KERNEL );
    if ( !kcalibs ) {
        return -ENOMEM;
    }
    memcpy( kcalibs, data, size );

    /* Patch offset -> kernel pointer in place. The blob crosses the V4L2
     * boundary so it is untrusted: bounds-check every offset against
     * `size` and reject a malformed/mismatched blob with -EINVAL rather
     * than dereferencing a wild pointer. Wire form holds a byte offset in
     * each __aligned_u64 slot; after patching it holds a real kernel
     * pointer (same storage, two interpretations). */
    hdr = sizeof( *kcalibs );
    if ( size < hdr ) {
        goto bad_blob;
    }

    for ( i = 0; i < MODALIX_ISP_CALIB_TOTAL_SIZE; i++ ) {
        uintptr_t    table_offset = (uintptr_t)kcalibs->calibrations[i];
        LookupTable *table;

        if ( !table_offset ) {
            continue;
        }

        /* descriptor must sit after the root header, be 8-byte aligned,
         * and fit wholly inside the blob */
        if ( table_offset < hdr || ( table_offset & 0x7 ) ||
             table_offset > size - sizeof( LookupTable ) ) {
            goto bad_blob;
        }

        table = (LookupTable *)( (uint8_t *)kcalibs + table_offset );
        kcalibs->calibrations[i] = (__u64)(uintptr_t)table;

        if ( table->ptr ) {
            uintptr_t data_offset = (uintptr_t)table->ptr;
            size_t    data_bytes  = (size_t)table->rows * table->cols * table->width;

            /* LUT data must also lie after the header and fit in the blob */
            if ( data_offset < hdr || data_offset > size ||
                 data_bytes > size - data_offset ) {
                goto bad_blob;
            }

            table->ptr = (__u64)(uintptr_t)( (uint8_t *)kcalibs + data_offset );
        }
    }

    mutex_lock( &p_ctx->lock );
    if ( p_ctx->calibrations != NULL ) {
        kfree( p_ctx->calibrations );
    }
    p_ctx->calibrations = kcalibs;
    WRITE_ONCE( p_ctx->calib_ready, true );
    mutex_unlock( &p_ctx->lock );

    /* Wake any FSM thread that hit wait_for_calibrations before this
     * push arrived. With SOCSW-5579 the kernel no longer has a baked-in
     * default to fall back to, so a missing push would otherwise hang
     * the read path. */
    wake_up_interruptible( &p_ctx->wq );

    LOG( LOG_DEBUG, "ctx %u: V4L2 install of calibration blob (%zu bytes)",
         ctx_id, size );
    return 0;

bad_blob:
    kfree( kcalibs );
    LOG( LOG_ERR, "ctx %u: rejected malformed calibration blob (%zu bytes)",
         ctx_id, size );
    return -EINVAL;
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

	LOG( LOG_DEBUG, "ctx %u: applied ISP bypass 0x%08x (reg now 0x%08x)",
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
        LOG( LOG_ERR, "num_of_contexts (%u) exceeds FIRMWARE_CONTEXT_NUMBER (%u).",
             num_of_contexts, FIRMWARE_CONTEXT_NUMBER );
        return -EINVAL;
    }

    for ( i = 0; i < num_of_contexts; i++ ) {
        struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[i];

        p_ctx->ctx_id = i;
        mutex_init( &p_ctx->lock );
        init_waitqueue_head( &p_ctx->wq );
        p_ctx->calib_ready = false;
        p_ctx->calibrations = NULL;

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

    LOG( LOG_INFO, "calib bridge: initialised %u ctxs", num_of_contexts );
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

void ctrl_channel_deinit( void )
{
    uint32_t i;
    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx[i];

        if ( !p_ctx->dev_inited )
            continue;

        /* Clear the ready flag and wake every waiter so they exit
         * cleanly with -EAGAIN / -ERESTARTSYS instead of hanging until
         * the wait times out. */
        WRITE_ONCE( p_ctx->calib_ready, false );
        wake_up_interruptible_all( &p_ctx->wq );

        mutex_lock( &p_ctx->lock );
        if ( p_ctx->calibrations ) {
            kfree( p_ctx->calibrations );
            p_ctx->calibrations = NULL;
        }
        mutex_unlock( &p_ctx->lock );

        p_ctx->dev_inited = 0;
    }
}
