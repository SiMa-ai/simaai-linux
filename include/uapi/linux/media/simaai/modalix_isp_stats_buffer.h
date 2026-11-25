/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * SiMa Modalix ISP — V4L2 META_CAPTURE stats buffer layout.
 *
 * The Modalix ISP exposes per-frame 3A statistics through a
 * V4L2_BUF_TYPE_META_CAPTURE video device (one per ISP context). User-
 * space mmaps the dequeued buffer and finds a single struct
 * modalix_isp_stats_buffer; the per-FSM-type stats sub-sections
 * (AE histogram, AWB per-zone, gamma histogram) sit inline.
 *
 * Producer: the kernel BH calls vb2_buffer_done() after copying the
 * current frame's stats into the queued buffer. Consumer: the
 * libcamera IPA dequeues via VIDIOC_DQBUF, reads the populated
 * sections, requeues with VIDIOC_QBUF for reuse.
 *
 * Sparse population is the norm: not every FSM produces stats every
 * frame, and during the staged rollout some sections may be empty
 * while the legacy sbuf path still services them. The `valid_mask`
 * tells consumers which sections were filled this frame.
 *
 * Layout discipline: new sub-sections append to the END of the
 * struct so older user-space binaries that don't know about a new
 * section can still read existing ones at unchanged offsets, gated
 * by a new valid_mask bit an old consumer never tests. struct
 * sizeof is treated as the V4L2 buffer size, so an append is still
 * a lockstep kernel+consumer deploy. Bumping
 * MODALIX_ISP_STATS_BUFFER_VERSION is reserved for changes that
 * reinterpret existing fields or move offsets, so user-space can
 * refuse those instead of silently misinterpreting.
 *
 * Pairs with [planned] modalix_isp_params_buffer.h on the other
 * direction (META_OUTPUT) — that header carries the IPA's computed
 * gains / CCM / gamma / iridix back to the kernel.
 */

#ifndef _UAPI_MODALIX_ISP_STATS_BUFFER_H
#define _UAPI_MODALIX_ISP_STATS_BUFFER_H

#include <linux/types.h>

/* Buffer layout version. Bump when the on-the-wire encoding changes
 * incompatibly; kernel rejects unknown versions. Producers should set
 * this field to MODALIX_ISP_STATS_BUFFER_VERSION_V1 for the current
 * layout. */
#define MODALIX_ISP_STATS_BUFFER_VERSION_V1  1u

/* Maximum AWB zones the HW reports per frame. Matches the Modalix
 * ISP's metering hardware (225 zone records). */
#define MODALIX_ISP_AWB_MAX_ZONES            225u

/* AE / gamma histogram bin count. Matches the ISP_METERING_HISTOGRAM
 * register-block size — 1024 bins, each a 32-bit pixel-count. */
#define MODALIX_ISP_HISTOGRAM_BINS           1024u

/* Maximum AE metering zones (aexp block) the HW reports per frame.
 * Same 15x15 grid ceiling as AWB. */
#define MODALIX_ISP_AE_MAX_ZONES             225u

/* valid_mask bit positions in struct modalix_isp_stats_buffer. */
#define MODALIX_ISP_STATS_VALID_AE           (1u << 0)
#define MODALIX_ISP_STATS_VALID_AWB          (1u << 1)
#define MODALIX_ISP_STATS_VALID_GAMMA        (1u << 2)
#define MODALIX_ISP_STATS_VALID_AE_ZONES     (1u << 3)

/**
 * struct modalix_isp_awb_zone - one entry in the AWB per-zone array.
 *
 * @rg:    red/green ratio (Q8 fixed-point); 256 == 1.0.
 * @bg:    blue/green ratio (Q8 fixed-point); 256 == 1.0.
 * @sum:   count of unclipped pixels contributing to this zone (the
 *         hardware pre-filters pixels outside the AWB white-level
 *         and Cr/Cb reference bounds, so a low @sum already implies
 *         a saturated or underexposed zone).
 */
struct modalix_isp_awb_zone {
	__u16 rg;
	__u16 bg;
	__u32 sum;
};

/**
 * struct modalix_isp_ae_stats - histogram statistics for AE.
 *
 * @histogram:     full-frame luma histogram, fixed-bin width.
 * @histogram_sum: total pixel count contributing to the histogram
 *                 (= sum over all bins; carried in the buffer for
 *                 convenience so user-space doesn't have to re-sum).
 */
struct modalix_isp_ae_stats {
	__u32 histogram[MODALIX_ISP_HISTOGRAM_BINS];
	__u32 histogram_sum;
	__u32 _reserved;
};

/**
 * struct modalix_isp_awb_stats - per-zone chromaticity statistics for
 * AWB.
 *
 * @zones:       fixed-size array; the first @zone_count entries are
 *               populated this frame, the rest are undefined.
 * @zone_count:  number of valid entries in @zones; bounded by
 *               MODALIX_ISP_AWB_MAX_ZONES.
 */
struct modalix_isp_awb_stats {
	struct modalix_isp_awb_zone zones[MODALIX_ISP_AWB_MAX_ZONES];
	__u32 zone_count;
	__u32 _reserved;
};

/**
 * struct modalix_isp_ae_zone - one entry in the AE per-zone (aexp)
 * array.
 *
 * @hist: the four stored bins of the zone's 5-bin intensity
 *        histogram, in threshold order: bin0, bin1, bin3, bin4.
 *        Counts are normalised by the zone's processed pixel count
 *        so the five bins sum to 65536; the middle bin is derived:
 *        bin2 = 65536 - (bin0 + bin1 + bin3 + bin4). Bin edges are
 *        the ISP's HIST_THRESH registers (defaults 16/32/208/224).
 */
struct modalix_isp_ae_zone {
	__u16 hist[4];
};

/**
 * struct modalix_isp_ae_zone_stats - AE per-zone histogram statistics
 * (the metering aexp block).
 *
 * Spatial complement of the full-frame AE histogram: one 5-bin
 * histogram per metering zone. Consumed by the rolling-shutter
 * (spatial) flicker detector, which needs a per-row luma profile the
 * full-frame histogram cannot provide.
 *
 * @zones:   fixed-size array in zone raster order; the first
 *           @zones_h * @zones_v entries are populated this frame,
 *           the rest are undefined.
 * @zones_h: metering grid width in zones.
 * @zones_v: metering grid height in zones; @zones_h * @zones_v is
 *           bounded by MODALIX_ISP_AE_MAX_ZONES.
 */
struct modalix_isp_ae_zone_stats {
	struct modalix_isp_ae_zone zones[MODALIX_ISP_AE_MAX_ZONES];
	__u16 zones_h;
	__u16 zones_v;
	__u32 _reserved;
};

/**
 * struct modalix_isp_gamma_stats - histogram statistics for the gamma
 * FSM's auto-contrast loop.
 *
 * Same layout as AE — same ISP histogram block, just sampled at a
 * different tap point in the pipeline. Carries them separately so
 * AE and gamma can tap at distinct points without sharing bins.
 */
struct modalix_isp_gamma_stats {
	__u32 histogram[MODALIX_ISP_HISTOGRAM_BINS];
	__u32 histogram_sum;
	__u32 _reserved;
};

/**
 * struct modalix_isp_stats_buffer - V4L2 META_CAPTURE buffer payload.
 *
 * @version:      MODALIX_ISP_STATS_BUFFER_VERSION_V1 for the layout
 *                described here. Future bumps coordinate with new
 *                ABI.
 * @frame_id:     monotonic per-context frame counter; identifies
 *                which frame these stats belong to.
 * @valid_mask:   bitmask of MODALIX_ISP_STATS_VALID_* flags
 *                indicating which sub-sections were filled this
 *                frame. Sections without their bit set hold
 *                undefined data.
 * @timestamp_ns: kernel CLOCK_MONOTONIC timestamp (nanoseconds)
 *                captured by the BH when it copied the stats from
 *                the ISP's metering memory. Useful for correlating
 *                stats with the frame's start-of-exposure event.
 * @ae:           histogram statistics for the AE FSM.
 * @awb:          per-zone chromaticity statistics for the AWB FSM.
 * @gamma:        histogram statistics for the gamma FSM.
 * @ae_zones:     AE per-zone histogram statistics (spatial flicker
 *                detector feed). Appended after @gamma per the
 *                layout discipline above: older user-space still
 *                finds @ae/@awb/@gamma at unchanged offsets and
 *                simply never sees MODALIX_ISP_STATS_VALID_AE_ZONES.
 *
 * Iridix is intentionally absent: the iridix FSM has no kernel→user
 * stats payload (it's user-space-produced only).
 */
struct modalix_isp_stats_buffer {
	__u32                            version;
	__u32                            frame_id;
	__u32                            valid_mask;
	__u32                            _reserved;
	__u64                            timestamp_ns;

	struct modalix_isp_ae_stats      ae;
	struct modalix_isp_awb_stats     awb;
	struct modalix_isp_gamma_stats   gamma;
	struct modalix_isp_ae_zone_stats ae_zones;
};

#endif /* _UAPI_MODALIX_ISP_STATS_BUFFER_H */
