/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * SiMa Modalix — the sensor half of the sensor-to-ISP contract.
 *
 * Vendor V4L2 control IDs a sensor subdev publishes so the ISP can learn
 * what the sensor's currently selected mode emits. Split out of
 * modalix_isp_v4l2.h so a sensor driver need not pull in the ISP's own
 * control, calibration and statistics UAPI to implement them.
 *
 * Stable, like the ISP's: never renumber an existing CID; new ones append.
 */

#ifndef _UAPI_MODALIX_SENSOR_V4L2_H
#define _UAPI_MODALIX_SENSOR_V4L2_H

#include <linux/videodev2.h>

/* ============================================================================
 * Vendor V4L2 control IDs exposed by SENSOR subdevs
 *
 * These live on the sensor's own control handler, not on the ISP video
 * node, and are how a multi-exposure sensor tells the ISP what its
 * currently selected mode actually emits.
 *
 * They are not uniformly read-only, so each says what it is:
 *
 *   WDR_MODE          read-only. The mode follows from media-bus format
 *                     negotiation, so it has exactly one selection axis.
 *   LINEARISED_BITS   read-only. It is a property of the selected mode.
 *   EXPOSURES         mode-dependent. Read-only in a mode with a fixed
 *                     exposure count, writable in a mode that offers a
 *                     choice, and refused with -EBUSY while streaming
 *                     because it is applied as a set at stream start.
 *                     Read it back after writing: a mode that cannot
 *                     honour the request clamps rather than failing.
 *   HDR_EXPOSURE_RATIO writable where the sensor supports it, and grabbed
 *                     while streaming for the same reason.
 *
 * A sensor that implements none of them is a plain linear sensor, and the
 * ISP defaults to LINEAR with one exposure.
 * ============================================================================*/

#define MODALIX_SENSOR_V4L2_CID_BASE   (V4L2_CTRL_CLASS_USER | 0xe000)

/* Values match the ISP core's WDR_MODE_* encoding. The meanings below are the
 * Arm Mali-C71AE software TRM's, which describes every mode except FS_HDR. */
enum modalix_sensor_wdr_mode {
	MODALIX_SENSOR_WDR_MODE_LINEAR = 0,  /* one exposure; frame stitch skipped */
	MODALIX_SENSOR_WDR_MODE_NATIVE = 1,  /* multi-exposure, combined in the sensor */
	MODALIX_SENSOR_WDR_MODE_FS_HDR = 2,  /* Arm documents no meaning for this one */
	MODALIX_SENSOR_WDR_MODE_FS_LIN = 3,  /* sensor supplies exposures, ISP stitches */
};

enum modalix_sensor_v4l2_cid {
	MODALIX_SENSOR_V4L2_CID_WDR_MODE = MODALIX_SENSOR_V4L2_CID_BASE,
	MODALIX_SENSOR_V4L2_CID_EXPOSURES,
	/* Ratio between successive exposures, as a menu of 2x/4x/8x/16x.
	 * Writable: it trades dynamic range against the longest exposure
	 * the sensor's delay buffers can hold. */
	MODALIX_SENSOR_V4L2_CID_HDR_EXPOSURE_RATIO,
	/* Bits the pixels represent once the ISP has expanded them: what the
	 * sensor linearised its exposures to before companding them onto the
	 * wire. Equal to the wire width when nothing is companded, which is
	 * also what the ISP assumes for a sensor that omits the control.
	 * A wider value is what tells the ISP to run its decompander. */
	MODALIX_SENSOR_V4L2_CID_LINEARISED_BITS,
};

/* Most exposures the ISP can be told a single mode carries. */
#define MODALIX_SENSOR_EXPOSURES_MAX   4

/* Widest data the ISP input port carries, and so the widest a sensor may
 * claim to linearise to. Matches the core's ISP_INPUT_PORT_MAX_BIT_WIDTH. */
#define MODALIX_SENSOR_LINEARISED_BITS_MAX   20

#endif /* _UAPI_MODALIX_SENSOR_V4L2_H */
