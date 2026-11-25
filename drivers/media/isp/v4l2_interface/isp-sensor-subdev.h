/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ISP real-sensor acquisition over the V4L2 subdev API: async-bind the
 * per-context sensor and read its format/timing via get_fmt + blanking.
 */

#ifndef __ISP_SENSOR_SUBDEV_H__
#define __ISP_SENSOR_SUBDEV_H__

#include <linux/types.h>
#include <linux/media/simaai/modalix_isp_sensor_info.h>

struct v4l2_device;

/*
 * Bayer order fed to the ISP TOP rggb_start field. Values match the HW
 * encoding ACAMERA_ISP_TOP_RGGB_START_* (R_GR_GB_B=0 .. B_GB_GR_R=3).
 */
enum isp_bayer_order {
	ISP_BAYER_RGGB = 0,
	ISP_BAYER_GRBG = 1,
	ISP_BAYER_GBRG = 2,
	ISP_BAYER_BGGR = 3,
};

/* Sensor truth handed to the ISP sensor FSM, sourced from the real subdev. */
struct isp_real_sensor_info {
	u16 active_width;   /* fed to the ISP, 32-pixel aligned */
	u16 active_height;
	u16 total_width;    /* active + HBLANK (sensor line length) */
	u16 total_height;   /* active + VBLANK (frame length lines) */
	u8  data_width;     /* bits per pixel from the mbus code */
	u8  rggb_start;     /* 0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR */
	u8  cfa_pattern;    /* 0 = bayer */
	u8  num_channels;   /* virtual channels in this mode (1 = linear) */
	u8  wdr_mode;       /* enum modalix_sensor_wdr_mode (0 = linear) */
	u8  exposures;      /* exposures this mode captures (1 = linear) */
	u8  linearised_bits;/* bits after expansion; == data_width uncompanded */
	u32 lines_per_second;
	u8  integration_time_apply_delay;
	u32 integration_time_min;   /* V4L2_CID_EXPOSURE min (lines) */
	u32 integration_time_max;   /* V4L2_CID_EXPOSURE max (lines) */
};

/*
 * Apply a live sensor snapshot to a sensor-info blob. Everything the snapshot
 * describes comes from it, so the blob cannot end up carrying one mode's
 * geometry beside another mode's exposure count. Both sites that fill the blob
 * - the sensor FSM at config and the V4L2 refresh at get - go through here, so
 * they cannot drift apart.
 */
static inline void isp_sensor_info_apply(acamera_cmd_sensor_info *gsi,
					 const struct isp_real_sensor_info *r)
{
	gsi->active_width = r->active_width;
	gsi->active_height = r->active_height;
	gsi->total_width = r->total_width;
	gsi->total_height = r->total_height;
	gsi->rggb_start = r->rggb_start;
	gsi->cfa_pattern = r->cfa_pattern;
	gsi->sensor_output_bits = r->data_width;
	gsi->lines_per_second = r->lines_per_second;
	gsi->integration_time_apply_delay = r->integration_time_apply_delay;
	gsi->sensor_exp_number = r->exposures;
	gsi->current_sensor_mode.num_channels = r->num_channels;
	/* Seed per-channel cv > CAP_MIN so the decompander channel-switch search
	 * finds a channel instead of logging a harmless CRIT (1 = CAP_NATIVE_PASS_THROUGH). */
	for (u8 i = 0; i < r->num_channels && i < GENERAL_ROUTER_MAX_ISP_CHANNELS; i++)
		gsi->current_sensor_mode.channel_info.channel_desc[i].cv = 1;
	gsi->current_sensor_mode.wdr_mode = r->wdr_mode;
	gsi->current_sensor_mode.exposures = r->exposures;
	gsi->current_sensor_mode.channel_info.exposure_max_bit_width =
		r->linearised_bits;
	gsi->current_sensor_mode.resolution.width = r->active_width;
	gsi->current_sensor_mode.resolution.height = r->active_height;

	/* A sensor that reports no bound keeps the caller's defaults; zeroing
	 * these would leave AE unbounded. */
	if (r->integration_time_max) {
		gsi->integration_time_min = r->integration_time_min;
		gsi->integration_time_max = r->integration_time_max;
		gsi->integration_time_limit = r->integration_time_max;
	}
}

/*
 * Register the ISP's sensor async-notifier on its v4l2_device. Walks each
 * simaai,v4l2-vdma node (id = ctx) to the sensor fwnode and binds it.
 * Non-fatal: a context with no reachable sensor (file-replay / unpopulated)
 * is simply left unbound.
 */
int isp_sensor_subdev_register(struct v4l2_device *v4l2_dev);

/* Tear down the notifier (module exit). */
void isp_sensor_subdev_unregister(void);

/*
 * Ask the sensor feeding @ctx_id to combine @exposures per frame. Resolves
 * the same subdev as isp_sensor_subdev_query(). Returns -ENODEV when no
 * sensor resolves, -ENOTTY when the sensor has no such control, or whatever
 * the control framework returns - notably -EACCES when the current mode
 * fixes the count.
 */
int isp_sensor_subdev_set_exposures(u32 ctx_id, u32 exposures);

/* Set the i2c bus of the sensor feeding a context (-1 clears it). */
void isp_sensor_subdev_set_source(u32 ctx_id, int i2c_bus);

/*
 * Query the real sensor feeding a context: by source bus if set, else by
 * matching a bound sensor to want_w x want_h. -ENODEV if none resolves.
 */
int isp_sensor_subdev_query(u32 ctx_id, u16 want_w, u16 want_h,
			    struct isp_real_sensor_info *out);

#endif /* __ISP_SENSOR_SUBDEV_H__ */
