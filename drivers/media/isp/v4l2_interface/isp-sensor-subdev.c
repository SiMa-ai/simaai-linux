// SPDX-License-Identifier: GPL-2.0
/*
 * ISP real-sensor acquisition: remember each ctx's sensor fwnode, resolve the
 * live subdev on demand for format/timing. Reference-only - the sensor is owned
 * by the capture media graph, so the ISP must not async-bind it.
 */

#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/math64.h>
#include <linux/i2c.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include <uapi/linux/media/simaai/modalix_isp_v4l2.h>

#include "acamera_configuration.h"
#include "acamera_logger.h"
#include "isp-sensor-subdev.h"

/* One slot per ISP context: libcamera hands out any ctx, 8 cameras use 0..7. */
#define ISP_SENSOR_MAX_CTX FIRMWARE_CONTEXT_NUMBER


static struct {
	struct fwnode_handle *sensor_fw[ISP_SENSOR_MAX_CTX]; /* per-ctx DT sensor fwnode */
	int ctx_source_bus[ISP_SENSOR_MAX_CTX];              /* libcamera-set source per ctx */
	bool registered;
} isp_sensor = {
	.ctx_source_bus = { [0 ... ISP_SENSOR_MAX_CTX - 1] = -1 },
};

/*
 * Sensor lifetime.
 *
 * A subdev pointer taken from i2c client data is only as good as the driver
 * that put it there. Unbind unregisters the subdev and frees the control
 * handler, and the sensor's private data is devm-allocated, so a raw pointer
 * held across anything that sleeps - and every read or write here talks i2c -
 * can be freed underneath us. Holding a device reference is not enough: it
 * keeps the struct device allocated, it does not stop .remove() running.
 *
 * So resolution returns a *held i2c client*, and the caller takes the driver
 * core's device lock before touching the subdev. .remove() runs under that
 * same lock, so it cannot be in progress while we hold it, and re-reading the
 * client data under the lock tells us whether it already ran.
 *
 *	client = sensor_client_*(...);	  holds a reference
 *	sd = sensor_lock(client);	  takes device_lock, revalidates
 *	... use sd ...
 *	sensor_unlock(client);		  device_unlock + put_device
 */

/* Held i2c client behind a sensor fwnode, or NULL. Caller puts it. */
static struct i2c_client *client_from_fwnode(struct fwnode_handle *fw)
{
	if (!fw)
		return NULL;
	/* i2c_find_device_by_fwnode() returns a reference; we keep it. */
	return i2c_find_device_by_fwnode(fw);
}

/*
 * Take the device lock and hand back the sensor subdev, or NULL if the driver
 * has gone. On NULL the caller must still sensor_unlock() to drop the
 * reference - the lock is released here.
 */
static struct v4l2_subdev *sensor_lock(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	device_lock(&client->dev);
	if (!client->dev.driver) {
		device_unlock(&client->dev);
		return NULL;
	}
	sd = i2c_get_clientdata(client);
	/* Trust only a real v4l2_subdev (v4l2_i2c_subdev_init sets sd->dev). */
	if (!sd || sd->dev != &client->dev) {
		device_unlock(&client->dev);
		return NULL;
	}
	return sd;
}

static void sensor_unlock(struct i2c_client *client, bool locked)
{
	if (locked)
		device_unlock(&client->dev);
	put_device(&client->dev);
}

/* Map a bayer mbus code to {rggb_start (enum isp_bayer_order), bits}. */
static int mbus_to_bayer(u32 code, u8 *rggb_start, u8 *bits)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:   *rggb_start = ISP_BAYER_RGGB; *bits = 8;  return 0;
	case MEDIA_BUS_FMT_SGRBG8_1X8:   *rggb_start = ISP_BAYER_GRBG; *bits = 8;  return 0;
	case MEDIA_BUS_FMT_SGBRG8_1X8:   *rggb_start = ISP_BAYER_GBRG; *bits = 8;  return 0;
	case MEDIA_BUS_FMT_SBGGR8_1X8:   *rggb_start = ISP_BAYER_BGGR; *bits = 8;  return 0;
	case MEDIA_BUS_FMT_SRGGB10_1X10: *rggb_start = ISP_BAYER_RGGB; *bits = 10; return 0;
	case MEDIA_BUS_FMT_SGRBG10_1X10: *rggb_start = ISP_BAYER_GRBG; *bits = 10; return 0;
	case MEDIA_BUS_FMT_SGBRG10_1X10: *rggb_start = ISP_BAYER_GBRG; *bits = 10; return 0;
	case MEDIA_BUS_FMT_SBGGR10_1X10: *rggb_start = ISP_BAYER_BGGR; *bits = 10; return 0;
	case MEDIA_BUS_FMT_SRGGB12_1X12: *rggb_start = ISP_BAYER_RGGB; *bits = 12; return 0;
	case MEDIA_BUS_FMT_SGRBG12_1X12: *rggb_start = ISP_BAYER_GRBG; *bits = 12; return 0;
	case MEDIA_BUS_FMT_SGBRG12_1X12: *rggb_start = ISP_BAYER_GBRG; *bits = 12; return 0;
	case MEDIA_BUS_FMT_SBGGR12_1X12: *rggb_start = ISP_BAYER_BGGR; *bits = 12; return 0;
	case MEDIA_BUS_FMT_SRGGB14_1X14: *rggb_start = ISP_BAYER_RGGB; *bits = 14; return 0;
	case MEDIA_BUS_FMT_SGRBG14_1X14: *rggb_start = ISP_BAYER_GRBG; *bits = 14; return 0;
	case MEDIA_BUS_FMT_SGBRG14_1X14: *rggb_start = ISP_BAYER_GBRG; *bits = 14; return 0;
	case MEDIA_BUS_FMT_SBGGR14_1X14: *rggb_start = ISP_BAYER_BGGR; *bits = 14; return 0;
	case MEDIA_BUS_FMT_SRGGB16_1X16: *rggb_start = ISP_BAYER_RGGB; *bits = 16; return 0;
	case MEDIA_BUS_FMT_SGRBG16_1X16: *rggb_start = ISP_BAYER_GRBG; *bits = 16; return 0;
	case MEDIA_BUS_FMT_SGBRG16_1X16: *rggb_start = ISP_BAYER_GBRG; *bits = 16; return 0;
	case MEDIA_BUS_FMT_SBGGR16_1X16: *rggb_start = ISP_BAYER_BGGR; *bits = 16; return 0;
	default:
		return -EINVAL;
	}
}

/* vdma node (port@0/endpoint@0) -> csi -> (port@0/endpoint@0 sink) -> sensor */
static struct fwnode_handle *vdma_to_sensor_fwnode(struct device_node *vdma_np)
{
	struct fwnode_handle *vdma_fw = of_fwnode_handle(vdma_np);
	struct fwnode_handle *ep, *csi_fw, *sensor_fw;

	ep = fwnode_graph_get_endpoint_by_id(vdma_fw, 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return NULL;
	csi_fw = fwnode_graph_get_remote_port_parent(ep);
	fwnode_handle_put(ep);
	if (!csi_fw)
		return NULL;

	ep = fwnode_graph_get_endpoint_by_id(csi_fw, 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	fwnode_handle_put(csi_fw);
	if (!ep)
		return NULL;
	sensor_fw = fwnode_graph_get_remote_port_parent(ep);
	fwnode_handle_put(ep);
	if (!sensor_fw)
		return NULL;

	if (!fwnode_device_is_available(sensor_fw)) {
		fwnode_handle_put(sensor_fw);
		return NULL;
	}
	return sensor_fw;
}

int isp_sensor_subdev_register(struct v4l2_device *v4l2_dev)
{
	struct device_node *vdma_np;
	unsigned int added = 0;

	/* No notifier on purpose: remember each ctx's sensor fwnode only. */
	(void)v4l2_dev;

	for_each_compatible_node(vdma_np, NULL, "simaai,v4l2-vdma") {
		struct fwnode_handle *sensor_fw;
		u32 id;

		if (of_property_read_u32(vdma_np, "id", &id) ||
		    id >= ISP_SENSOR_MAX_CTX)
			continue;

		sensor_fw = vdma_to_sensor_fwnode(vdma_np);
		if (!sensor_fw)
			continue;

		/* vdma_to_sensor_fwnode() returns a held reference. */
		if (isp_sensor.sensor_fw[id])
			fwnode_handle_put(isp_sensor.sensor_fw[id]);
		isp_sensor.sensor_fw[id] = sensor_fw;
		added++;
		LOG_CAT( ALOG_CAT_V4L2, LOG_NOTICE, "ISP ctx %u: sensor fwnode resolved for on-demand query", id);
	}

	if (!added)
		LOG_CAT( ALOG_CAT_V4L2, LOG_INFO, "ISP: no sensor fwnodes found (file-replay only)");

	isp_sensor.registered = true;
	return 0;
}

void isp_sensor_subdev_unregister(void)
{
	int i;

	if (!isp_sensor.registered)
		return;
	for (i = 0; i < ISP_SENSOR_MAX_CTX; i++) {
		if (isp_sensor.sensor_fw[i]) {
			fwnode_handle_put(isp_sensor.sensor_fw[i]);
			isp_sensor.sensor_fw[i] = NULL;
		}
	}
	isp_sensor.registered = false;
}

void isp_sensor_subdev_set_source(u32 ctx_id, int i2c_bus)
{
	if (ctx_id >= ISP_SENSOR_MAX_CTX)
		return;
	isp_sensor.ctx_source_bus[ctx_id] = i2c_bus;
}

/* Held client for a context's DT sensor fwnode, or NULL. */
static struct i2c_client *client_for_ctx(u32 ctx)
{
	if (ctx >= ISP_SENSOR_MAX_CTX)
		return NULL;
	return client_from_fwnode(isp_sensor.sensor_fw[ctx]);
}

/* An i2c-client child of an adapter that is a sensor v4l2_subdev. */
static int match_sensor_on_bus(struct device *dev, void *data)
{
	struct i2c_client **out = data;
	struct i2c_client *client = i2c_verify_client(dev);
	struct v4l2_subdev *sd;
	int found = 0;

	if (!client)
		return 0;

	/*
	 * Deciding whether this client is a sensor means reading its private
	 * data and its subdev ops, which is the very thing unbind is free to
	 * tear down - so take the device lock first. Revalidating later cannot
	 * undo a dereference that already happened.
	 */
	device_lock(dev);
	if (!dev->driver)
		goto out;
	sd = i2c_get_clientdata(client);
	if (!sd || sd->dev != &client->dev)
		goto out;
	/* A sensor exposes pad get_fmt; on its own i2c bus it sits alone (a GMSL
	 * serializer/deserializer lives on a different adapter). */
	if (!sd->ops || !sd->ops->pad || !sd->ops->pad->get_fmt)
		goto out;

	/* Take the reference while still holding the lock, so the client the
	 * caller gets cannot have been detached in between. */
	get_device(dev);
	*out = client;
	found = 1;
out:
	device_unlock(dev);
	return found;
}

/* Live sensor subdev on i2c adapter `bus`, searched globally so it resolves
 * whether the sensor is direct-on-CSI or behind a GMSL serializer/deserializer
 * (on its own nested i2c adapter). libcamera passes the bus per context. */
static struct i2c_client *client_by_bus(int bus)
{
	struct i2c_adapter *adap;
	struct i2c_client *client = NULL;

	if (bus < 0)
		return NULL;
	adap = i2c_get_adapter(bus);
	if (!adap)
		return NULL;
	device_for_each_child(&adap->dev, &client, match_sensor_on_bus);
	i2c_put_adapter(adap);
	return client;
}

/* Held client whose sensor's active format matches w x h, or NULL. */
static struct i2c_client *client_by_format(u16 w, u16 h)
{
	int i;

	if (!w || !h)
		return NULL;
	for (i = 0; i < ISP_SENSOR_MAX_CTX; i++) {
		struct v4l2_subdev_format fmt = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE };
		struct i2c_client *client = client_from_fwnode(isp_sensor.sensor_fw[i]);
		struct v4l2_subdev *sd;
		bool match;

		if (!client)
			continue;
		sd = sensor_lock(client);
		if (!sd) {
			sensor_unlock(client, false);
			continue;
		}
		match = !v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt) &&
			fmt.format.width == w && fmt.format.height == h;
		device_unlock(&client->dev);
		if (match)
			return client;
		put_device(&client->dev);
	}
	return NULL;
}

/* Read sensor info from a subdev into out. */
/*
 * Everything below has to describe ONE mode.
 *
 * The format and the controls that qualify it - WDR mode, exposure count,
 * linearised depth - are set together by the sensor's set_fmt, and read here
 * one at a time. A format change landing in the middle yields a description
 * that never existed: one mode's geometry with another's exposure count. The
 * IPA picks a calibration from this, so a mixed answer configures the ISP for
 * a mode the sensor is not producing.
 *
 * Holding the subdev's active state lock across the reads would close that,
 * and deadlocks: this would take the state lock and then the control handler
 * lock, while a sensor's own control paths take them the other way round.
 * That is not theoretical - it hung a board.
 *
 * So take the snapshot without holding anything: read the format, read the
 * controls, read the format again, and start over if it moved. A mode change
 * is not silent, so a repeat that agrees at both ends did not straddle one.
 */
static int read_sensor_info_once(struct v4l2_subdev *sd,
				 struct isp_real_sensor_info *out,
				 struct v4l2_mbus_framefmt *seen)
{
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_ACTIVE };
	struct v4l2_ctrl *ctrl;
	u8 rggb_start, bits;
	u16 native_width;
	int ret;

	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;
	*seen = fmt.format;

	ret = mbus_to_bayer(fmt.format.code, &rggb_start, &bits);
	if (ret)
		return ret;

	memset(out, 0, sizeof(*out));
	native_width = fmt.format.width;
	/* Use the sensor's streamed width as-is; the 32-byte-aligned input
	 * constraint is enforced during V4L2 format negotiation, not here. */
	out->active_width = native_width;
	out->active_height = fmt.format.height;
	out->data_width = bits;
	out->rggb_start = rggb_start;
	out->cfa_pattern = 0;
	out->num_channels = 1;
	out->integration_time_apply_delay = 2;

	/* Multi-exposure sensors describe their active mode through these
	 * two read-only controls. A sensor that has neither is linear. */
	out->wdr_mode = MODALIX_SENSOR_WDR_MODE_LINEAR;
	out->exposures = 1;
	ctrl = v4l2_ctrl_find(sd->ctrl_handler, MODALIX_SENSOR_V4L2_CID_WDR_MODE);
	if (ctrl)
		out->wdr_mode = v4l2_ctrl_g_ctrl(ctrl);
	ctrl = v4l2_ctrl_find(sd->ctrl_handler, MODALIX_SENSOR_V4L2_CID_EXPOSURES);
	if (ctrl)
		out->exposures = clamp_val(v4l2_ctrl_g_ctrl(ctrl), 1,
					   MODALIX_SENSOR_EXPOSURES_MAX);

	/* A sensor that compands says how wide the data really is; one that
	 * does not says nothing, and the wire width is the whole truth. A
	 * value narrower than the wire would mean the sensor throws bits away
	 * on the way out, which no mode does - take the wire width instead of
	 * a shift the ISP cannot make sense of. */
	out->linearised_bits = bits;
	ctrl = v4l2_ctrl_find(sd->ctrl_handler,
			      MODALIX_SENSOR_V4L2_CID_LINEARISED_BITS);
	if (ctrl)
		out->linearised_bits = clamp_val(v4l2_ctrl_g_ctrl(ctrl), bits,
						 MODALIX_SENSOR_LINEARISED_BITS_MAX);

	out->total_height = out->active_height;
	ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_VBLANK);
	if (ctrl)
		out->total_height = out->active_height + v4l2_ctrl_g_ctrl(ctrl);

	out->total_width = native_width;
	ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_HBLANK);
	if (ctrl)
		out->total_width = native_width + v4l2_ctrl_g_ctrl(ctrl);

	ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (ctrl && out->total_width)
		out->lines_per_second =
			div_u64(v4l2_ctrl_g_ctrl_int64(ctrl), out->total_width);

	/* Exposure limits verbatim from the sensor driver, in whatever unit
	 * its V4L2_CID_EXPOSURE uses (rows on mainline drivers, microseconds
	 * on the econ MCU drivers). The IPA owns the unit contract
	 * (v4l2_exposure_unit in the tuning YAML) and converts to the rows
	 * the 3A consumes — see ipa_sensor_integration_limits(). */
	ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_EXPOSURE);
	if (ctrl) {
		out->integration_time_min = ctrl->minimum;
		out->integration_time_max = ctrl->maximum;
	}

	return 0;
}

static int read_sensor_info(struct v4l2_subdev *sd,
			    struct isp_real_sensor_info *out)
{
	struct v4l2_mbus_framefmt before;
	unsigned int attempt;
	int ret;

	/*
	 * Take the whole snapshot twice and require the two to agree. A sensor
	 * whose modes differ only in the exposure count - the AR0822's two-
	 * and three-exposure eHDR modes carry the same geometry and the same
	 * media-bus code - changes neither the format nor the resolution when
	 * it switches, so the format alone cannot qualify the snapshot.
	 *
	 * Reading the qualifying controls a second way rather than a second
	 * time is what makes this fragile: the fallbacks have to agree exactly
	 * with the ones above, and a sensor that has no such control then
	 * compares a derived value against a different default and never
	 * settles. Run the same function twice instead, so there is one set of
	 * rules.
	 */
	for (attempt = 0; attempt < 3; attempt++) {
		struct isp_real_sensor_info again;
		struct v4l2_mbus_framefmt before_again;

		ret = read_sensor_info_once(sd, out, &before);
		if (ret)
			return ret;

		ret = read_sensor_info_once(sd, &again, &before_again);
		if (ret)
			return ret;

		/* Every field, not a chosen few: the snapshot also samples
		 * blanking, pixel rate and the exposure limits through their
		 * own control reads, so a VBLANK change alone can pair an old
		 * frame length with a new exposure maximum. read_sensor_info_once()
		 * zeroes the whole structure before filling it, padding
		 * included, so comparing the bytes is well defined. */
		if (before.code == before_again.code &&
		    before.width == before_again.width &&
		    before.height == before_again.height &&
		    !memcmp(out, &again, sizeof(*out)))
			return 0;
	}

	LOG_CAT( ALOG_CAT_V4L2, LOG_ERR,
	    "sensor '%s': the format kept changing while it was being read",
	    sd->name);
	return -EAGAIN;
}

/*
 * Resolve the sensor feeding @ctx_id, the same way a query does: the
 * libcamera-selected source bus first, then this context's DT sensor.
 * Format matching is deliberately not used here - writing a control to a
 * sensor picked by "something that size" would be a guess, and a guess that
 * reprograms a sensor is worse than a failure.
 */
static struct i2c_client *client_for_write(u32 ctx_id)
{
	struct i2c_client *client;

	if (ctx_id >= ISP_SENSOR_MAX_CTX)
		return NULL;

	client = client_by_bus(isp_sensor.ctx_source_bus[ctx_id]);
	if (!client)
		client = client_for_ctx(ctx_id);

	return client;
}

int isp_sensor_subdev_set_exposures(u32 ctx_id, u32 exposures)
{
	struct i2c_client *client = client_for_write(ctx_id);
	struct v4l2_subdev *sd;
	struct v4l2_ctrl *ctrl;
	int ret;

	if (!client)
		return -ENODEV;

	sd = sensor_lock(client);
	if (!sd) {
		sensor_unlock(client, false);
		return -ENODEV;
	}

	ctrl = v4l2_ctrl_find(sd->ctrl_handler,
			      MODALIX_SENSOR_V4L2_CID_EXPOSURES);
	if (!ctrl) {
		sensor_unlock(client, true);
		return -ENOTTY;
	}

	ret = v4l2_ctrl_s_ctrl(ctrl, exposures);
	if (ret)
		LOG_CAT( ALOG_CAT_V4L2, LOG_NOTICE,
		    "ISP ctx %u sensor '%s': refused %u exposures (%d)",
		    ctx_id, sd->name, exposures, ret);
	else
		LOG_CAT( ALOG_CAT_V4L2, LOG_NOTICE,
		    "ISP ctx %u sensor '%s': now combining %u exposures",
		    ctx_id, sd->name, exposures);

	sensor_unlock(client, true);
	return ret;
}

int isp_sensor_subdev_query(u32 ctx_id, u16 want_w, u16 want_h,
			    struct isp_real_sensor_info *out)
{
	struct i2c_client *client;
	struct v4l2_subdev *sd;
	int ret;

	if (ctx_id >= ISP_SENSOR_MAX_CTX)
		return -EINVAL;

	/* Prefer the libcamera-selected source bus, then this context's DT
	 * sensor, then any sensor whose active format matches. */
	client = client_by_bus(isp_sensor.ctx_source_bus[ctx_id]);
	if (!client)
		client = client_for_ctx(ctx_id);
	if (!client)
		client = client_by_format(want_w, want_h);
	if (!client)
		return -ENODEV;

	sd = sensor_lock(client);
	if (!sd) {
		sensor_unlock(client, false);
		return -ENODEV;
	}

	ret = read_sensor_info(sd, out);
	if (ret) {
		sensor_unlock(client, true);
		return ret;
	}

	LOG_CAT( ALOG_CAT_V4L2, LOG_NOTICE,
	    "ISP ctx %u sensor '%s': %ux%u total %ux%u %u-bit rggb%u lps %u",
	    ctx_id, sd->name, out->active_width, out->active_height,
	    out->total_width, out->total_height, out->data_width,
	    out->rggb_start, out->lines_per_second);
	if (out->linearised_bits != out->data_width)
		LOG_CAT( ALOG_CAT_V4L2, LOG_NOTICE,
		    "ISP ctx %u sensor '%s': companded %u-bit wire carries %u-bit data",
		    ctx_id, sd->name, out->data_width, out->linearised_bits);

	sensor_unlock(client, true);
	return 0;
}
