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

#include "sensor_api.h"  
#include "acamera_logger.h"

#include <linux/of_graph.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

extern struct platform_device *g_pdev;

// Extern functions for dummy sensor
extern void sensor_init_dummy( void **priv, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const );
// Extern functions for imx477 sensor
extern void sensor_init_imx477( void **priv, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const );
extern void sensor_init_imx678( void **priv, uint8_t location, sensor_control_t *ctrl, const sensor_options_t *const );

struct sensor_info {
	int sensor_id;
	void (*sensor_driver_init) (void **, uint8_t, sensor_control_t *, const sensor_options_t *const);
};

struct sensor_info sensors[] = {

	{0, sensor_init_dummy},
	{1, sensor_init_imx477},
	{2, sensor_init_imx678},
};

uint32_t get_sensor_id(uint32_t ctx_id) {

	struct device_node *port;
	struct device_node *node;
	uint32_t sensor_id = 0;
	int rc = -EINVAL;

	port = of_graph_get_port_by_id(g_pdev->dev.of_node, ctx_id);
	if (!port) {
		LOG (LOG_ERR, "failed to get port by ctx id : %u", ctx_id);
		return 0;
	}

    for_each_child_of_node(port, node) {
		 if (of_find_property(node, "sensor-id", NULL)) {
			rc = of_property_read_u32(node, "sensor-id", &sensor_id);
			if (rc != 0) {
				LOG (LOG_ERR, "ERROR : getting sensor-id for node %s", node->name);
				sensor_id = 0;
				break;
			}
			LOG (LOG_DEBUG, "SUCCESS : Sensor id is %d", sensor_id);
			break;
		}
	}

    of_node_put(port);
    of_node_put(node);

	return sensor_id;
}

int8_t get_dma_index(uint32_t ctx_id) {

	struct device_node *port;
	struct device_node *node;
	int32_t dma_index = ctx_id;
	int rc = -EINVAL;

	port = of_graph_get_port_by_id(g_pdev->dev.of_node, ctx_id);
	if (!port) {
		LOG (LOG_ERR, "failed to get port by ctx id : %u", ctx_id);
		return -EINVAL;
	}

    for_each_child_of_node(port, node) {
		 if (of_find_property(node, "dma-index", NULL)) {
			rc = of_property_read_u32(node, "dma-index", &dma_index);
			if (rc != 0) {
				LOG (LOG_ERR, "ERROR : getting dma-index for node %s", node->name);
				dma_index = ctx_id;
				break;
			}
			LOG (LOG_DEBUG, "SUCCESS : dma index is %d", dma_index);
			break;
		}
	}

    of_node_put(port);
    of_node_put(node);

	return dma_index;
}

void sensor_init_common( void **priv_ptr, uint8_t location, sensor_control_t *ctrl,
							const sensor_options_t *const options) {

	int iter = 0;
	int sensor_id = get_sensor_id(location);

	for (iter = 0; iter < sizeof(sensors)/sizeof(sensors[0]); iter++ ) {

		if(sensors[iter].sensor_id == sensor_id) {
			sensors[iter].sensor_driver_init(priv_ptr, location, ctrl, options);
			return;
		}
	}

	LOG (LOG_ERR, "No match found for sensor id falling back to dummy");
	sensor_init_dummy(priv_ptr, location, ctrl, options);
}
