/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2018-2019 Synopsys, Inc. and/or its affiliates.
 *
 * Synopsys DesignWare MIPI CSI-2 platform data
 *
 * Author: Luis Oliveira <Luis.Oliveira@synopsys.com>
 */

#include <linux/kernel.h>

#ifndef DW_PDATA_NAMES_DEFINED
#define DW_PDATA_NAMES_DEFINED
struct pdata_names {
	const char *name;
};
#endif

static const struct pdata_names csis[] = {
	{ .name = "dw-csi.0", },
	{ .name = "dw-csi.1", },
};
