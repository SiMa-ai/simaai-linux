// SPDX-License-Identifier: GPL-2.0
/*
 * Platform PCIe EP driver for SiMa.ai SoCs
 *
 * Copyright (C) 2024 SiMa.ai
 *
 */

#ifndef __SIMA_PEP_DRV_MOD_H__
#define __SIMA_PEP_DRV_MOD_H__

#include "simaai_pep_drv.h"

#define SI_MLSOC_CAPABILITIES_0		0x10
#define SI_MLSOC_CAPABILITIES_1		0x14
#define SI_MLSOC_CAPABILITIES_2		0x18
#define SI_MLSOC_CAPABILITIES_3		0x1C
#define SI_MLSOC_CAPABILITIES_4		0x20
#define SI_MLSOC_CAPABILITIES_5		0x24
#define SI_MLSOC_CAPABILITIES_6		0x28
#define SI_MLSOC_CAPABILITIES_7		0x2C

struct si_mod_mlsoc_caps_0 {
	uint32_t tops:2;
	uint32_t max_mqs:1;
	uint32_t max_dqs:3;
	uint32_t max_cameras:3;
	uint32_t max_mipi_int:3;
	uint32_t encoder:1;
	uint32_t decoder:1;
	uint32_t rc_mode:1;
	uint32_t daisy_chain:1;
	uint32_t reserved:16;
};
STOP_BUILD_IF_NOT(sizeof(struct si_mod_mlsoc_caps_0) <= 4);

extern const struct si_pep_drv_ops modalix_drv_ops;

#endif
