// SPDX-License-Identifier: GPL-2.0
/*
 * Platform PCIe EP driver for SiMa.ai SoCs
 *
 * Copyright (C) 2024 SiMa.ai
 *
 */

#ifndef __SIMA_PEP_DRV_DAV_H__
#define __SIMA_PEP_DRV_DAV_H__

#include "simaai_pep_drv.h"

#define SIMA_DAV_MAX_MQES       128
#define SIMA_DAV_MAX_DQS        6
/* Offset of DMA registers drom dbi2 base */
#define SIMA_DAV_PF0_DMA_REGS	0x280000

extern const struct si_pep_drv_ops davinci_drv_ops;

#endif
