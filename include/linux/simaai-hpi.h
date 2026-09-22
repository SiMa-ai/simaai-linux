/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2026 SiMa ai
 */
#ifndef __LINUX_SOC_SIMAAI_HPI_H__
#define __LINUX_SOC_SIMAAI_HPI_H__

#include <linux/types.h>
#include <uapi/linux/simaai/simaai-hpi.h>

/* controller <-> client wire message */
struct simaai_hpi_mbox_msg {
	u32 p0;
	u32 p1;
};

/* in-kernel API for other drivers */
int simaai_hpi_xfer(const struct simaai_hpi_msg *req, struct simaai_hpi_msg *rsp,
		    const void *payload, size_t len);

#endif  /* __LINUX_SIMAAI_HPI_H__ */
