/* SPDX-License-Identifier: (GPL-2.0+ OR MIT)
 *
 * Copyright (c) 2021 sima.ai
 *
 * Author: Nilesh Raghuvanshi <nilesh.r@sima.ai>
 */

#ifndef _SIMAAI_STU_H_
#define _SIMAAI_STU_H_

struct simaai_stu *simaai_stu_get_by_phandle(struct device_node *np, const char *phandle_name);

int simaai_stu_get_bus_address(struct simaai_stu *stu, dma_addr_t phys_addr, dma_addr_t *bus_addr);

int simaai_stu_get_dev_address(struct simaai_stu *stu, dma_addr_t bus_addr, dma_addr_t *dev_addr);

#endif
