/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2026 SiMa.ai
 *
 * Kernel-internal image-format constants shared by the model-load and
 * job-submit paths. MLA_OP_MODEL_DEFINE_V3 validates against these
 * numeric constants independently of any wire-format struct layout.
 */
#ifndef _MLA_IMAGE_FMT_H_
#define _MLA_IMAGE_FMT_H_

#include <linux/types.h>

#define MLA_DMA_UNIT_COUNT_SIZE         4

#endif /* _MLA_IMAGE_FMT_H_ */
