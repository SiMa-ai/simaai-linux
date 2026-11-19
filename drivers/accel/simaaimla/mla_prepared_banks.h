/* SPDX-License-Identifier: GPL-2.0 */
#ifndef SIMAAI_MLA_PREPARED_BANKS_H
#define SIMAAI_MLA_PREPARED_BANKS_H

#include <linux/errno.h>
#include <linux/stddef.h>
#include <linux/types.h>
#include <linux/simaai/mla.h>

/* req is zero-filled before copying its optional tail. Reject a partial new
 * tail rather than interpreting a truncated count as successful admission.
 * Existing shorter requests and unknown complete future tails remain valid.
 */
static inline int mla_prepared_banks_validate(size_t req_len, u32 count,
					     u32 reserved, u32 maximum)
{
	if (req_len > offsetof(struct mla_model_define_v3_req,
			       prepared_bank_count) &&
	    req_len < offsetofend(struct mla_model_define_v3_req,
				 prepared_reserved))
		return -EINVAL;
	if (reserved || count > maximum)
		return -EINVAL;
	return 0;
}

#endif
