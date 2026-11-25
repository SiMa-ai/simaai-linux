/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SIMAAI_MLA_MEMORY_H
#define SIMAAI_MLA_MEMORY_H

#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/scatterlist.h>

/* DDR descriptor addresses, not the wider host dma_addr_t or MIM register. */
#define MLA_DMA_ADDRESS_LIMIT	(1ULL << 40)
#define MLA_SG_MAX_LENGTH	(UINT_MAX & PAGE_MASK)

static inline int mla_memory_size(size_t size, size_t *aligned)
{
	if (!size || size > SIZE_MAX - (PAGE_SIZE - 1))
		return -EINVAL;
	*aligned = PAGE_ALIGN(size);
	return 0;
}

static inline int mla_dma_range_check(u64 address, u64 size, u64 alignment)
{
	if (!size || !address || !alignment || (alignment & (alignment - 1)) ||
	    (address & (alignment - 1)))
		return -EINVAL;
	if (address >= MLA_DMA_ADDRESS_LIMIT ||
	    size > MLA_DMA_ADDRESS_LIMIT - address)
		return -ERANGE;
	return 0;
}

/* Multiple SG entries may describe one contiguous interval (e.g. >4 GiB).
 * Do not accept gaps, overlaps, zero-length entries or a truncated exporter.
 * Validate every mapped entry, not only the prefix the caller plans to use.
 */
static inline int mla_sg_dma_address(const struct sg_table *sgt, u64 size,
				     u64 alignment, dma_addr_t *address)
{
	struct scatterlist *sg;
	u64 first, next, total = 0;
	unsigned int i;
	int ret;

	if (!sgt || !sgt->sgl || !sgt->nents ||
	    sgt->nents > sgt->orig_nents || !address)
		return -EINVAL;
	first = sg_dma_address(sgt->sgl);
	ret = mla_dma_range_check(first, size, alignment);
	if (ret)
		return ret;
	next = first;
	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		u64 length;

		if (!sg)
			return -EINVAL;
		length = sg_dma_len(sg);
		if (!length || sg_dma_address(sg) != next)
			return -EINVAL;
		if (check_add_overflow(total, length, &total) ||
		    check_add_overflow(next, length, &next) ||
		    next > MLA_DMA_ADDRESS_LIMIT)
			return -ERANGE;
	}
	if (total < size)
		return -EINVAL;
	*address = first;
	return 0;
}

/* SG lengths are unsigned int, even on a 64-bit host. */
static inline unsigned int mla_sg_segment_length(u64 remaining)
{
	return min_t(u64, remaining, MLA_SG_MAX_LENGTH);
}

static inline int mla_sg_from_contiguous_pages(struct sg_table *sgt,
					     struct page *first, size_t size)
{
	struct scatterlist *sg;
	u64 count = size / MLA_SG_MAX_LENGTH + !!(size % MLA_SG_MAX_LENGTH);
	size_t offset = 0;
	unsigned int i;
	int ret;

	if (!first || !size || !PAGE_ALIGNED(size) || count > UINT_MAX)
		return -EINVAL;
	ret = sg_alloc_table(sgt, count, GFP_KERNEL);
	if (ret)
		return ret;
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		unsigned int length = mla_sg_segment_length(size - offset);

		sg_set_page(sg, pfn_to_page(page_to_pfn(first) +
					 (offset >> PAGE_SHIFT)), length, 0);
		offset += length;
	}
	return 0;
}

#endif /* SIMAAI_MLA_MEMORY_H */
