/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DMABUF Heaps Allocation Infrastructure
 *
 * Copyright (C) 2011 Google, Inc.
 * Copyright (C) 2019 Linaro Ltd.
 */

#ifndef _DMA_HEAPS_H
#define _DMA_HEAPS_H

#include <linux/types.h>

struct dma_buf;
struct dma_heap;

/**
 * struct dma_heap_ops - ops to operate on a given heap
 * @allocate:		allocate dmabuf and return struct dma_buf ptr
 * @allocated_bytes:	optional; bytes currently allocated from this heap
 *
 * allocate returns dmabuf on success, ERR_PTR(-errno) on error.
 *
 * Only the heap can report occupancy for a shared region: consumers allocate
 * through the fd, so no one consumer sees what the others hold.
 */
struct dma_heap_ops {
	struct dma_buf *(*allocate)(struct dma_heap *heap,
				    unsigned long len,
				    u32 fd_flags,
				    u64 heap_flags);
	u64 (*allocated_bytes)(struct dma_heap *heap);
};

/**
 * struct dma_heap_export_info - information needed to export a new dmabuf heap
 * @name:	used for debugging/device-node name
 * @ops:	ops struct for this heap
 * @priv:	heap exporter private data
 *
 * Information needed to export a new dmabuf heap.
 */
struct dma_heap_export_info {
	const char *name;
	const struct dma_heap_ops *ops;
	void *priv;
};

void *dma_heap_get_drvdata(struct dma_heap *heap);

const char *dma_heap_get_name(struct dma_heap *heap);

u64 dma_heap_allocated_bytes(struct dma_heap *heap);

struct dma_heap *dma_heap_add(const struct dma_heap_export_info *exp_info);

struct dma_heap *dma_heap_find(const char *name);

struct dma_buf *dma_heap_buffer_alloc(struct dma_heap *heap, size_t len,
				      u32 fd_flags, u64 heap_flags);

#endif /* _DMA_HEAPS_H */
