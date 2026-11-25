/******************************************************************************
*
* Copyright (C) 2019 Allegro DVT2.  All rights reserved.
*
******************************************************************************/

#include <linux/device.h>
#include "al_alloc.h"

struct al5_buffer_info {
	dma_addr_t bus_address;
	u32 size;
};

int al5_create_dmabuf_fd(struct device *dev, size_t size, struct al5_dma_buffer *buffer);
int al5_allocate_dmabuf(struct device *dev, size_t size, u32 *fd);
int al5_allocate_dmabuf_cached(struct device *dev, size_t size, u32 *fd);
int al5_dmabuf_get_address(struct device *dev, u32 fd, dma_addr_t *bus_address);

struct al5r_codec_chan;
struct al5_import_set;

/* Resolve fd to a bus address and RETAIN the attachment and mapping in the
 * channel's import set.  Repeat lookups of the same buffer share one mapping.
 */
int al5_dmabuf_import_address(struct al5r_codec_chan *chan, struct device *dev,
			      u32 fd, dma_addr_t *bus_address);

struct al5_import_set *al5_dmabuf_alloc_import_set(struct device *dev);
/* Take an extra reference, for work that outlives the channel. */
struct al5_import_set *al5_dmabuf_get_import_set(struct al5r_codec_chan *chan);
/* Drop a reference; the last one unmaps, detaches and releases everything. */
void al5_dmabuf_put_import_set(struct al5_import_set *set);

