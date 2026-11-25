/******************************************************************************
*
* Copyright (C) 2019 Allegro DVT2.  All rights reserved.
*
******************************************************************************/

#include "al_dmabuf.h"

int al5_create_dmabuf_fd(struct device *dev, unsigned long size,
			 struct al5_dma_buffer *buffer)
{
	pr_err("dmabuf interface not supported");
	return -EINVAL;
}

int al5_allocate_dmabuf(struct device *dev, int size, u32 *fd)
{
	pr_err("dmabuf interface not supported");
	return -EINVAL;
}

int al5_dmabuf_get_address(struct device *dev, u32 fd, u32 *bus_address)
{
	pr_err("dmabuf interface not supported");
	return -EINVAL;
}

int al5_dmabuf_import_address(struct al5r_codec_chan *chan, struct device *dev,
			      u32 fd, dma_addr_t *bus_address)
{
	pr_err("dmabuf interface not supported");
	return -EINVAL;
}

struct al5_import_set *al5_dmabuf_alloc_import_set(struct device *dev)
{
	return NULL;
}

struct al5_import_set *al5_dmabuf_get_import_set(struct al5r_codec_chan *chan)
{
	return NULL;
}

void al5_dmabuf_put_import_set(struct al5_import_set *set)
{
}

