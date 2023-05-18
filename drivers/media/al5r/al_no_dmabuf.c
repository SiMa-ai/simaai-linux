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

