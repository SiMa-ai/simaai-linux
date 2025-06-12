/*
*
* SPDX-License-Identifier: GPL-2.0
*
* Copyright (C) 2011-2021 ARM or its affiliates
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2.
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*/

#include "system_stdlib.h"
#include "acamera_logger.h"

#include <asm/io.h>
#include "linux/string.h"
#include "linux/slab.h"

#include <linux/simaai-stu.h>

extern struct simaai_stu *stu;

int32_t system_memcpy( void *dst, const void *src, uint32_t size )
{
    int32_t result = 0;
    memcpy( dst, src, size );
    return result;
}


int32_t system_memset( void *ptr, uint8_t value, uint32_t size )
{
    int32_t result = 0;
    memset( ptr, value, size );
    return result;
}


int32_t system_memcpy_phy2phy( uint32_t dst_addr, uint32_t src_addr, uint32_t size )
{
    int32_t result = 0;
	dma_addr_t dst_add = 0;
	dma_addr_t src_add = 0;

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return 0;
	}

	int ret = simaai_stu_get_dev_address(stu, dst_addr, &dst_add);
	if (ret != 0) {
		LOG ( LOG_CRIT, "error : getting device address for %#x\n", dst_addr);
		// assert if possible
		return 0;
	}

	ret = simaai_stu_get_dev_address(stu, src_addr, &src_add);
	if (ret != 0) {
		LOG ( LOG_CRIT, "error : getting device address for %#x\n", src_addr);
		// assert if possible
		return 0;
	}

    void *dst = ioremap( dst_add, size );
    void *src = ioremap( src_add, size );

    memcpy( dst, src, size );

    iounmap( src );
    iounmap( dst );

    return result;
}

int32_t system64_memcpy_phy2phy( uint64_t dst_addr, uint64_t src_addr, uint32_t size )
{
    int32_t result = 0;
#if 1
    void *buf = NULL , *dst = NULL , *src = NULL;
	uint32_t paddr;

    buf = kzalloc(size, GFP_KERNEL);
    if (!buf) {
	result = -ENOMEM;
	return result;
    }

    dst = ioremap( dst_addr, size );
    src = ioremap( src_addr, size );

    if (src == NULL || dst == NULL) {
	result = -ENOMEM;
	return result;
    }

    memcpy_fromio(buf, src, size);
    memcpy_toio(dst, buf, size);
    kfree(buf);

    iounmap( src );
    iounmap( dst );

    return result;
#endif
#if 0
    int32_t result = 0;

    void *dst = ioremap( dst_addr, size );
    void *src = ioremap( src_addr, size );

    memcpy( dst, src, size );

    iounmap( src );
    iounmap( dst );

    return result;
#endif

}


int32_t system_memcpy_vir2phy( uint32_t dst_addr, void *src, uint32_t size )
{
    int32_t result = 0;
	dma_addr_t dst_add = 0;

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return 0;
	}

	int ret = simaai_stu_get_dev_address(stu, dst_addr, &dst_add);
	if (ret != 0) {
		LOG ( LOG_CRIT, "memcpy v2p : error : getting device address for %#x\n", dst_addr);
		// assert if possible
		return 0;
	}

    void *dst = ioremap(dst_add, size );

    memcpy( dst, src, size );

    iounmap( dst );

    return result;
}

int32_t system64_memcpy_vir2phy( uint64_t dst_addr, void *src, uint32_t size )
{
    int32_t result = 0;

    void *dst = ioremap( dst_addr, size );

    if (dst == NULL) {
        result = -ENOMEM;
        return result;
    }   

    memcpy_toio( dst, src, size );

    iounmap( dst );

    return result;
}

