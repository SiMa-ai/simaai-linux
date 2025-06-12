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

#include "system_isp_io.h"
#include "acamera_logger.h"

#include <linux/simaai-stu.h>
#include <asm/io.h>


#define ISP_PHYS_ADDRESS_SPACE_SIZE (0x1FFFF)
#define ISP_CDMA_ADDRESS_SPACE_SIZE	(128*1024*1024)

extern struct simaai_stu *stu;

struct isp_mapped_mem {

	dma_addr_t reg_phys_addr;
	dma_addr_t reg_bus_addr;
	u32 reg_size;
	void __iomem *reg_addr;
	dma_addr_t cdma_phys_addr;
	dma_addr_t cdma_bus_addr;
	u32 cdma_size;
	void __iomem *cdma_addr;
};

static struct isp_mapped_mem isp_mem;

/**
 *   Initialize access to ISP I/O memory.
 *
 *   This function initialize access to ISP I/O memory.
 *
 *   @return 0 on success
 */
int32_t system_isp_init( void )
{
	int rc = -1;

	isp_mem.reg_phys_addr = PHY_ADDR_ISP;
	isp_mem.reg_bus_addr = 0;
	isp_mem.reg_size = ISP_PHYS_ADDRESS_SPACE_SIZE;
	isp_mem.cdma_phys_addr = PHY_ADDR_CDMA;
	isp_mem.cdma_bus_addr = 0;
	isp_mem.cdma_size = ISP_CDMA_ADDRESS_SPACE_SIZE;
	

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return -1;
	}

	rc = simaai_stu_get_bus_address(stu, isp_mem.reg_phys_addr, &isp_mem.reg_bus_addr);
	if (rc != 0) {
		LOG ( LOG_CRIT, "ERROR : getting bus address for %#llx\n", isp_mem.reg_phys_addr);
		// ASSERT if possible
		return -1;
	}

	LOG (LOG_INFO, "STU : successfully mapped reg address space phys to bus : %#llx -> %#llx",
					isp_mem.reg_phys_addr, isp_mem.reg_bus_addr);

	isp_mem.reg_addr = ioremap(isp_mem.reg_phys_addr, isp_mem.reg_size);
	if (!isp_mem.reg_addr) {
		LOG( LOG_CRIT, "Failed to map register address space %#llx, size : %#x",
								isp_mem.reg_phys_addr, isp_mem.reg_size);
		return -1;
	}

	LOG (LOG_INFO, "isp reg phy address %#llx remaped to %#llx",
					isp_mem.reg_phys_addr, isp_mem.reg_addr);

	rc = simaai_stu_get_bus_address(stu, isp_mem.cdma_phys_addr, &isp_mem.cdma_bus_addr);
	if (rc != 0) {
		LOG ( LOG_CRIT, "ERROR : getting bus address for %#llx\n", isp_mem.cdma_phys_addr);
		return 0;
	}

	LOG (LOG_INFO, "STU : successfully mapped phys to bus: %#llx -> %#llx",
					isp_mem.cdma_phys_addr, isp_mem.cdma_bus_addr);

	isp_mem.cdma_addr = ioremap(isp_mem.cdma_phys_addr, isp_mem.cdma_size);
	if (!isp_mem.cdma_addr) {
		LOG( LOG_CRIT, "Failed to map cdma address space %#llx, size : %#x",
								isp_mem.cdma_phys_addr, isp_mem.cdma_size);
		return -1;
	}

	LOG (LOG_INFO, "isp reg phy address %#llx rempaed to %#llx",
					isp_mem.cdma_phys_addr, isp_mem.cdma_addr);

	return 0;
}

/**
 *   DeInitialize access to ISP I/O memory.
 *
 *   This function DeInitialize access to ISP I/O memory.
 *
 */
void system_isp_deinit( void )
{
	
	iounmap(isp_mem.reg_addr);
	iounmap(isp_mem.cdma_addr);

	isp_mem.reg_phys_addr = 0;
	isp_mem.reg_bus_addr = 0;
	isp_mem.reg_size = 0;
	isp_mem.reg_phys_addr = 0;
	isp_mem.reg_bus_addr = 0;
	isp_mem.cdma_size = 0;

}

uint8_t system_isp_mem_read( void *dst, uint32_t src, uint32_t elem, uint8_t elemSz )
{

	LOG (LOG_CRIT, "ISP mem read not implemented !!!");
    return 0;
}

uint32_t system_isp_read_32( uint32_t addr )
{
	uint32_t val;
#if 0

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return 0;
	}

	int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
	if (ret != 0) {
		LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
		// ASSERT if possible
		return 0;
	}

	void __iomem *ioaddr = ioremap(dma_addr, 0x4);
	val = readl(ioaddr);
	iounmap(ioaddr);

#endif

	if ((addr >= isp_mem.reg_bus_addr) && (addr < (isp_mem.reg_bus_addr + isp_mem.reg_size))) {
		val = readl(isp_mem.reg_addr + (addr - isp_mem.reg_bus_addr));
	} else if ((addr >= isp_mem.cdma_bus_addr) && (addr < (isp_mem.cdma_bus_addr + isp_mem.cdma_size))) {
		val = readl(isp_mem.cdma_addr + (addr - isp_mem.cdma_bus_addr));
	} else {

		LOG (LOG_ERR, "No Match found for %#x", addr);
		dma_addr_t dma_addr;

		if (!stu) {
			LOG ( LOG_CRIT, "STU is not initialized");
			return 0;
		}

		int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
		if (ret != 0) {
			LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
			// ASSERT if possible
			return 0;
		}

		void __iomem *ioaddr = ioremap(dma_addr, 0x4);
		val = readl(ioaddr);
		iounmap(ioaddr);

	}

    return val;
}

uint16_t system_isp_read_16( uint32_t addr )
{
	dma_addr_t dma_addr;
	uint16_t val;

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return 0;
	}

	int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
	if (ret != 0) {
		LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
		// ASSERT if possible
		return 0;
	}

	void __iomem *ioaddr = ioremap(dma_addr, 0x4);
	val = readw(ioaddr);
	iounmap(ioaddr);

    return val;
}

uint8_t system_isp_read_8( uint32_t addr )
{
	dma_addr_t dma_addr;
	uint8_t val;

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return 0;
	}

	int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
	if (ret != 0) {
		LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
		// ASSERT if possible
		return 0;
	}

	void __iomem *ioaddr = ioremap(dma_addr, 0x4);
	val = readb(ioaddr);
	iounmap(ioaddr);

    return val;
}

void system_isp_write_32( uint32_t addr, uint32_t data )
{

	if ((addr >= isp_mem.reg_bus_addr) && (addr < (isp_mem.reg_bus_addr + isp_mem.reg_size))) {
		writel(data, isp_mem.reg_addr + (addr - isp_mem.reg_bus_addr));
	} else if ((addr >= isp_mem.cdma_bus_addr) && (addr < (isp_mem.cdma_bus_addr + isp_mem.cdma_size))) {
		writel(data, isp_mem.cdma_addr + (addr - isp_mem.cdma_bus_addr));
	} else {
		LOG( LOG_ERR, "No match for %#lx", addr);
		dma_addr_t dma_addr;

		if (!stu) {
			LOG ( LOG_CRIT, "STU is not initialized");
			return;
		}

		int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
		if (ret != 0) {
			LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
			// ASSERT if possible
			return;
		}

		void __iomem *ioaddr = ioremap(dma_addr, 0x4);
	
    	writel(data, ioaddr);
		iounmap(ioaddr);
	}
}

void system_isp_write_16( uint32_t addr, uint16_t data )
{
	dma_addr_t dma_addr;

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return;
	}

	int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
	if (ret != 0) {
		LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
		// ASSERT if possible
		return;
	}

	void __iomem *ioaddr = ioremap(dma_addr, 0x4);
	
    writew(data, ioaddr);
	iounmap(ioaddr);

}

void system_isp_write_8( uint32_t addr, uint8_t data )
{
	dma_addr_t dma_addr;

	if (!stu) {
		LOG ( LOG_CRIT, "STU is not initialized");
		return;
	}

	int ret = simaai_stu_get_dev_address(stu, addr, &dma_addr);
	if (ret != 0) {
		LOG ( LOG_CRIT, "ERROR : getting device address for %#x\n", addr);
		// ASSERT if possible
		return;
	}

	void __iomem *ioaddr = ioremap(dma_addr, 0x4);
	
    writeb(data, ioaddr);
	iounmap(ioaddr);

}
