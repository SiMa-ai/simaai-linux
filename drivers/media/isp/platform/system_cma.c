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

#include "system_cma.h"
#include "system_assert.h"
#include "acamera_logger.h"

#include <linux/dma-mapping.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/simaai-stu.h>

#define MAX_NODES 2048

typedef struct {
    dma_addr_t paddr;
    void *vaddr;
    size_t size;
} node_t;

static node_t nodes[MAX_NODES] = {0};
static DEFINE_MUTEX( ddr_alloc_lock );

// Filled/used by main.c
struct platform_device *g_pdev = NULL;
size_t g_resmem_remaining = 0;
size_t g_resmem_nodes_allocated = 0;

static size_t resmem_used = 0;

extern struct simaai_stu *stu;

// Not in API
static inline void system_cma_update_tracking_info( size_t size, bool alloc )
{
    // Linux kernel rounds size to the next power of two, with a minimum of PAGE_SIZE
    size_t actual_size = 1ul << ( get_order( size ) + PAGE_SHIFT );
    if ( alloc ) {
        g_resmem_remaining -= actual_size;
        resmem_used += actual_size;
    } else {
        g_resmem_remaining += actual_size;
        resmem_used -= actual_size;
    }
}

// Not in API. Used by main.c
void system_cma_print_tracking_info( void )
{
    LOG( LOG_NOTICE, "system_cma: used = 0x%zx (%zu MiB), remaining = 0x%zx (%zu MiB); %zu node(s)", resmem_used, resmem_used >> 20, g_resmem_remaining, g_resmem_remaining >> 20, g_resmem_nodes_allocated );
}

/**
 * @brief      Allocates mem on ddr
 *
 * @param[out] out_virtaddr  pointer to the virtual address pointer. Optional, can be NULL.
 * @param[out] out_physaddr  pointer to the physical address address pointer. Optional, can be NULL.
 * @param[in]  size          size of mem in bytes to allocate
 *
 * @return     Returns 0 on success, < 0 on failure
 */
int system_cma_alloc( void **out_virtaddr, uint32_t *out_physaddr, uint32_t size )
{
    assert( size != 0 );
    node_t node = {0};
    dma_addr_t paddr;
    void *vaddr = NULL;

	LOG ( LOG_INFO, "size of the alloc is %d", size);
    mutex_lock( &ddr_alloc_lock );
    if ( g_resmem_nodes_allocated < MAX_NODES ) {
        vaddr = dma_alloc_coherent( &g_pdev->dev, size, &node.paddr, GFP_KERNEL );
    } else {
		LOG (LOG_ERR, "Ran out of nodes");
	}

    if ( vaddr != NULL ) {
        node.vaddr = vaddr;
        node.size = size;
        nodes[g_resmem_nodes_allocated++] = node;
		if (stu) {
			int res = simaai_stu_get_bus_address(stu, node.paddr, &paddr);
			if (res != 0) {
				LOG (LOG_CRIT, "ERROR : getting bus address for phys address, %#llx", node.paddr);
				return -EFAULT;
			}
		} else {
	        paddr = node.paddr;
		}

		LOG ( LOG_INFO, "phys Address : %#llx, buss Address %#x", node.paddr, paddr);
        system_cma_update_tracking_info( size, true );
        system_cma_print_tracking_info();
    } else {
        LOG( LOG_CRIT, "system_cma_alloc(size 0x%x): failed", size );
        system_cma_print_tracking_info();
        paddr = 0;
    }
    mutex_unlock( &ddr_alloc_lock );

    if ( out_virtaddr != NULL ) {
        *out_virtaddr = vaddr;
    }
    if ( out_physaddr != NULL ) {
        *out_physaddr = (uint32_t)paddr;
    }

    return vaddr == NULL ? -1 : 0;
}

/**
 * @brief      Deallocates mem on ddr(allocated with system_cma_alloc)
 *
 * @param      addr  address of start of mem to dealocated
 *
 * @details    Deallocates mem on ddr(allocated with system_cma_alloc). Address has to be allocated.
 */
void system_cma_free( uint32_t addr )
{
    int i;
    dma_addr_t paddr = addr;
    size_t size = 0;

    mutex_lock( &ddr_alloc_lock );
    for ( i = 0; i < g_resmem_nodes_allocated; i++ ) {
        if ( nodes[i].paddr == paddr ) {
            size = nodes[i].size;
            break;
        }
    }

    if ( i >= g_resmem_nodes_allocated ) {
        LOG( LOG_CRIT, "system_dma_free on unallocated addr %08x", addr );
        mutex_unlock( &ddr_alloc_lock );
        return;
    }

    // Free the node
    dma_free_coherent( &g_pdev->dev, size, nodes[i].vaddr, nodes[i].paddr );

    // Swap last element with the just-freed one, then reduce size by 1.
    nodes[i] = nodes[--g_resmem_nodes_allocated];

    system_cma_update_tracking_info( size, false );
    system_cma_print_tracking_info();

    mutex_unlock( &ddr_alloc_lock );
}
