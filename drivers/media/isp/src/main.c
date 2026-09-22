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

#include <asm/io.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/devfreq.h>
#include "system_control.h" //BSP
#include "system_stdlib.h"
#include "system_timer.h" //system_timer_usleep
#include "acamera.h"      //Isp includes
#include "acamera_configuration.h"
#include "acamera_control_config.h"
#include "acamera_isp_config.h"
#include "acamera_logger.h"

#include <linux/simaai-stu.h>

// The settings for each firmware context were pre-generated and
// saved in this header file. They are given as a reference and should be changed
// according to the customer needs.
#include "runtime_initialization_settings.h"

// Include V4L2 specific headers, define structs and names
#if V4L2_INTERFACE_BUILD
#include "isp-v4l2.h"
#define ISP_V4L2_MODULE_NAME "isp-v4l2"
static struct v4l2_device v4l2_dev;
#else
#include "acamera_frame_consumer.h"
#endif

// Sima.ai STU driver handle
struct simaai_stu *stu;

static struct clk *ispclk;
static struct clk *isppll;

static struct devfreq *isp_df;

extern void system_interrupts_set_irq( void *pdev, int irq_num, int flags );

// system_cma.c
extern struct platform_device *g_pdev;
extern size_t g_resmem_remaining;
extern size_t g_resmem_nodes_allocated;
void system_cma_print_tracking_info( void ); // NOT an API function

static const struct of_device_id isp_dt_match[] = {
    {.compatible = "arm,isp"},
    {}};

MODULE_DEVICE_TABLE( of, isp_dt_match );

static int __maybe_unused isp_runtime_resume(struct device *dev)
{
	pr_debug("%s\n", __func__);
	return clk_prepare_enable(ispclk);
}

static int __maybe_unused isp_runtime_suspend(struct device *dev)
{
	pr_debug("%s\n", __func__);
	clk_disable_unprepare(ispclk);
	return 0;
}

static const struct dev_pm_ops isp_pm_ops = {
	SET_RUNTIME_PM_OPS(isp_runtime_suspend, isp_runtime_resume, NULL)
};

static struct platform_driver isp_platform_driver = {
    .driver = {
        .name = "arm,isp",
        .owner = THIS_MODULE,
        .of_match_table = isp_dt_match,
	.pm = &isp_pm_ops,
    },
};

static semaphore_t isp_sync_semaphore = NULL;
static struct task_struct *isp_fw_process_thread = NULL;
static struct task_struct *isp_fw_init_thread = NULL;

static int isp_fw_process( void *data )
{
    semaphore_t sem = data;

    int32_t ret = 1;
	LOG(LOG_INFO, "ISP : fw process thread %d", current->pid);
    while ( !kthread_should_stop() || ret ) {

        /* no timeout needed as it will be wokenup when packet arrives.
	 * This change necessary to keep the PM runtime in sane state
	 * */
        system_semaphore_wait( sem, 0);

	if (pm_runtime_resume_and_get(&g_pdev->dev) >=0) {
            ret = acamera_process_event();
            pm_runtime_put(&g_pdev->dev);
        } else {
            ret = 0;
        }
    }
    LOG( LOG_INFO, "isp_fw_process stop" );
    return 0;
}

acamera_settings *get_settings_by_id(u8 ctx_id) {

	if (ctx_id >= FIRMWARE_CONTEXT_NUMBER)
		return NULL;

	return &settings[ctx_id];
}

static void isp_cdma_init( void )
{
#if defined( ISP_HAS_MCFE_FSM )
    uint32_t pid_reg, pid_backup , version_reg , api_reg , revision_reg;
    uint64_t ctx_backup_physaddr = PHY_ADDR_CDMA;
    dma_addr_t ctx_backup_busaddr = 0;
	dma_addr_t phys_isp_addr;
    int i, result;

    // Initialize CDMA address.
    LOG( LOG_NOTICE, "Copying default register space to CDMA spaces..." );
    if ( PHY_ADDR_ISP == 0 || PHY_ADDR_CDMA == 0 ) {
        LOG( LOG_CRIT, "- Error, NULL address base. (ISP=%x, CDMA=%x)", PHY_ADDR_ISP, PHY_ADDR_CDMA );
    }

    // Load product ID on back-up space to check if it's loaded.
    pid_reg = acamera_isp_id_product_read( PHY_ADDR_ISP );
	result = simaai_stu_get_bus_address(stu, PHY_ADDR_CDMA, (dma_addr_t *)&ctx_backup_busaddr);
	if (result != 0) {
		LOG (LOG_CRIT, "ERROR : getting bus address for phys address, %#llx", PHY_ADDR_CDMA);
		return;
	}

    pid_backup = acamera_isp_id_product_read( ctx_backup_busaddr );
    version_reg = acamera_isp_id_version_read( PHY_ADDR_ISP );
    api_reg = acamera_isp_id_api_read( PHY_ADDR_ISP );
    revision_reg =  acamera_isp_id_revision_read( PHY_ADDR_ISP );

    LOG( LOG_NOTICE, "- Product ID: REG = %x, BACKUP = %x", pid_reg, pid_backup );

    if ( pid_reg != pid_backup ) {
        // Make back-up of register init to ctx_backup_physaddr.
		result =  simaai_stu_get_dev_address(stu, (dma_addr_t)PHY_ADDR_ISP, &phys_isp_addr);
    	if (result != 0) {
			LOG (LOG_CRIT, "ERROR : getting dev address for bus address, %#lx", PHY_ADDR_ISP);
			return;
		}
        LOG( LOG_NOTICE, "- Preparing CDMA base with default ISP registers on %llx...", ctx_backup_physaddr );
		LOG (LOG_INFO, "phys isp address %#x, dev address %#llx", PHY_ADDR_ISP, phys_isp_addr);
		system64_memcpy_phy2phy( ctx_backup_physaddr, (uint64_t)phys_isp_addr, LEN_ADDR_ISP );
    }

    // Copy from CDMA base address to CDMA slot addresses, and assign isp_base for contexts.
    for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
        uint64_t base_addr = PHY_ADDR_CDMA + LEN_ADDR_ISP + i * ( LEN_ADDR_ISP + LEN_ADDR_META );
        system64_memcpy_phy2phy(base_addr, ctx_backup_physaddr, LEN_ADDR_ISP );
        result = simaai_stu_get_bus_address(stu, base_addr, (dma_addr_t *)&settings[i].isp_base);
    	if (result != 0) {
			LOG (LOG_CRIT, "ERROR : getting bus address for phys address, %#llx", base_addr);
			return;
		}
	    LOG( LOG_NOTICE, "- Base address for CTX#%02d to %#llx - %llx...", i, base_addr, settings[i].isp_base );
    }
#endif
    // Otherwise, do nothing if MFCE fsm is not present
}

/**
 * isp_devfreq_target() - devfreq target callback, scale the ISP PLL
 * @dev:   the devfreq device (ISP platform device)
 * @freq:  in/out, requested frequency in Hz; updated to the chosen OPP rate
 * @flags: devfreq frequency-selection flags
 *
 * Snaps the request to a supported OPP and reprograms the ISP PLL to it.
 *
 * Return: 0 on success or a negative errno from the OPP lookup / clk_set_rate().
 */

static int isp_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
    struct dev_pm_opp *opp;
    int ret;

    opp = devfreq_recommended_opp(dev, freq, flags);
    if (IS_ERR(opp))
        return PTR_ERR(opp);
    dev_pm_opp_put(opp);

    ret = clk_set_rate(isppll, *freq);
    return ret;
}

/**
 * isp_devfreq_get_cur_freq() - devfreq callback, report the current ISP rate
 * @dev:  the devfreq device (ISP platform device)
 * @freq: out, the current frequency in Hz
 *
 * Reads the live PLL rate and rounds it up to the nearest OPP so devfreq's
 * statistics can match it.
 *
 * Return: 0 always.
 */

static int isp_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
    struct dev_pm_opp *opp;
    unsigned long rate;

    rate = clk_get_rate(isppll);

    opp = dev_pm_opp_find_freq_ceil(dev, &rate);
    if (!IS_ERR(opp))
	dev_pm_opp_put(opp);

    *freq = rate;
    return 0;
}

static struct devfreq_dev_profile isp_devfreq_profile = {
    .target        = isp_devfreq_target,
    .get_cur_freq  = isp_devfreq_get_cur_freq,
    .polling_ms    = 0,
};

static int isp_fw_init_kthread_func( void *pdev )
{
    int result = 0;
    LOG( LOG_INFO, "isp_fw_init start" );
    // The firmware supports multicontext.
    // It means that the customer can use the same firmware for controlling
    // several instances of different sensors/ISP. To initialize a context,
    // the structure acamera_settings must be filled properly.
    // The total number of initialized contexts must not exceed FIRMWARE_CONTEXT_NUMBER.
    // All contexts are enumerated from 0 till ctx_number - 1.

    result = pm_runtime_resume_and_get(&g_pdev->dev);
    if(result < 0)
	return -1;

    system_semaphore_init( &isp_sync_semaphore, 1 );

// V4L2 builds do not need internal frame consumer
#if !V4L2_INTERFACE_BUILD
    // Register frame consumer module. Sets up callback to the frame streamer to
    // get notification on a new raw or output frame ready to be fetched
    result = frame_consumer_initialize();
#endif

    if ( result == 0 ) {

        // Needed to be able to access CDMA registers:
        bsp_init();
        isp_cdma_init();

        result = acamera_init( settings, FIRMWARE_CONTEXT_NUMBER, isp_sync_semaphore );
        LOG( LOG_INFO, "isp_fw_init::acamera_init() result %d", result );
    }
    LOG( LOG_INFO, "isp_fw_init result %d", result );

// V4L2 builds do not require automatic context start
#if !V4L2_INTERFACE_BUILD
    // Starting all contexts
    if ( result == 0 ) {
        int i;
        for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
            acamera_config_ctx( i );
            acamera_start_ctx( i );
        }
    }
#endif

    if ( result == 0 ) {
        LOG( LOG_INFO, "Start firmware process thread." );
        isp_fw_process_thread = kthread_run( isp_fw_process, isp_sync_semaphore, "isp_process" );
        if ( isp_fw_process_thread == NULL ) {
            LOG( LOG_CRIT, "Failed to start firmware processing thread." );
        } else {
            LOG( LOG_DEBUG, "isp_fw_process_thread pid: %d", isp_fw_process_thread->pid );
        }
    }

// V4L2 device and instance initialisation
#if V4L2_INTERFACE_BUILD
    if ( pdev == NULL ) {
        LOG( LOG_ERR, "Failed to register V4L2 device. Platform device pointer is NULL." );
        goto release_pm;
    }

    static atomic_t drv_instance = ATOMIC_INIT( 0 );

    v4l2_device_set_name( &v4l2_dev, ISP_V4L2_MODULE_NAME, &drv_instance );
    result = v4l2_device_register( &( ( (struct platform_device *)pdev )->dev ), &v4l2_dev );
    if ( result == 0 ) {
        LOG( LOG_INFO, "V4L2 device successfully registered." );
    } else {
        LOG( LOG_ERR, "Failed to register V4L2 device (%d).", result );
        goto release_pm;
    }

    result = isp_v4l2_create_instance( &v4l2_dev );
    if ( result < 0 ) {
        LOG( LOG_ERR, "Failed to register ISP V4L2 driver (%d).", result );
        goto release_pm;
    }
#endif

release_pm:
    pm_runtime_put(&g_pdev->dev);

    LOG( LOG_INFO, "isp_fw_init_thread exit" );
    isp_fw_init_thread = NULL;
    return PTR_ERR_OR_ZERO( isp_fw_process_thread );
}

static int32_t isp_platform_probe( struct platform_device *pdev )
{
    int32_t rc = 0;
    struct resource *isp_irq;
	int irq;
    struct device_node *resmem_node;
    struct resource resmem;
    struct dev_pm_opp *opp;
    unsigned long init_rate;

    // Init DMA memory
    rc = of_reserved_mem_device_init( &pdev->dev );
    if ( rc ) {
        LOG( LOG_ERR, "Error, Could not get reserved memory" );
        return -1;
    }

    g_pdev = pdev;

    // Get "isp-reserved" memory region info:
    // get device node
    resmem_node = of_parse_phandle( pdev->dev.of_node, "memory-region", 0 );
    if ( resmem_node == NULL ) {
        LOG( LOG_ERR, "No 'memory-region' node, please check your DTB file" );
        return -1;
    }
    // convert to resource
    rc = of_address_to_resource( resmem_node, 0, &resmem );
    of_node_put( resmem_node );
    if ( rc != 0 ) {
        LOG( LOG_ERR, "Failed to convert memory-region node into a resource" );
        return -1;
    }
    g_resmem_remaining = (size_t)resource_size( &resmem );
    LOG( LOG_NOTICE, "CMA region: start = 0x%zx, size = 0x%zx", (size_t)resmem.start, g_resmem_remaining );

  if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64))) {
      LOG(LOG_ERR, "Could not set DMA mask\n");
      return -1;
    }

    // Initialize IRQs
    irq = platform_get_irq_byname( pdev, "ISP" );
    if ( irq > 0 ) {
        LOG( LOG_NOTICE, "ISP irq = %d, flags = 0x%x, %s id: %d!", irq, IRQF_TRIGGER_HIGH, pdev->name, pdev->id );
        system_interrupts_set_irq( (void *)pdev, irq, IRQF_TRIGGER_HIGH);
    } else {
        LOG( LOG_ERR, "Error, no irq found from DT" );
        return -1;
    }

	stu = simaai_stu_get_by_phandle(pdev->dev.of_node, "simaai,stu");
	if (IS_ERR(stu)) {
		if (PTR_ERR(stu) != -EPROBE_DEFER)
			stu = NULL;
		else
			return PTR_ERR(stu);
	} else
		LOG( LOG_NOTICE, "SUCCESS getting STU handle");

    ispclk = devm_clk_get(&pdev->dev, "gate");
    if (IS_ERR(ispclk))
        return dev_err_probe(&pdev->dev, PTR_ERR(ispclk), "failed to get ISP gate clock\n");
    isppll = devm_clk_get(&pdev->dev, "pll");
    if (IS_ERR(isppll))
        return dev_err_probe(&pdev->dev, PTR_ERR(isppll), "no PLL clock\n");

    rc = clk_prepare_enable(ispclk);
    if (rc) return rc;

    pm_runtime_set_active(&pdev->dev);
    pm_runtime_enable(&pdev->dev);

     /* OPP table defines the frequencies the PLL may run at. */
    rc = devm_pm_opp_of_add_table(&pdev->dev);
    if (rc) {
        dev_err_probe(&pdev->dev, rc, "failed to add OPP table\n");
	goto err_probe;
    }

    /* Seed devfreq's initial frequency from the PLL's current rate. */
    init_rate = clk_get_rate(isppll);

    opp = dev_pm_opp_find_freq_ceil(&pdev->dev, &init_rate);
    if (!IS_ERR(opp))
	dev_pm_opp_put(opp);

    isp_devfreq_profile.initial_freq = init_rate;
    isp_df = devm_devfreq_add_device(&pdev->dev, &isp_devfreq_profile,
			 DEVFREQ_GOV_USERSPACE, NULL);
    if (IS_ERR(isp_df)) {
        dev_err_probe(&pdev->dev, PTR_ERR(isp_df),
                         "ISP devfreq registration failed\n");
	goto err_probe;
    }
    if ( rc == 0 ) {
        isp_fw_init_thread = kthread_run( isp_fw_init_kthread_func, pdev, "isp_fw_init" );
        if ( isp_fw_init_thread == NULL ) {
            LOG( LOG_CRIT, "Failed to start firmware init thread." );
        } else {
            LOG( LOG_DEBUG, "isp_fw_init_thread pid: %d", isp_fw_init_thread->pid );
        }
    }

	LOG( LOG_INFO, "ISP driver probe complete !!!");

    return PTR_ERR_OR_ZERO( isp_fw_init_thread );

err_probe:
    clk_disable_unprepare(ispclk);
    pm_runtime_disable(&pdev->dev);

    return -1;

}

/**
 * @brief      Kernel module entry point
 *
 */
static int __init fw_module_init( void )
{
    int32_t rc = 0;

    LOG( LOG_INFO, "ISP %s", __FUNCTION__ );

    rc = platform_driver_probe( &isp_platform_driver,
                                isp_platform_probe );
    return rc;
}

/**
 * @brief      Kernel module exit clean up
 */
static void __exit fw_module_exit( void )
{
    LOG( LOG_INFO, "ISP %s", __FUNCTION__ );

    if ( isp_fw_init_thread ) {
        kthread_stop( isp_fw_init_thread );
    }

    if ( isp_fw_process_thread ) {
        // adding events to transit FSMs to deinit state
        int i;
        for ( i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++ ) {
            acamera_start_ctx( i );
            acamera_stop_ctx( i );
            acamera_deinit_ctx( i );
        }

	system_semaphore_raise( isp_sync_semaphore );
        // blocks until isp_fw_process_thread terminates
        kthread_stop( isp_fw_process_thread );
    }

    // This api function will free all resources allocated by the firmware.
    acamera_deinit();

// V4L2 device and instance deinitialization
#if V4L2_INTERFACE_BUILD
    // Destroy V4L2 instances and unregister device
    isp_v4l2_destroy_instance();
#endif

    system_cma_print_tracking_info();
    if ( g_resmem_nodes_allocated > 0 ) {
        LOG( LOG_WARNING, "system_cma: memory leak detected (%zu nodes)!", g_resmem_nodes_allocated );
    }

    bsp_destroy();

    system_semaphore_destroy( isp_sync_semaphore );

    of_reserved_mem_device_release( &g_pdev->dev );
    platform_driver_unregister( &isp_platform_driver );

    if (g_pdev)
        pm_runtime_disable(&g_pdev->dev);

}

module_init( fw_module_init );
module_exit( fw_module_exit );
MODULE_LICENSE( "GPL v2" );
MODULE_AUTHOR( "Chris K" );
