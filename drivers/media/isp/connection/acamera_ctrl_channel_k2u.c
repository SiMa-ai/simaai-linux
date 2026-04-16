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

#include <linux/fs.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <uapi/linux/isp/isp_ioctl.h>
#include "system_stdlib.h"
#include "acamera_command_api.h"
#include "acamera_ctrl_channel.h"
#include "acamera_logger.h"
#include "acamera_calib_mgr.h"

#define CTRL_CHANNEL_FIFO_INPUT_SIZE ( 4 * 1024 )
#define CTRL_CHANNEL_FIFO_OUTPUT_SIZE ( 8 * 1024 )

struct user_calibration_data {
    ACameraCalibrations *calibrations;
    int32_t ( *new_cb )( uint32_t wdr_mode, void *param );
    int32_t ( *old_cb )( uint32_t wdr_mode, void *param );
};

struct ctrl_channel_dev_context {
    uint8_t dev_inited;
    int dev_minor_id;
    char *dev_name;
    int dev_opened;

    struct miscdevice ctrl_dev;
    struct mutex fops_lock;

    struct kfifo ctrl_kfifo_in;
    struct kfifo ctrl_kfifo_out;

    struct ctrl_cmd_item cmd_item;

    struct user_calibration_data user_calibrations[FIRMWARE_CONTEXT_NUMBER];
    uint8_t num_of_contexts;
};


static struct ctrl_channel_dev_context ctrl_channel_ctx;

#define REPEAT_16(x) x(0) x(1) x(2) x(3) x(4) x(5) x(6) x(7) x(8) x(9) x(10) x(11) x(12) x(13) x(14) x(15)
#define ASSIGN_CALIB_CALLBACK(CTX) custom_get_calib_ctx##CTX,
#define DEFINE_CALIB_CALLBACK(CTX) \
static int32_t custom_get_calib_ctx##CTX(uint32_t wdr_mode, void *param) { \
	if (param != NULL) { \
		if (ctrl_channel_ctx.user_calibrations[CTX].calibrations != NULL) { \
			ACameraCalibrations *c = (ACameraCalibrations *)param; \
			ACameraCalibrations *user_c = ctrl_channel_ctx.user_calibrations[CTX].calibrations; \
			int i; \
			for (i = 0; i < CALIBRATION_TOTAL_SIZE; i++) { \
				if (user_c->calibrations[i] != NULL && user_c->calibrations[i]->ptr != NULL) { \
					c->calibrations[i] = user_c->calibrations[i]; \
				} \
			} \
			LOG(LOG_INFO, "Wrote user calibrations for CTX %d (WDR Mode: %u)\n", CTX, wdr_mode); \
			return 0; \
		} else if (ctrl_channel_ctx.user_calibrations[CTX].old_cb) { \
		return ctrl_channel_ctx.user_calibrations[CTX].old_cb(wdr_mode, param); \
		} \
	} \
	return -1; \
}

REPEAT_16(DEFINE_CALIB_CALLBACK)

int32_t (*cbs[])( uint32_t wdr_mode, void *param ) = {
    REPEAT_16(ASSIGN_CALIB_CALLBACK)
};

static int ctrl_channel_fops_open( struct inode *inode, struct file *f )
{
    int rc;
    struct ctrl_channel_dev_context *p_ctx = &ctrl_channel_ctx;
    int minor = iminor( inode );

    LOG( LOG_INFO, "client is opening..., minor: %d.", minor );

    if ( minor != p_ctx->dev_minor_id ) {
        LOG( LOG_CRIT, "Not matched ID, minor_id: %d, expected: %d(dev name: %s)", minor, p_ctx->dev_minor_id, p_ctx->dev_name );
        return -ERESTARTSYS;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        goto lock_failure;
    }

    if ( p_ctx->dev_opened ) {
        LOG( LOG_ERR, "open(%s) failed, already opened.", p_ctx->dev_name );
        rc = -EBUSY;
    } else {
        p_ctx->dev_opened = 1;
        rc = 0;
        LOG( LOG_INFO, "open(%s) succeed.", p_ctx->dev_name );

        LOG( LOG_INFO, "Bf set, private_data: %p.", f->private_data );
        f->private_data = p_ctx;
        LOG( LOG_INFO, "Af set, private_data: %p.", f->private_data );
    }

    mutex_unlock( &p_ctx->fops_lock );

lock_failure:
    return rc;
}

static int ctrl_channel_fops_release( struct inode *inode, struct file *f )
{
    int rc;
    struct ctrl_channel_dev_context *p_ctx = (struct ctrl_channel_dev_context *)f->private_data;

    if ( p_ctx != &ctrl_channel_ctx ) {
        LOG( LOG_ERR, "Invalid parameter: %p, expected: %p.", p_ctx, &ctrl_channel_ctx );
        return -EINVAL;
    }

    rc = mutex_lock_interruptible( &p_ctx->fops_lock );
    if ( rc ) {
        LOG( LOG_ERR, "Error: lock failed of dev: %s.", p_ctx->dev_name );
        return rc;
    }

    if ( p_ctx->dev_opened ) {
        p_ctx->dev_opened = 0;
        f->private_data = NULL;
        kfifo_reset( &p_ctx->ctrl_kfifo_in );
        kfifo_reset( &p_ctx->ctrl_kfifo_out );
        LOG( LOG_INFO, "close(%s) succeed, private_data: %p.", p_ctx->dev_name, p_ctx );
    } else {
        LOG( LOG_CRIT, "Fatal error: wrong state of dev: %s, dev_opened: %d.", p_ctx->dev_name, p_ctx->dev_opened );
        rc = -EINVAL;
    }

    mutex_unlock( &p_ctx->fops_lock );

    return 0;
}

static ssize_t ctrl_channel_fops_write( struct file *file, const char __user *buf, size_t count, loff_t *ppos )
{
    int rc;
    unsigned int copied;
    struct ctrl_channel_dev_context *p_ctx = (struct ctrl_channel_dev_context *)file->private_data;

    if ( p_ctx != &ctrl_channel_ctx ) {
        LOG( LOG_ERR, "Invalid parameter: %p, expected: %p.", p_ctx, &ctrl_channel_ctx );
        return -EINVAL;
    }

    if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_CRIT, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    rc = kfifo_from_user( &p_ctx->ctrl_kfifo_in, buf, count, &copied );

    mutex_unlock( &p_ctx->fops_lock );

    LOG( LOG_DEBUG, "wake up reader." );

    return rc ? rc : copied;
}

static ssize_t ctrl_channel_fops_read( struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
    int rc = 0;
    unsigned int copied = 0;
    struct ctrl_channel_dev_context *p_ctx = (struct ctrl_channel_dev_context *)file->private_data;

    if ( p_ctx != &ctrl_channel_ctx ) {
        LOG( LOG_ERR, "Invalid parameter: %p, expected: %p.", p_ctx, &ctrl_channel_ctx );
        return -EINVAL;
    }

    if ( mutex_lock_interruptible( &p_ctx->fops_lock ) ) {
        LOG( LOG_CRIT, "Fatal error: access lock failed." );
        return -ERESTARTSYS;
    }

    if ( !kfifo_is_empty( &p_ctx->ctrl_kfifo_out ) ) {
        rc = kfifo_to_user( &p_ctx->ctrl_kfifo_out, buf, count, &copied );
    }

    mutex_unlock( &p_ctx->fops_lock );

    return rc ? rc : copied;
}

static long ctrl_channel_fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ctrl_channel_dev_context *p_ctx = (struct ctrl_channel_dev_context *)file->private_data;
	struct isp_calibration_data calib;
	ACameraCalibrations *kcalibs;
	int i;

	LOG(LOG_ERR, "Entered IOCTL");
	if (p_ctx != &ctrl_channel_ctx)
		return -EINVAL;

	if (cmd == ISP_IOC_SET_CALIBRATION) {
		if (copy_from_user(&calib, (void __user *)arg,
			sizeof(calib))) return -EFAULT;

		if (calib.ctx_id >= p_ctx->num_of_contexts || calib.size == 0)
			return -EINVAL;

		mutex_lock(&p_ctx->fops_lock);

		if (p_ctx->user_calibrations[calib.ctx_id].calibrations != NULL)
			kfree(p_ctx->user_calibrations[calib.ctx_id].calibrations);

		kcalibs = kmalloc(calib.size, GFP_KERNEL);
		if (!kcalibs) {
			mutex_unlock(&p_ctx->fops_lock);
			return -ENOMEM;
		}

		if (copy_from_user(kcalibs, calib.data, calib.size)) {
			kfree(kcalibs);
			p_ctx->user_calibrations[calib.ctx_id].calibrations = NULL;
			mutex_unlock(&p_ctx->fops_lock);
			return -EFAULT;
		}

		for (i = 0; i < CALIBRATION_TOTAL_SIZE; i++) {
			if (kcalibs->calibrations[i]) {
				uintptr_t table_offset = (uintptr_t)kcalibs->calibrations[i];
				LookupTable *table = (LookupTable *)((uint8_t *)kcalibs + table_offset);
				kcalibs->calibrations[i] = table;

				if (table->ptr) {
					uintptr_t data_offset = (uintptr_t)table->ptr;
					table->ptr = (const void *)((uint8_t *)kcalibs + data_offset);
				}
			}
		}

		p_ctx->user_calibrations[calib.ctx_id].calibrations = kcalibs;
		LOG(LOG_ERR, "Patched calibration data for CTX %u", calib.ctx_id);

		mutex_unlock(&p_ctx->fops_lock);
		return 0;
	}

	return -ENOTTY;
}

static struct file_operations isp_fops = {
    .owner = THIS_MODULE,
    .open = ctrl_channel_fops_open,
    .release = ctrl_channel_fops_release,
    .read = ctrl_channel_fops_read,
    .write = ctrl_channel_fops_write,
    .llseek = noop_llseek,
    .unlocked_ioctl = ctrl_channel_fops_ioctl,
};


static int32_t ctrl_channel_write( const struct ctrl_cmd_item *p_cmd, const void *data, uint32_t data_size )
{
    if ( !ctrl_channel_ctx.dev_inited ) {
        LOG( LOG_ERR, "dev is not initialized, failed to write." );
        return -1;
    }

    mutex_lock( &ctrl_channel_ctx.fops_lock );

    int32_t rc = kfifo_in( &ctrl_channel_ctx.ctrl_kfifo_out, p_cmd, sizeof( struct ctrl_cmd_item ) );
    if ( data )
        kfifo_in( &ctrl_channel_ctx.ctrl_kfifo_out, data, data_size );

    mutex_unlock( &ctrl_channel_ctx.fops_lock );

    return rc;
}


int32_t ctrl_channel_init( acamera_settings *settings, uint8_t num_of_contexts )
{
    int32_t rc;
    int i;

    system_memset( &ctrl_channel_ctx, 0, sizeof( ctrl_channel_ctx ) );
    ctrl_channel_ctx.ctrl_dev.name = CTRL_CHANNEL_DEV_NAME;
    ctrl_channel_ctx.dev_name = CTRL_CHANNEL_DEV_NAME;
    ctrl_channel_ctx.ctrl_dev.minor = MISC_DYNAMIC_MINOR;
    ctrl_channel_ctx.ctrl_dev.fops = &isp_fops;
    ctrl_channel_ctx.num_of_contexts = num_of_contexts;

    for(i = 0; i < num_of_contexts; i++) {
        ctrl_channel_ctx.user_calibrations[i].calibrations = NULL;
	ctrl_channel_ctx.user_calibrations[i].old_cb = settings[i].get_calibrations;
	settings[i].get_calibrations = cbs[i];
    }

    rc = misc_register( &ctrl_channel_ctx.ctrl_dev );
    if ( rc ) {
        LOG( LOG_ERR, "Error: register ISP ctrl channel device failed, ret: %d.", rc );
        return rc;
    }

    ctrl_channel_ctx.dev_minor_id = ctrl_channel_ctx.ctrl_dev.minor;
    mutex_init( &ctrl_channel_ctx.fops_lock );

    rc = kfifo_alloc( &ctrl_channel_ctx.ctrl_kfifo_in, CTRL_CHANNEL_FIFO_INPUT_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_in alloc failed, ret: %d.", rc );
        goto failed_kfifo_in_alloc;
    }

    rc = kfifo_alloc( &ctrl_channel_ctx.ctrl_kfifo_out, CTRL_CHANNEL_FIFO_OUTPUT_SIZE, GFP_KERNEL );
    if ( rc ) {
        LOG( LOG_ERR, "Error: kfifo_out alloc failed, ret: %d.", rc );
        goto failed_kfifo_out_alloc;
    }

    ctrl_channel_ctx.dev_inited = 1;

    LOG( LOG_INFO, "ctrl_channel_dev_context(%s) init OK.", ctrl_channel_ctx.dev_name );

    return 0;

failed_kfifo_out_alloc:
    kfifo_free( &ctrl_channel_ctx.ctrl_kfifo_in );
failed_kfifo_in_alloc:
    misc_deregister( &ctrl_channel_ctx.ctrl_dev );

    LOG( LOG_ERR, "Error: init failed for dev: %s.", ctrl_channel_ctx.ctrl_dev.name );
    return rc;
}

void ctrl_channel_process( void )
{
}

void ctrl_channel_deinit( void )
{
	int i;
    if ( ctrl_channel_ctx.dev_inited ) {
        kfifo_free( &ctrl_channel_ctx.ctrl_kfifo_in );

        kfifo_free( &ctrl_channel_ctx.ctrl_kfifo_out );

        misc_deregister( &ctrl_channel_ctx.ctrl_dev );

	for(i = 0; i < ctrl_channel_ctx.num_of_contexts; i++) {
		if (ctrl_channel_ctx.user_calibrations[i].calibrations) {
			kfree(ctrl_channel_ctx.user_calibrations[i].calibrations);
			ctrl_channel_ctx.user_calibrations[i].calibrations = NULL;
		}
	}

        LOG( LOG_INFO, "misc_deregister dev: %s.", ctrl_channel_ctx.ctrl_dev.name );
    } else {
        LOG( LOG_INFO, "dev not initialized, do nothing." );
    }
}

static uint8_t is_uf_needed_command( uint8_t command_type, uint8_t command, uint8_t direction )
{
    uint8_t rc = 0;

    // We only support SET in user-FW.
    if ( COMMAND_GET == direction )
        return rc;

    switch ( command_type ) {

        // If we need to blacklist some commands add corresponding case statement

    default:
        rc = 1;
        break;
    }

    return rc;
}

void ctrl_channel_handle_api_command( uint32_t ctx_id, uint8_t cmd_if_mode, uint8_t type, uint8_t command, uint32_t value, uint8_t direction )
{
    if ( !ctrl_channel_ctx.dev_inited ) {
        LOG( LOG_ERR, "FW ctrl channel is not initialized." );
        return;
    }

    if ( !ctrl_channel_ctx.dev_opened ) {
        LOG( LOG_DEBUG, "FW ctrl channel is not opened, skip." );
        return;
    }

    /* If user-FW is not needed this command, we do nothing */
    if ( !is_uf_needed_command( type, command, direction ) ) {
        return;
    }

    struct ctrl_cmd_item *p_cmd = &ctrl_channel_ctx.cmd_item;
    p_cmd->cmd_category = CTRL_CMD_CATEGORY_API_COMMAND;
    p_cmd->cmd_len = sizeof( struct ctrl_cmd_item );
    p_cmd->cmd_ctx_id = ctx_id;
    p_cmd->cmd_if_mode = cmd_if_mode;
    p_cmd->cmd_type = type;
    p_cmd->cmd_id = command;
    p_cmd->cmd_direction = direction;
    p_cmd->cmd_value = value;

    LOG( LOG_INFO, "api_command: cmd_ctx_id: %u, cmd_if_mode: %u, cmd_type: %u, cmd_id: %u, cmd_direction: %u, cmd_value: %u.",
         p_cmd->cmd_ctx_id, p_cmd->cmd_if_mode, p_cmd->cmd_type, p_cmd->cmd_id, p_cmd->cmd_direction, p_cmd->cmd_value );

    ctrl_channel_write( p_cmd, NULL, 0 );
}

void ctrl_channel_handle_api_calibration( uint32_t ctx_id, uint8_t type, uint8_t id, uint8_t direction, void *data, uint32_t data_size )
{
    if ( !ctrl_channel_ctx.dev_inited ) {
        LOG( LOG_ERR, "FW ctrl channel is not initialized." );
        return;
    }

    if ( !ctrl_channel_ctx.dev_opened ) {
        LOG( LOG_INFO, "FW ctrl channel is not opened, skip." );
        return;
    }

    struct ctrl_cmd_item *p_cmd = &ctrl_channel_ctx.cmd_item;
    p_cmd->cmd_category = CTRL_CMD_CATEGORY_API_CALIBRATION;
    p_cmd->cmd_len = sizeof( struct ctrl_cmd_item ) + data_size;
    p_cmd->cmd_ctx_id = ctx_id;
    p_cmd->cmd_type = type;
    p_cmd->cmd_id = id;
    p_cmd->cmd_direction = direction;
    p_cmd->cmd_value = data_size;

    LOG( LOG_INFO, "api_calibration: cmd_ctx_id: %u, cmd_type: %u, cmd_id: %u, cmd_direction: %u, cmd_value: %u.",
         p_cmd->cmd_ctx_id, p_cmd->cmd_type, p_cmd->cmd_id, p_cmd->cmd_direction, p_cmd->cmd_value );

    ctrl_channel_write( p_cmd, data, data_size );
}

void ctrl_channel_handle_api_event( uint32_t ctx_id, uint8_t cmd_if_mode, uint32_t event_id )
{
    if ( !ctrl_channel_ctx.dev_inited ) {
        LOG( LOG_ERR, "FW ctrl channel is not initialized." );
        return;
    }

    if ( !ctrl_channel_ctx.dev_opened ) {
        LOG( LOG_DEBUG, "FW ctrl channel is not opened, skip." );
        return;
    }

    // If Command interface is not in passive mode - skip
    if ( cmd_if_mode != CMD_IF_MODE_PASSIVE ) {
        return;
    }

    struct ctrl_cmd_item *p_cmd = &ctrl_channel_ctx.cmd_item;
    p_cmd->cmd_category = CTRL_CMD_CATEGORY_API_EVENT;
    p_cmd->cmd_len = sizeof( struct ctrl_cmd_item );
    p_cmd->cmd_ctx_id = ctx_id;
    p_cmd->cmd_if_mode = cmd_if_mode;
    p_cmd->cmd_value = event_id;

    LOG( LOG_INFO, "api_event: cmd_ctx_id: %u, cmd_if_mode: %u, cmd_value: %u.",
         p_cmd->cmd_ctx_id, p_cmd->cmd_if_mode, p_cmd->cmd_value );

    ctrl_channel_write( p_cmd, NULL, 0 );
}