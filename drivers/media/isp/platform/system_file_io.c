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

#include "system_file_io.h"
#include <linux/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/fs.h>
#include <linux/version.h>


void *system_file_open( const char *path, uint32_t flags )
{
    struct file *filp = NULL;
    int32_t rights = 0;
    //mm_segment_t oldfs;
    int err = 0;
#if 0
    oldfs = get_fs();
#if ( LINUX_VERSION_CODE > KERNEL_VERSION( 5, 0, 21 ) )
    set_fs( KERNEL_DS );
#else
    set_fs( get_ds() );
#endif
#endif    
    filp = filp_open( path, O_RDWR | O_SYNC | O_CREAT, rights );
#if 0
    set_fs( oldfs );
#endif    
    if ( IS_ERR( filp ) ) {
        err = PTR_ERR( filp );
        return NULL;
    }
    return (void *)filp;
}


int32_t system_file_write( void *file, uint64_t offset, uint8_t *data, uint32_t size )
{
    //mm_segment_t oldfs;
    loff_t loff = offset;
    int ret;
#if 0
    oldfs = get_fs();
#if ( LINUX_VERSION_CODE > KERNEL_VERSION( 5, 0, 21 ) )
    set_fs( KERNEL_DS );
#else
    set_fs( get_ds() );
#endif
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION( 4, 14, 13 )
    ret = vfs_write( file, data, size, &loff );
#else
    ret = kernel_write( file, data, size, &loff );
#endif
#if 0
    set_fs( oldfs );
#endif    
    return ret;
}


int32_t system_file_read( void *file, uint32_t offset, uint8_t *data, uint32_t size )
{
    //mm_segment_t oldfs;
    loff_t loff = offset;
    int ret;
#if 0
    oldfs = get_fs();
#if ( LINUX_VERSION_CODE > KERNEL_VERSION( 5, 0, 21 ) )
    set_fs( KERNEL_DS );
#else
    set_fs( get_ds() );
#endif
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION( 4, 14, 13 )
    ret = vfs_read( file, data, size, &loff );
#else
    ret = kernel_read( file, data, size, &loff );
#endif
#if 0
    set_fs( oldfs );
#endif    
    return ret;
}


int32_t system_file_close( void *file )
{
    return filp_close( file, NULL );
}
