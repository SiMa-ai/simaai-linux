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

#include "acamera_logger.h"
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>

/* If the kernel is not used then simply define these strings to empty string.
 * This way we can have a single LUT definition */

#ifndef KERN_DEBUG
#define KERN_DEBUG ""
#endif

#ifndef KERN_INFO
#define KERN_INFO ""
#endif

#ifndef KERN_NOTICE
#define KERN_NOTICE ""
#endif

#ifndef KERN_WARNING
#define KERN_WARNING ""
#endif

#ifndef KERN_ERR
#define KERN_ERR ""
#endif

#ifndef KERN_CRIT
#define KERN_CRIT ""
#endif

#ifndef KERN_ALERT
#define KERN_ALERT ""
#endif

#ifndef KERN_EMERG
#define KERN_EMERG ""
#endif

/*Table of const pointers to const strings. */
static const char *const m_level_names[LOG_LEVEL_MAX] = {
    [LOG_LEVEL_DEBUG] = KERN_DEBUG LOG_COLOR_DEBUG "[DEBUG]" LOG_COLOR_DEFAULT "  ",
    [LOG_LEVEL_INFO] = KERN_INFO LOG_COLOR_INFO "[INFO]" LOG_COLOR_DEFAULT "   ",
    [LOG_LEVEL_NOTICE] = KERN_NOTICE LOG_COLOR_NOTICE "[NOTICE]" LOG_COLOR_DEFAULT " ",
    [LOG_LEVEL_WARNING] = KERN_WARNING LOG_COLOR_WARN "[WARN]" LOG_COLOR_DEFAULT "   ",
    [LOG_LEVEL_ERROR] = KERN_ERR LOG_COLOR_ERROR "[ERROR]" LOG_COLOR_DEFAULT "  ",
    [LOG_LEVEL_CRIT] = KERN_CRIT LOG_COLOR_CRIT "[!CRIT!]" LOG_COLOR_DEFAULT " ",
    [LOG_LEVEL_ALERT] = KERN_ALERT LOG_COLOR_CRIT "[!ALERT!]" LOG_COLOR_DEFAULT " ",
    [LOG_LEVEL_EMERG] = KERN_EMERG LOG_COLOR_CRIT "[!EMERG!]" LOG_COLOR_DEFAULT " ",
};


/** logger internal data structure*/
static struct {
    void *lock;               /**< Pointer to a lock object (optional). */
    alog_lock_fn log_lock_fn; /**< User provided locking function, NULL by default. */
    alog_level_t level;       /**< Current log level. */
    alog_mask_t mask;         /**< current log mask*/
    uint32_t categories;      /**< runtime category bitmask (alog_category_t) */
    uint32_t throttle;        /**< sampled-log interval; 1 = every frame, 100 default */

} g_alog = {.lock = NULL,
            .log_lock_fn = NULL,
            .level = LOG_DEFAULT_VALUE,
            .mask = ( alog_mask_t )( ( 1u << LOG_LEVEL_MAX ) - 1u ),
            .categories = ALOG_CAT_ALL,
            .throttle = 100};


void alog_lock( void )
{
    if ( g_alog.log_lock_fn )
        g_alog.log_lock_fn( true, g_alog.lock );
}

void alog_unlock( void )
{
    if ( g_alog.log_lock_fn )
        g_alog.log_lock_fn( false, g_alog.lock );
}


void alog_set_locking( alog_lock_fn fun, void *lock )
{
    g_alog.log_lock_fn = fun;
    g_alog.lock = lock;
}

void alog_set_level( const alog_level_t level )
{
    if ( level < LOG_LEVEL_MAX ) {
        alog_lock();
        g_alog.level = level;
        alog_unlock();
    }
}


alog_level_t alog_get_level( void ) { return g_alog.level; }


void alog_set_mask( const alog_mask_t mask )
{
    alog_lock();
    g_alog.mask = mask;
    alog_unlock();
}

alog_mask_t alog_get_mask( void ) { return g_alog.mask; }


bool alog_check_level( const alog_level_t level )
{
    if ( g_alog.level == LOG_LEVEL_NOTHING ) return false;
    if ( level < g_alog.level ) return false;
    if ( ( ( 1 << level ) & g_alog.mask ) == 0 ) return false;
    return true;
}

const char *alog_level_string( const alog_level_t level )
{
    return m_level_names[level % LOG_LEVEL_MAX];
}


void alog_set_categories( const uint32_t categories )
{
    alog_lock();
    g_alog.categories = categories;
    alog_unlock();
}

uint32_t alog_get_categories( void ) { return g_alog.categories; }

bool alog_check_cat( const uint32_t cat ) { return ( g_alog.categories & cat ) != 0; }

void alog_set_throttle( const uint32_t n ) { g_alog.throttle = n ? n : 1; }

uint32_t alog_throttle( void ) { return g_alog.throttle ? g_alog.throttle : 1; }


/* Parse "key = value" for level/categories out of the config text buffer.
 * Unknown keys are ignored; absent keys keep their compiled defaults. */
static void alog_parse_config( const char *buf )
{
    const char *p = buf;

    while ( p && *p ) {
        unsigned long v;

        while ( *p == ' ' || *p == '\t' ) p++;
        if ( *p == '#' ) {
            /* comment: skip to end of line */
        } else if ( !strncmp( p, "level", 5 ) ) {
            const char *e = strchr( p, '=' );
            if ( e && !kstrtoul( skip_spaces( e + 1 ), 0, &v ) )
                alog_set_level( ( alog_level_t )v );
        } else if ( !strncmp( p, "categories", 10 ) ) {
            const char *e = strchr( p, '=' );
            if ( e && !kstrtoul( skip_spaces( e + 1 ), 0, &v ) )
                alog_set_categories( ( uint32_t )v );
        } else if ( !strncmp( p, "throttle", 8 ) ) {
            const char *e = strchr( p, '=' );
            if ( e && !kstrtoul( skip_spaces( e + 1 ), 0, &v ) )
                alog_set_throttle( ( uint32_t )v );
        }
        p = strchr( p, '\n' );
        if ( p ) p++;
    }
}

void alog_load_config( void )
{
    struct file *f;
    char *buf;
    loff_t pos = 0;
    ssize_t n;

    f = filp_open( ISP3A_LOG_CONF_PATH, O_RDONLY, 0 );
    if ( IS_ERR( f ) )
        return; /* absent -> keep compiled defaults */

    buf = kzalloc( 1024, GFP_KERNEL );
    if ( buf ) {
        n = kernel_read( f, buf, 1023, &pos );
        if ( n > 0 ) {
            buf[n] = '\0';
            alog_parse_config( buf );
            LOG( LOG_NOTICE, "isp3a log config loaded: level=%d categories=0x%x",
                 alog_get_level(), alog_get_categories() );
        }
        kfree( buf );
    }
    filp_close( f, NULL );
}
