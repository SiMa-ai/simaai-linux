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

#ifndef ACAMERA_LOGGER_H
#define ACAMERA_LOGGER_H

#include "system_log.h"            // Necessary for printf implementation.
#include "acamera_configuration.h" // Set file can define LOG setup.
#include <linux/sched.h>
/*============================================
=            Logger Configuration            =
============================================*/

#ifdef SYSTEM_LOG_USE_COLOR
#define LOG_USE_COLOR SYSTEM_LOG_USE_COLOR
#else
#define LOG_USE_COLOR 0
#endif

#ifndef SYSTEM_PRINTF
#warning SYSTEM_PRINTF undefined, using printf
#define SYSTEM_PRINTF printf
#endif

#ifdef SYSTEM_LOG_POST_FIX
#define LOG_POST_FIX SYSTEM_LOG_POST_FIX
#else
#define LOG_POST_FIX ""
#endif

#ifdef SYSTEM_LOG_DEFAULT_VALUE
#define LOG_DEFAULT_VALUE SYSTEM_LOG_DEFAULT_VALUE
#else
#define LOG_DEFAULT_VALUE LOG_LEVEL_NOTICE
#endif

/*=====  End of Logger Configuration  ======*/


#if LOG_USE_COLOR
#define LOG_COLOR_DEBUG "\e[0;37m"
#define LOG_COLOR_INFO "\e[32m"
#define LOG_COLOR_NOTICE "\e[36m"
#define LOG_COLOR_WARN "\e[33m"
#define LOG_COLOR_ERROR "\e[0;31m"
#define LOG_COLOR_CRIT "\e[1;31m"
#define LOG_COLOR_FILE "\e[1;30m"
#define LOG_COLOR_DEFAULT "\033[0m"
#else
#define LOG_COLOR_DEBUG ""
#define LOG_COLOR_INFO ""
#define LOG_COLOR_NOTICE ""
#define LOG_COLOR_WARN ""
#define LOG_COLOR_ERROR ""
#define LOG_COLOR_CRIT ""
#define LOG_COLOR_FILE ""
#define LOG_COLOR_DEFAULT ""
#endif

#ifndef __FILENAME__
#define __FILENAME__ ( __builtin_strrchr( __FILE__, '/' ) ? __builtin_strrchr( __FILE__, '/' ) + 1 : __FILE__ )
#endif

/** Locking function prototype.*/
typedef void ( *alog_lock_fn )( const bool lock, void *lock_ptr );

/** Type of the mask, currently 8 bits. */
typedef uint8_t alog_mask_t;


/** Log levels, ideally these should be in order of verbosity (nothing at top,
 * crit at bottom) but I kept the previous levels to maintain API consistency */
typedef enum {
    LOG_LEVEL_DEBUG,   /**< Debug statements. */
    LOG_LEVEL_INFO,    /**< Informational message e.g. startup information at driver initialization. */
    LOG_LEVEL_NOTICE,  /**< Nothing serious, but notably nevertheless. Often used to report security events. */
    LOG_LEVEL_WARNING, /**< A warning, meaning nothing serious by itself but might indicate problems. */
    LOG_LEVEL_ERROR,   /**< An error condition, often used by drivers to indicate difficulties with the hardware. */
    LOG_LEVEL_CRIT,    /**< A critical condition occurred like a serious hardware/software failure. */
    LOG_LEVEL_ALERT,   /**< Something bad happened and action must be taken immediately. */
    LOG_LEVEL_EMERG,   /**< Emergency message, the system is about to or may crash. */
    LOG_LEVEL_NOTHING, /**< Do not log anything to the ouput. Quiet mode. */
    /* Leave this at the bottom */
    LOG_LEVEL_MAX,
} alog_level_t;


/* ============================================================================
 * Runtime 3A log categories (component-based, shared contract).
 *
 * A single category bit gates every LOG_CAT() site for that component across
 * the WHOLE pipeline -- so enabling e.g. ALOG_CAT_AWB traces the end-to-end
 * path: IPA param-load -> IPA 3A stats/compute -> control-plane transfer ->
 * this kernel driver's HW apply. The bit assignment below is a fixed contract
 * shared by three places that MUST stay in sync:
 *   - this header (kernel ISP driver)
 *   - src/ipa/modalix/isp_3a/.../util/logger/acamera_logger.h (libcamera IPA)
 *   - the on-board config file ISP3A_LOG_CONF_PATH
 *
 * At init the driver reads ISP3A_LOG_CONF_PATH; if present its `level` and
 * `categories` bitmask override the compiled defaults, if absent the compiled
 * defaults stand (zero overhead, normal behaviour). This exists so QA can turn
 * on targeted 3A tracing to debug a customer-reported issue without a rebuild.
 * ============================================================================*/
#define ISP3A_LOG_CONF_PATH "/usr/share/libcamera/ipa/modalix/isp3a_log.conf"

typedef enum {
    ALOG_CAT_AE         = 1u << 0,  /* auto-exposure */
    ALOG_CAT_AWB        = 1u << 1,  /* auto-white-balance */
    ALOG_CAT_AF         = 1u << 2,  /* auto-focus */
    ALOG_CAT_CCM        = 1u << 3,  /* colour correction matrix */
    ALOG_CAT_GAMMA      = 1u << 4,  /* gamma / tone */
    ALOG_CAT_IRIDIX     = 1u << 5,  /* iridix (local tone / DRC) */
    ALOG_CAT_SINTER     = 1u << 6,  /* spatial noise reduction */
    ALOG_CAT_SHADING    = 1u << 7,  /* lens shading (radial + mesh LSC) */
    ALOG_CAT_BLACKLEVEL = 1u << 8,  /* black level / sensor offset */
    ALOG_CAT_DEMOSAIC   = 1u << 9,  /* demosaic */
    ALOG_CAT_SHARPEN    = 1u << 10, /* sharpening */
    ALOG_CAT_CNR        = 1u << 11, /* chroma noise reduction */
    ALOG_CAT_DPC        = 1u << 12, /* defect pixel correction */
    ALOG_CAT_SENSOR     = 1u << 13, /* sensor exposure/gain program + mode */
    ALOG_CAT_CONTROL    = 1u << 14, /* command/control plane (SYSTEM_* + param xfer) */
    ALOG_CAT_SBUF       = 1u << 15, /* stats-buffer ring (IPA<->kernel) */
    ALOG_CAT_V4L2       = 1u << 16, /* V4L2 stream / buffer path */
    ALOG_CAT_GENERAL    = 1u << 17, /* uncategorised / framework */
    ALOG_CAT_FLICKER    = 1u << 18, /* AE anti-flicker / flicker detection */
    ALOG_CAT_ALL        = 0xffffffffu,
} alog_category_t;


/**
 * @brief      Provides a function callback for locking and a mutex which is
 *             used by it.
 *
 * @param      fun   The function callback
 * @param      lock  The lock dataq
 *
 * @details    By default the logger is not safe for multi thread, it requires a
 *             user lock implemenation to be provided.
 * @warning    If you plan on calling LOG from ISR then the lock needs to be
 *             adequate (spinlock). If you only use LOG from thread context you
 *             can use a sleeping lock (mutex).
 */
void alog_set_locking( alog_lock_fn fun, void *lock );


/**
 * @brief      Sets log level.
 *
 * @param[in]  level  The level
 *
 * @details    Only messages of the provided level or above will be displayed.
 */
void alog_set_level( const alog_level_t level );


/**
 * @brief      Returns debug level.
 *
 * @return     Debug level
 *
 * @details    Used by the system API.
 */
alog_level_t alog_get_level( void );


/**
 * @brief      Sets the log filter mask.
 *
 * @param[in]  mask  The mask
 *
 * @details    The user can specify a mask for which messages to display. For
 *             example, if log level is LOG_LEVEL_DEBUG (all messages) we can set
 *             a mask to only display notice and critical level:
 * @code
 *             alog_set_mask( ( 1 << LOG_LEVEL_NOTICE) | ( 1 << LOG_LEVEL_CRIT ) );
 * @endcode
 */
void alog_set_mask( const alog_mask_t mask );


/**
 * @brief      Returns the log mask.
 *
 * @return     Bitmask of allowed logs
 *
 */
alog_mask_t alog_get_mask( void );


/**
 * @brief      Locks the logger using locking set by #alog_set_locking.
 *
 * @details    This is a private function, but needs to be exposed here since the
 *             log statement is a macro implementation.
 */
void alog_lock( void );

/**
 * @brief      Unlocks the logger using locking set by #alog_set_locking.
 *
 * @details    This is a private function, but needs to be exposed here since the
 *             log statement is a macro implementation.
 */
void alog_unlock( void );


/**
 * @brief      Checks if requested level can log.
 *
 * @param[in]  level  The level
 *
 * @return     True if log is allowed, false otherwise.
 *
 */
bool alog_check_level( const alog_level_t level );


/**
 * @brief      Load level + category bitmask from ISP3A_LOG_CONF_PATH if it
 *             exists, else leave the compiled defaults untouched. Safe to call
 *             from process context only (it reads a file). Called at ISP init.
 */
void alog_load_config( void );


/**
 * @brief      Sets the runtime category bitmask (see alog_category_t).
 */
void alog_set_categories( const uint32_t categories );


/**
 * @brief      Returns the runtime category bitmask.
 */
uint32_t alog_get_categories( void );


/**
 * @brief      True if any of the requested category bits are currently enabled.
 */
bool alog_check_cat( const uint32_t cat );


/** @brief Sets the sampled-log interval (1 = every frame). */
void alog_set_throttle( const uint32_t n );

/** @brief Sampled-log interval for LOG_CAT_SAMPLED; always >= 1. */
uint32_t alog_throttle( void );


/**
 * @brief      Helper function to return the log level string.
 *
 * @param[in]  level  The level
 *
 * @return     Pointer to log level string from static table.
 *
 * @details    This will always return a valid string, but levels above
 *             LOG_LEVEL_MAX will return an incorrect log level.
 *             It is the users responsibility to provide correct log level.
 */
const char *alog_level_string( const alog_level_t level );


#if defined( SYSTEM_GET_TIME_STAMP ) && !( __KERNEL__ )
/**
 * @brief      Helper log macro.
 *
 * @param      level  The level
 * @param      fmt    The format
 * @param      ...    va args
 *
 * @details    Helper macro to avoid parameter duplication. This version
 *             includes time string for non kernel builds where the user has
 *             provided a time string implementation. Printf like usage:
 * @code
 * LOG( LOG_CRIT, "This is a cool little message with an integer value %d", some_value );
 * @endcode
 *
 * @note       If printk is used you do not need to use any locking since it is
 *             safe to call from anywhere in the kernel.
 */
#define LOG( level, fmt, ... )                                                                 \
    do {                                                                                       \
        if ( alog_check_level( level ) ) {                                                     \
            alog_lock();                                                                       \
            SYSTEM_PRINTF( "%s%s" LOG_COLOR_FILE "%s:%d:%d: " LOG_COLOR_DEFAULT fmt LOG_POST_FIX, \
                           SYSTEM_GET_TIME_STAMP(),                                            \
                           alog_level_string( level ),                                         \
                           __FILENAME__,                                                       \
                           __LINE__,current->pid,                                              \
                           ##__VA_ARGS__ );                                                    \
            alog_unlock();                                                                     \
        }                                                                                      \
    } while ( 0 )


#else /* For kernel debug or if no time func has been provided. */

/**
 * @brief      Helper log macro.
 *
 * @param      level  The level
 * @param      fmt    The format
 * @param      ...    va args
 *
 * @return     { description_of_the_return_value }
 *
 * @details    Helper macro to avoid parameter duplication. Printf like usage:
 * @code
 * LOG( LOG_CRIT, "ISP irq = %d, flags = 0x%x %s id :%d!", (int)isp_irq->start, (int)isp_irq->flags, pdev->name, pdev->id );
 * @endcode
 *
 * @note       If printk is used you do not need to use any locking since it is
 *             safe to call from anywhere in the kernel.
 */
#define LOG( level, fmt, ... )                                                               \
    do {                                                                                     \
        if ( alog_check_level( level ) ) {                                                   \
            alog_lock();                                                                     \
            SYSTEM_PRINTF( "%s" LOG_COLOR_FILE "%s:%d:%d: " LOG_COLOR_DEFAULT fmt LOG_POST_FIX, \
                           alog_level_string( level ),                                       \
                           __FILENAME__,                                                     \
                           __LINE__, current->pid,                                           \
                           ##__VA_ARGS__ );                                                  \
            alog_unlock();                                                                   \
        }                                                                                    \
    } while ( 0 )


#endif /* defined( SYSTEM_GET_TIME_STAMP ) && !( __KERNEL__ ) */


#define LOG_CONT( level, fmt, ... )              \
    do {                                         \
        if ( alog_check_level( level ) ) {       \
            alog_lock();                         \
            SYSTEM_PRINTF( fmt, ##__VA_ARGS__ ); \
            alog_unlock();                       \
        }                                        \
    } while ( 0 )


/**
 * @brief      Category-gated log. Prints only when both the level passes AND at
 *             least one of the requested category bits (alog_category_t) is
 *             enabled by the runtime mask (default: all, or per the on-board
 *             config file). Tag every per-component 3A trace site with this so
 *             QA can isolate one block end-to-end.
 * @code
 * LOG_CAT( ALOG_CAT_AWB, LOG_DEBUG, "awb gains r=%d b=%d", r, b );
 * @endcode
 */
#define LOG_CAT( cat, level, fmt, ... )              \
    do {                                             \
        if ( alog_check_cat( cat ) )                 \
            LOG( level, fmt, ##__VA_ARGS__ );        \
    } while ( 0 )


/**
 * @brief      Category-gated log sampled 1-in-alog_throttle() calls (each site
 *             keeps its own counter). throttle defaults to 100 (thins high-rate
 *             per-frame traces); set `throttle = 1` in the config file to log
 *             every frame when chasing fast oscillation/flip scenarios that a
 *             fixed hard-coded stride would mask.
 */
#define LOG_CAT_SAMPLED( cat, level, fmt, ... )                            \
    do {                                                                   \
        static uint32_t _alog_sample_n = 0;                                \
        if ( alog_check_cat( cat ) &&                                      \
             ( _alog_sample_n++ % alog_throttle() ) == 0 )                 \
            LOG( level, fmt, ##__VA_ARGS__ );                              \
    } while ( 0 )


/* This is neccessary to maintain backwards compatibility, ideally each of this would be function call. Forexample LOG_DEBUG(...). */

#define LOG_DEBUG LOG_LEVEL_DEBUG
#define LOG_INFO LOG_LEVEL_INFO
#define LOG_NOTICE LOG_LEVEL_NOTICE
#define LOG_WARNING LOG_LEVEL_WARNING
#define LOG_ERR LOG_LEVEL_ERROR
#define LOG_CRIT LOG_LEVEL_CRIT
#define LOG_ALERT LOG_LEVEL_ALERT
#define LOG_EMERG LOG_LEVEL_EMERG


/* ============================================================================
 * Per-component verbose traces (now runtime categories, see LOG_CAT above).
 *
 * These were compile-time #ifdef ENABLE_<NAME>_LOGS toggles. They are now
 * runtime LOG_CAT( ALOG_CAT_<NAME>, ... ) sites, gated by the category bitmask
 * loaded from ISP3A_LOG_CONF_PATH -- so QA enables them from the on-board file
 * without a rebuild. Old name -> new category bit:
 *
 *   ENABLE_SBUF_LOGS        -> ALOG_CAT_SBUF     (sbuf ring get/set hot path)
 *   ENABLE_AE_LOGS          -> ALOG_CAT_AE       (AE per-FSM traces)
 *   ENABLE_AWB_LOGS         -> ALOG_CAT_AWB      (AWB per-FSM traces)
 *   ENABLE_GAMMA_LOGS       -> ALOG_CAT_GAMMA    (gamma per-FSM traces)
 *   ENABLE_IRIDIX_LOGS      -> ALOG_CAT_IRIDIX   (iridix per-FSM traces)
 *   ENABLE_V4L2_STREAM_LOGS -> ALOG_CAT_V4L2     (V4L2 buffer-list traces)
 *
 * WARNING: some of these sites live inside spin_lock_irqsave regions on the
 * per-frame hot path. When their category AND the log level are both enabled,
 * the printf under the lock can hold ISP spinlocks with IRQs disabled for tens
 * of ms. That is why the default level (ERROR) already gates out the DEBUG
 * traces even with categories=all: enabling them is an explicit QA action.
 * Enable only the categories you need while reproducing an issue, then delete
 * the config file.
 * ============================================================================*/


#endif // ACAMERA_LOGGER_H
