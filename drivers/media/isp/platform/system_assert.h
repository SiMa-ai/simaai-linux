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

#ifndef __SYSTEM_ASSERT_H__
#define __SYSTEM_ASSERT_H__

/** @file system_assert.h
    @addtogroup   platform Platform
    @{
*/

/**
 * @brief      Calls when assertion fails.
 *
 * @param      file        The file
 * @param[in]  line        The line
 * @param      expression  The expression
 *
 * @details    Not to be called by user.
 * @warning    This handler is fatal and will not return.
 */
void _assert_handler( char const *const file, const int line, char const *const expression ) __attribute__( ( noreturn ) );

#ifdef assert
#undef assert
#endif
/**
 * @brief      Assert macro
 *
 * @param      expression  Expression to check
 *
 * @details    Calls handler with current file and line if the assertion is
 *             false.
 */
#define assert( expression ) \
    if ( !( expression ) )   \
    _assert_handler( __FILE__, __LINE__, #expression )


/** @} */

#endif /*__SYSTEM_ASSERT_H__*/
