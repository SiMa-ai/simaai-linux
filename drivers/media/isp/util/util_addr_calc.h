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

#ifndef __UTIL_ADDR_CALC_H__
#define __UTIL_ADDR_CALC_H__

//SYSPHY - system physical addr ; ISPAS - ISP address space
#define ADDR_SYSPHY2ISPAS( x ) ( ( x ) + ( ISPAS_MINUS_SYSPHY ) )
#define ADDR_ISPAS2SYSPHY( x ) ( ( x ) - ( ISPAS_MINUS_SYSPHY ) )


#endif /* __UTIL_ADDR_CALC_H__ */
