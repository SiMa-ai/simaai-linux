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

/** @file util_crc16.h
    @addtogroup
    @{
*/

#ifndef _UTIL_CRC16_H_
#define _UTIL_CRC16_H_

#include "util_crc.h" //config

#if CRC16_POLY_CCITT
uint16_t acrc_ccitt_memory( const uint16_t init, const unsigned char *data,
                            const size_t sz );
uint16_t acrc_ccitt_byte( const uint16_t init, const unsigned char byte );
#endif /*CRC16_POLY_CCITT*/

#endif

/** @}*/