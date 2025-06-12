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

#ifndef _REVISION_H_
#define _REVISION_H_

#define FIRMWARE_REVISION 0x1a36a296u
#define FIRMWARE_VERSION ( ( 2 * 100 ) + ( 0 ) )
#define FIRMWARE_VERSION_R 2
#define FIRMWARE_VERSION_P 0
#define FIRMWARE_VERSION_STR "2.0"


#define ISP_RTL_VERSION ( ( ISP_RTL_VERSION_R * 100 ) + ( ISP_RTL_VERSION_P ) )
#define ISP_RTL_VERSION_STR "1.2"
#define ISP_RTL_SVN_REVISION_STR "r101397"

#endif /*_REVISION_H_*/
