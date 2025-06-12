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

#ifndef __SYSTEM_FILE_IO_H__
#define __SYSTEM_FILE_IO_H__

#include "system_types.h"

/**
 *   Open a file
 *
 *   This function returns a pointer to a file descriptor which
 *   should be used for any other file io operations
 *
 *   @param path - path to the file
 *   @param @flags - flags for the open
 *
 *
 *   @return pointer to the file descriptor or NULL if failed
 */

void *system_file_open( const char *path, uint32_t flags );


/**
 *   Write data to the file
 *
 *   This function writes input dat to the file. File must be opened in advance.
 *
 *   @param file - pointer to the file descriptor
 *   @param offset - write offset
 *   @param data - pointer to the data to be written
 *   @param size - write size
 *
 *   @return number of bytes written or -1 on error
 */
int32_t system_file_write( void *file, uint64_t offset, uint8_t *data, uint32_t size );


/**
 *   Read data from the file
 *
 *   This function reads data from the file. File must be opened in advance.
 *
 *   @param file - pointer to the file descriptor
 *   @param offset - read offset
 *   @param data - pointer to the data to be read
 *   @param size - read size
 *
 *   @return number of bytes read or -1 on error
 */
int32_t system_file_read( void *file, uint32_t offset, uint8_t *data, uint32_t size );


/**
 *   Close the file
 *
 *   This function closes the file which was open by system_file_open
 *
 *   @param file - pointer to a file descriptor
 *
 *   @return 0 on success or -1 on error
 */
int32_t system_file_close( void *file );


#endif /* __SYSTEM_FILE_IO_H__ */
