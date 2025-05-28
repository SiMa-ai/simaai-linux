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

#ifndef __SYSTEM_EXT_MEM_IO_H__
#define __SYSTEM_EXT_MEM_IO_H__

#include "system_types.h"

typedef struct _SMemDescriptor {
    uint32_t offset;
    uint32_t size;
    uint32_t cached;
} SMemDescriptor;


typedef struct _SMemContext {
    SMemDescriptor dsr;
    void *ptr;
} SMemContext;

/**
 *   Initialize external memory access
 *
 *   This function should initialze external memory access.
 *
 *   @param  dsr - input memory descriptor
 *   @param  ctx - output memory context
 *
 *   @return 0 - on success
 *          -1 - on error
 */
int32_t system_ext_mem_init( SMemDescriptor *dsr, SMemContext *ctx );


/**
 *   Close external memory access
 *
 *   This function should close external memory access.
 *
 *   @param  ctx - memory context
 *
 *   @return 0 - on success
 *          -1 - on error
 */
int32_t system_ext_mem_terminate( SMemContext *ctx );


/**
 *   Read 32 bit word from external memory
 *
 *   This function returns a 32 bits word from external memory with a given offset.
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory to read 32 bits word.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_EXT_ADDR
 *
 *   @return 32 bits memory value
 */
uint32_t system_ext_mem_read_32( SMemContext *ctx, uint32_t offset );


/**
 *   Read 16 bit word from external memory
 *
 *   This function returns a 16 bits word from external memory with a given offset.
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory to read 16 bits word.
 *                 Correct values from 0 to max available memory size
 *
 *   @return 16 bits memory value
 */
uint16_t system_ext_mem_read_16( SMemContext *ctx, uint32_t offset );


/**
 *   Read 8 bit word from external memory
 *
 *   This function returns a 8 bits word from external memory with a given offset.
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory to read 8 bits word.
 *                 Correct values from 0 to max available memory size
 *
 *   @return 8 bits memory value
 */
uint8_t system_ext_mem_read_8( SMemContext *ctx, uint32_t offset );


/**
 *   Copies the content of the external memory into the given memory area
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory.
 *                 Correct values from 0 to max available memory size
 *   @param  buffer - destination memory area.
 *   @param  count - the size of area to be copied.
 *                Correct values from 0 to max available memory size minus offset.
 */
void system_ext_mem_read_buffer( SMemContext *ctx, uint32_t offset, uint8_t *buffer, uint32_t count );


/**
 *   Write 32 bits word to external memory
 *
 *   This function writes a 32 bits word to external memory with a given offset.
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory to write data.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_EXT_ADDR.
 *   @param  data - data to be written
 */
void system_ext_mem_write_32( SMemContext *ctx, uint32_t offset, uint32_t data );


/**
 *   Write 16 bits word to external memory
 *
 *   This function writes a 16 bits word to external memory with a given offset.
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory to write data.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_EXT_ADDR.
 *   @param  data - data to be written
 */
void system_ext_mem_write_16( SMemContext *ctx, uint32_t offset, uint16_t data );


/**
 *   Write 8 bits word to external memory
 *
 *   This function writes a 8 bits word to external memory with a given offset.
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory to write data.
 *                 Correct values from 0 to ACAMERA_ISP_MAX_EXT_ADDR.
 *   @param  data - data to be written
 */
void system_ext_mem_write_8( SMemContext *ctx, uint32_t offset, uint8_t data );


/**
 *   Copies the content of the memory area to the external memory
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory.
 *                 Correct values from 0 to max available memory size
 *   @param  buffer - source memory area.
 *   @param  count - the size of area to be copied.
 *                Correct values from 0 to max available memory size minus offset.
 */
void system_ext_mem_write_buffer( SMemContext *ctx, uint32_t offset, const uint8_t *buffer, uint32_t count );


/**
 *   Fill external memory area with zeroes
 *
 *   @param  ctx - memory context.
 *   @param  offset - the offset in external memory.
 *                 Correct values from 0 to max available memory size
 *   @param  count - the size of area to be copied.
 *                Correct values from 0 to max available memory size minus offset.
 */
void system_ext_mem_write_zero( SMemContext *ctx, uint32_t offset, uint32_t count );

#endif /* __SYSTEM_EXT_MEM_IO_H__ */
