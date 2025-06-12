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

#ifndef __ACAMERA_MATH_H__
#define __ACAMERA_MATH_H__

#include "system_types.h"
#include "acamera_configuration.h"

#ifndef MODULATION_ENTRY_DEFINED
#define MODULATION_ENTRY_DEFINED
typedef struct {
    uint16_t x, y;
} modulation_entry_t;
#endif /*MODULATION_ENTRY_DEFINED*/

typedef struct {
    uint32_t x, y;
} modulation_entry_32_t;

#if __KERNEL__
#include <asm/div64.h>
#include "linux/math64.h"
//#define usleep(a) usleep_range((a),(a)+100)
#else
#define div64_u64( x, y ) ( ( x ) / ( y ) )
#define div64_s64( x, y ) ( ( x ) / ( y ) )
#endif

#ifndef ACAMERA_ABS
#define ACAMERA_ABS( a ) ( ( a ) >= 0 ? ( a ) : -( a ) )
#endif

#ifndef U16_MAX
#define U16_MAX 0xFFFF
#endif


#ifndef ACAMERA_SIGN
#define ACAMERA_SIGN( a ) ( ( a ) >= 0 ? ( 1 ) : ( -1 ) )
#endif /*ACAMERA_SIGN*/

#ifndef ACAMERA_MIN
#define ACAMERA_MIN( a, b ) ( ( a ) >= b ? ( b ) : ( a ) )
#endif /*ACAMERA_MIN*/

#ifndef ACAMERA_MAX
#define ACAMERA_MAX( a, b ) ( ( a ) >= b ? ( a ) : ( b ) )
#endif /*ACAMERA_MAX*/

#ifndef ACAMERA_ABSDIFF
#define ACAMERA_ABSDIFF( a, b ) ( ( a ) > ( b ) ? ( a - b ) : ( b - a ) )
#endif /*ACAMERA_ABSDIFF*/

#define LIN_EQUATION_FRACTION_SIZE 5


#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif
#ifndef MAX
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )
#endif

/* A macro to check if x >= low && x <= high. Can be used to for validation.
 Works for signed values. For older compiler versions (gcc6) using subtraction
 is faster than comparisons*/
#define IN_RANGE( x, high, low ) ( ( ( x ) - ( high ) ) * ( ( x ) - ( low ) ) <= 0 )

#if defined( ARRAY_SIZE )
#undef ARRAY_SIZE
#endif

#define ARRAY_SIZE( a ) \
    ( sizeof( a ) / sizeof( a[0] ) )
#define ARRAY_2D_COLUMNS( a ) \
    ( sizeof( a[0] ) / sizeof( a[0][0] ) )
#define ARRAY_2D_ROWS( a ) \
    ( sizeof( a ) / sizeof( a[0] ) )


#define round_shift( a, sh ) ( ( ( a ) >> ( sh ) ) + ( ( ( a ) >> ( sh - 1 ) ) & 1 ) )
#define PI_Q12 12868 //Q12 Format

#define bitrshift( a, sh ) ( ( ( sh ) >= 0 ) ? ( ( a ) >> ( sh ) ) : ( ( a ) << -( sh ) ) )

uint8_t leading_one_position( const uint32_t in );
uint32_t sqrt64( uint64_t arg );
uint16_t sqrt32( uint32_t arg );
uint32_t log2_int_to_fixed( const uint32_t val, const uint8_t out_precision, const uint8_t shift_out );
uint32_t log2_fixed_to_fixed( const uint32_t val, const int in_fix_point, const uint8_t out_fix_point );
int32_t log2_fixed_to_fixed_64( uint64_t val, int32_t in_fix_point, uint8_t out_fix_point );
uint32_t math_exp2( uint32_t val, const unsigned char shift_in, const unsigned char shift_out );
uint8_t sqrt16( uint16_t arg );
uint8_t log16( uint16_t arg );
uint32_t math_log2( const uint32_t val, const uint8_t out_precision, const uint8_t shift_out );
uint32_t multiplication_fixed_to_fixed( uint32_t a, uint32_t b, const int x1, const int x2 );

int32_t solving_lin_equation_a( int32_t y1, int32_t y2, int32_t x1, int32_t x2, int16_t a_fraction_size );
int32_t solving_lin_equation_b( int32_t y1, int32_t a, int32_t x1, int16_t a_fraction_size );
int32_t solving_nth_root_045( int32_t x, const int16_t fraction_size );
uint32_t div_fixed( uint32_t a, uint32_t b, int16_t a_fraction_size );
uint16_t sqrt32( uint32_t arg );

uint16_t line_offset( uint16_t line_len, uint8_t bytes_per_pixel );
int16_t acamera_cosine( uint32_t theta );
int16_t acamera_sine( uint32_t theta );

#define ACAMERA_MODULO( N, D ) ( ( N ) - ( ( ( N ) / ( D ) ) * ( D ) ) )


uint16_t calc_modulation_u16( uint16_t x, const modulation_entry_t *p_table, int table_len );
uint32_t calc_modulation_u32( uint32_t x, const modulation_entry_32_t *p_table, int table_len );

uint16_t calc_scaled_modulation_u16( uint16_t x, uint16_t target_min_y, uint16_t target_max_y, const modulation_entry_t *p_table, int table_len );

uint16_t calc_equidistant_modulation_u16( uint16_t x, const uint16_t *p_table, uint16_t table_len );
uint32_t calc_equidistant_modulation_u32( uint32_t x, const uint32_t *p_table, uint32_t table_len );
uint16_t calc_inv_equidistant_modulation_u16( uint16_t x, const uint16_t *p_table, uint16_t table_len );
uint32_t calc_inv_equidistant_modulation_u32( uint32_t x, const uint32_t *p_table, uint32_t table_len );


#endif /* __ACAMERA_MATH_H__ */
