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

#include "acamera_isp_ctx.h"

void ae_histogram_dgain_compensation( histogram_fsm_ptr_t p_fsm );
void configure_histogram_neq_lut( histogram_fsm_const_ptr_t p_fsm, int input );
void histogram_configure_position( histogram_fsm_const_ptr_t p_fsm );
void ae_update_histogram_shading_lut( histogram_fsm_const_ptr_t p_fsm, int input );
void ae_update_histogram_wbgain( histogram_fsm_const_ptr_t p_fsm, int input );
void ae_update_histogram_black_level( histogram_fsm_const_ptr_t p_fsm, int input );
void ae_update_histogram_geometry( histogram_fsm_const_ptr_t p_fsm, int input );
void ae_update_histogram_zone_weights( histogram_fsm_const_ptr_t p_fsm, int input );
