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

/**************************************************
 * Headers
 **************************************************/
#include "module_mcfe_usecase.h"
#include "module_mcfe.h"
#include "module_mcfe_common.h"
#include "module_mcfe_service.h"

extern int module_mcfe_usecase_init_tdmf( module_mcfe_usecase_config_t *, mcfe_slot_id_t, int );
extern int module_mcfe_usecase_init_m2m( module_mcfe_usecase_config_t *, mcfe_slot_id_t, int );
module_mcfe_usecase_init_func_t usecase_type_init_funcs[MODULE_MCFE_USECASE_MAX] = {
    module_mcfe_usecase_init_tdmf,
    module_mcfe_usecase_init_m2m};

extern int module_mcfe_usecase_deinit_tdmf( module_mcfe_usecase_config_t * );
extern int module_mcfe_usecase_deinit_m2m( module_mcfe_usecase_config_t * );
module_mcfe_usecase_deinit_func_t usecase_type_deinit_funcs[MODULE_MCFE_USECASE_MAX] = {
    module_mcfe_usecase_deinit_tdmf,
    module_mcfe_usecase_deinit_m2m};

/**************************************************
 * Init, deinit
 **************************************************/
int module_mcfe_usecase_init( module_mcfe_usecase_config_t *config, module_mcfe_usecase_type_t type, mcfe_slot_id_t slot_id, int hist_position_is_be )
{
    // Check input parameters.
    if ( config == NULL ) {
        LOG( LOG_ERR, "Error. Invalid parameter. Instance pointer is NULL." );
        return MCFE_ERR_INV_PARM;
    }

    if ( ( type <= MODULE_MCFE_USECASE_NONE ) || ( type > MODULE_MCFE_USECASE_MAX ) ) {
        LOG( LOG_ERR, "Error. Invalid parameter. Type value is out of range: %d.", type );
        return MCFE_ERR_INV_PARM;
    }

    if ( slot_id >= MODULE_MCFE_SLOT_ARRAY_SIZE ) {
        LOG( LOG_ERR, "Error. Invalid parameter. Slot id value is out of range: %u.", slot_id );
        return MCFE_ERR_INV_PARM;
    }

    if ( hist_position_is_be != 0 && hist_position_is_be != 1 ) {
        LOG( LOG_ERR, "Error. Invalid parameter. Histogram position value is out of range: %d.", hist_position_is_be );
        return MCFE_ERR_INV_PARM;
    }

    return usecase_type_init_funcs[type - 1]( config, slot_id, hist_position_is_be );
}

int module_mcfe_usecase_deinit( module_mcfe_usecase_config_t *config )
{
    // Check input parameters.
    if ( config == NULL ) {
        LOG( LOG_ERR, "Error. Invalid parameter. Instance pointer is NULL." );
        return MCFE_ERR_INV_PARM;
    }

    if ( ( config->type <= MODULE_MCFE_USECASE_NONE ) || ( config->type > MODULE_MCFE_USECASE_MAX ) ) {
        LOG( LOG_ERR, "Error. Invalid parameter. Type value is out of range: %d.", config->type );
        return MCFE_ERR_INV_PARM;
    }

    return usecase_type_deinit_funcs[config->type - 1]( config );
}
