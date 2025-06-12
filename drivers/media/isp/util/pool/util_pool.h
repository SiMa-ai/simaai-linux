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

#ifndef __UTIL_POOL_H__
#define __UTIL_POOL_H__

#include "system_types.h"

typedef struct _apool_element_t apool_element;
struct _apool_element_t {
    void *data; //Ptr do data
};

typedef struct _apool_t apool;
struct _apool_t {
    uint32_t max_size;       //Max elements in pool
    uint32_t current_size;   //Current element count
    apool_element *elements; //Ptr to element array
};

/**
 * @brief   Initializes pool data structure
 *
 * @param   hndl handle to apool data structure
 * @param   elements ptr to element array
 * @param   max_size max amount of elements to handle
 *
 * @return  0 on success
 *
 * @details This function should be called by the user to initialise the pool
 */
void apool_create( apool *hndl, apool_element *elements, uint32_t max_size );

/**
 * @brief   Deinitializes pool data structure
 *
 * @param   hndl handle to apool data structure
 *
 * @return  0 on success
 *
 * @details This function should be called when the user has finished using the pool
 */
void apool_destroy( apool *hndl );

/**
 * @brief   Adds an element to the pool
 *
 * @param   hndl handle to apool data structure
 * @param   dat Ptr to stored data
 *
 * @return  0 on success
 *
 * @details This function should be called when the user want to add an element to the pool
 */
int apool_add( apool *hndl, void *data );

/**
 * @brief   Removes an element to the pool
 *
 * @param   hndl handle to apool data structure
 * @param   data Ptr to stored data to compare with
 *
 * @return  The data of the matching removed element (if any), or NULL.
 *
 * @details This function should be called when the user want to remove an element to the pool
 */
void *apool_remove( apool *hndl, void *data );

/**
 * @brief   Returns a random in use element
 *
 * @param   hndl handle to apool data structure
 *
 * @return  The data of the removed used element (if any), or NULL.
 *
 * @details This function should be called when the user want to remove an arbitrary
 *          in use element to the pool
 */
void *apool_remove_used( apool *hndl );

/**
 * @brief   Checks wether a specific data is contained in the pool
 *
 * @param   hndl handle to apool data structure
 *
 * @param   data ptr to adress of data
 *
 * @return  Returns a 0 when the elements is contained in the pool
 *
 * @details This function should be called when the user want to remove an arbitrary
 *          in use element to the pool
 */
int apool_contains( apool *hndl, void *data );

/**
 * @brief   Returns count of elements in pool
 *
 * @param   hndl handle to apool data structure
 *
 * @return  0 on success
 *
 * @details This function should be called when the user want to check the count
 *          of in use elements in the pool
 */
uint32_t apool_size( apool *hndl );

#endif