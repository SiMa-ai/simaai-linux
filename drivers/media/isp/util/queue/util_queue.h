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

#ifndef __UTIL_QUEUE_H__
#define __UTIL_QUEUE_H__

#include "system_types.h"

typedef struct aqueue_t aqueue;
struct aqueue_t {
    uint32_t filled;   //Amount of in use elements inside of the queue
    uint32_t head;     //Id of the ring buffer head
    uint32_t tail;     //Id of the ring buffer tail
    uint32_t max_size; //Max elements that could be handled by the queue
    void **array;      //Array of elements
};

/**
 * @brief   Initializes queue data structure
 *
 * @param   hndl handle to aqueue data structure
 * @param   holders ptr to array of holders
 * @param   max_size max amount of elements to handle
 *
 * @return  0 on success
 *
 * @details This function should be called by the user to initialise the queue
 */
void aqueue_create( aqueue *hndl, void **holders, uint32_t max_size );

/**
 * @brief   Deinitializes queue data structure
 *
 * @param   hndl handle to aqueue data structure
 *
 * @return  0 on success
 *
 * @details This function should be called by the user to deinitialise the queue
 */
void aqueue_destroy( aqueue *hndl );

/**
 * @brief   Enqueue data in the queue
 *
 * @param   hndl handle to aqueue data structure
 *
 * @return  0 on success
 *
 * @details This function should be called by the user to enqueue an element
 */
int aqueue_enqueue( aqueue *hndl, void *element );

/**
 * @brief   Dequeue data in the queue
 *
 * @param   hndl handle to aqueue data structure
 *
 * @return  0 on success
 *
 * @details This function should be called by the user to dequeue an element
 */
void *aqueue_dequeue( aqueue *hndl );

/**
 * @brief   Returns number of elements in queue
 *
 * @param   hndl handle to aqueue data structure
 *
 * @return  0 on success
 *
 * @details This function should be called by the user to query the number of
 *          elements contained in the queue
 */
uint32_t aqueue_size( aqueue *hndl );

#endif