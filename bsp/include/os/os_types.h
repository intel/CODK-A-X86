/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __OS_TYPES_H__
#define __OS_TYPES_H__

#include "stddef.h"
#include "stdbool.h"
#include "stdint.h"

/** @addtogroup os
 * @{
 */
/**********************************************************
 ************** General definitions  **********************
 **********************************************************/
#define UNUSED(p) (void)(p)     /**< Macro for unused arguments, to prevent warnings */

/** Generic OS type for execution status. */
typedef enum {
    E_OS_OK = 0,                  /**< Generic OK status. */
    /* use negative values for errors */
    E_OS_ERR = -1,                /**< Generic error status. */
    E_OS_ERR_TIMEOUT = -2,        /**< Timeout expired. */
    E_OS_ERR_BUSY = -3,           /**< Resource is not available. */
    E_OS_ERR_OVERFLOW = -4,       /**< Service would cause an overflow. */
    E_OS_ERR_EMPTY = -5,          /**< No data available (e.g.  queue is empty). */
    E_OS_ERR_NOT_ALLOWED = -6,    /**< Service is not allowed in current execution context. */
    E_OS_ERR_NO_MEMORY = -7,      /**< All allocated resources are already in use */
    E_OS_ERR_NOT_SUPPORTED = -8,  /**< Service is not supported on current context or OS. */
    E_OS_WDT_EXPIRE=-9,           /**< Quark watchdog expire. */
    /* more error codes to be defined */
    E_OS_ERR_UNKNOWN = - 100,     /**< Invalid error code (bug?). */
} OS_ERR_TYPE;



/* Types for kernel objects. */
typedef void* T_SEMAPHORE;                /**< Handler of Semaphore. */
typedef void* T_MUTEX;                    /**< Handler of Mutex. */
typedef void* T_QUEUE;                    /**< Handler of Queue. */
typedef void* T_QUEUE_MESSAGE;            /**< Handler of QueueMessage. */
typedef void* T_TIMER;                    /**< Handler of Timer. */
typedef void (* T_ENTRY_POINT) (void* );  /**< Handler of Entry Point. */
typedef void* T_TASK;                     /**< Handler of Task. */
typedef uint8_t T_TASK_PRIO;              /**< Handler of Task Priority. */


/* States of a task. */
typedef enum {
    E_TASK_UNCREATED = 0,   /**< Task was not created */
    E_TASK_RUNNING,         /**< Task is running */
    E_TASK_SUSPENDED,       /**< Task is suspended */
} T_TASK_STATE;


/* Special values for "timeout" parameter. */
#define OS_NO_WAIT                0     /**< The blocking function returns immediately */
#define OS_WAIT_FOREVER           -1    /**< The blocking function will wait indefinitely */

/** @}*/

#endif
