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

/**
 * @defgroup os_zephyr Zephyr OS Abstraction Layer
 * Implementation of the "OS abstraction API" for Zephyr.
 *
 * It supports _microkernel_ and _nanokernel_. Their main difference is that the
 * nanokernel only has one thread, whereas microkernel can have multiple
 * threads/tasks/fibers.
 *
 * @ingroup os
 * @{
 */

/**
 * @file common.c
 *
 * ZEPHYR OS abstraction / internal utilities
 *
 * Functions are exported by common.h
 *
 */

/******* Framework headers : */
#include "os/os_types.h"    /* framework-specific types */
#include "zephyr/os_specific.h"
#include "infra/panic.h"


#include "nanokernel.h"
#ifdef   CONFIG_MICROKERNEL
#include "microkernel.h"
#include "zephyr.h"
#endif

#include "balloc_debug.h"

extern void framework_init_sync(void);
extern void framework_init_queue(void);
extern void framework_init_interrupt(void);
extern void framework_init_timer(void);
/********************/




/**********************************************************
 ************** Extern variables   ************************
 **********************************************************/

/**********************************************************
 ************** Local definitions  ************************
 **********************************************************/
#ifdef CONFIG_NANOKERNEL
static struct nano_sem _PoolLockSem ;
#else /* CONFIG_MICROKERNEL */
static ksem_t   _PoolLockSem ;
#endif




/**********************************************************
 ************** Private variables  ************************
 **********************************************************/

/**********************************************************
 ************** Forward declarations **********************
 **********************************************************/

/**********************************************************
 ************** Private functions  ************************
 **********************************************************/

/**********************************************************
 ************** Exported functions ************************
 **********************************************************/

/**
 * Locks access to a pool of synchronization objects.
 *
 * This function manages the concurrent accesses to the pools
 * of semaphores and mutexes.
 *
 * This function is only used when creating and deleting semaphores or mutexes.
 *
 * Rationale:
 *
 *  This function does not manage concurrent accesses to the pools when running
 *  from an ISR context. ( Creating a semaphore or a mutex in an ISR does not
 *  seem a valid use-case anyway ).
 */
void _PoolLock ( void )
{
    T_EXEC_LEVEL execLvl;

    execLvl = _getExecLevel();

    /* use the private semaphore to manage access to the pool of sync objects */
    switch ( execLvl )
    {
    case E_EXEC_LVL_FIBER :
#ifdef CONFIG_NANOKERNEL
        nano_fiber_sem_take_wait (&_PoolLockSem) ;
#else
        /* TODO:  where is FIBER_SemaTestW ? */
        (void) task_sem_take_wait( _PoolLockSem ) ;
#endif
        break;

    case E_EXEC_LVL_TASK :
#ifdef CONFIG_NANOKERNEL
        nano_task_sem_take_wait (&_PoolLockSem) ;
#else
        (void) task_sem_take_wait ( _PoolLockSem ) ;
#endif
        break;

    default:
        /* do nothing */
        break;
    }
}

/**
 * Unlocks access to a pool of synchronization objects.
 *
 * This function manages the concurrent accesses to the pools
 * of semaphores and mutexes.
 *
 * This function is only used when creating and deleting semaphores or mutexes.
 *
 * Rationale:
 *
 *  This function does not manage concurrent accesses to the pools when running
 *  from an ISR context. ( Creating a semaphore or a mutex in an ISR does not
 *  seem a valid use-case anyway ).
 *
 */
void _PoolUnlock ( void )
{
    T_EXEC_LEVEL execLvl;

    execLvl = _getExecLevel();

    /* free up the private semaphore  */
    switch ( execLvl )
    {
    case E_EXEC_LVL_FIBER :
#ifdef CONFIG_NANOKERNEL
        nano_fiber_sem_give (&_PoolLockSem) ;
#else
        fiber_sem_give ( _PoolLockSem, (struct cmd_pkt_set *) 0 ) ;
#endif
        break;

    case E_EXEC_LVL_TASK :
#ifdef CONFIG_NANOKERNEL
        nano_task_sem_give (&_PoolLockSem) ;
#else
        task_sem_give ( _PoolLockSem ) ;
#endif
        break;

    default:
        /* do nothing */
        break;
    }
}


/**
 * Initializes private variables.
 *
 * IMPORTANT : this function must be called during the initialization
 *             of the OS abstraction layer.
 *             This function shall only be called once after reset, otherwise
 *             it may cause the take/lock and give/unlock services to fail.
 */
void framework_init_common (void)
{
#ifdef CONFIG_NANOKERNEL
    nano_sem_init (&_PoolLockSem);
    nano_task_sem_give (&_PoolLockSem);
#else
    _PoolLockSem = FWK_SYNC_SEM; /* FWK_SYNC_SEM is defined by allnodes.h */
#endif

    _PoolUnlock();

    /* initialize all modules of the OS abstraction */
    framework_init_sync();
    framework_init_queue();
    framework_init_interrupt();
    framework_init_timer();
}


/**
 * Copies error code to caller's variable, or panics if caller did not specify
 * an error variable.
 *
 * @param [out] err : pointer to caller's error variable - may be _NULL
 * @param [in] localErr : error detected by the function
 *
 */
void error_management (OS_ERR_TYPE* err, OS_ERR_TYPE localErr)
{
    if ( NULL != err )
    {
        *err = localErr ;
    }
    else
    {
        if ( E_OS_OK != localErr)
        {
#ifdef CONFIG_BALLOC_STATISTICS_TRACK_OWNER
            if (localErr == E_OS_ERR_NO_MEMORY)
                print_pool();
#endif
            /* TODO: clean-up */
            panic (localErr);
        }
    }
}



/**
 * Converts VxMicro's semaphore API return value to OS_ERR_TYPE error code.
 *
 * @param [in] vxErr : error code returned by the microkernel's semaphore functions
 *
 * @return framework standardized error code (::OS_ERR_TYPE)
 */
OS_ERR_TYPE _VxErrToOsErr ( int vxErr )
{
    OS_ERR_TYPE osErr = E_OS_ERR_UNKNOWN ;

#ifdef CONFIG_NANOKERNEL
    switch ( vxErr )
    {
    case 0:
        osErr = E_OS_ERR_BUSY;
        break;

    case 1:
        osErr = E_OS_OK;
        break;

    default:
        break;
    }

#else

    switch ( vxErr )
    {
    case RC_OK:
        osErr = E_OS_OK;
        break;

    case RC_FAIL:
        osErr = E_OS_ERR_BUSY;
        break;

    case RC_TIME:
        osErr = E_OS_ERR_TIMEOUT;
        break;

    default:
        break;
    }
#endif
    return (osErr);
}


#ifdef CONFIG_MICROKERNEL
/* convert os abstraction timeout to VxMciro TICK */
int32_t convert_milliseconds_to_ticks(int time_in_ms)
{
#if TICKFREQ == 1000 && TICKS_UNLIMITED == OS_WAIT_FOREVER && TICKS_NONE == OS_NO_WAIT
    return (int32_t)time_in_ms;
#else
    if(time_in_ms == OS_NO_WAIT)
    {
        return (int32_t)TICKS_NONE;
    }
    else if(time_in_ms == OS_WAIT_FOREVER)
    {
        return (int32_t)TICKS_UNLIMITED;
    }
    else
    {
        return CONVERT_MS_TO_TICKS(time_in_ms);
    }
#endif
}
#endif

/** @} */
