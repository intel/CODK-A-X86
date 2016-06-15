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
 * @ingroup os
 * @{
 */

/**
 * \file timer.c
 *
 * ZEPHYR OS abstraction / timer services
 *
 * Functions are exported by os.h
 *
 */


/******* Framework headers : */
#include "os/os.h"             /* framework export definitions */
#include "os/os_types.h"       /* framework-specific types */
#include "zephyr/os_specific.h" /* zephyr-specific definitions */
#include "zephyr/os_config.h"
#include "zephyr/common.h"

#ifdef CONFIG_NANOKERNEL
#define _GET_TICK()             ( nano_tick_get_32() )
#elif CONFIG_MICROKERNEL
int32_t convert_milliseconds_to_ticks(int time_in_ms);
#define _GET_TICK()             ( task_tick_get_32() )
#endif


#include "nanokernel.h"   /* ZEPHYR nanokernel API declaration */

#ifdef   CONFIG_MICROKERNEL
#include "zephyr.h"

#endif

/********************/


/**********************************************************
 ************** Extern variables   ************************
 **********************************************************/
extern int tickunit;

/**********************************************************
 ************** Local definitions  ************************
 **********************************************************/
/*type for timer status */
typedef enum
{
    E_TIMER_READY = 0,
    E_TIMER_RUNNING
} T_TIMER_STATUS;

/** Timer descriptor */
typedef struct {
    T_ENTRY_POINT callback;   /* function to call on timer expiration -- a NULL value means the timer is not running */
    void* data;               /* data to provide to the callback */
    uint32_t expiration;        /* tick in us when timer is due to expire */
    uint32_t delay;             /* timer "timeout" in us -- used for repeating timers */
    uint8_t repeat;          /* specifies if timer shall be automatically restarted upon expiration */
    uint8_t status;          /* describe the timer state */
} T_TIMER_DESC;

/** Chained list of timers */
typedef struct _timer_list {
    T_TIMER_DESC desc;
    struct _timer_list* prev;
    struct _timer_list* next;
}T_TIMER_LIST_ELT;



/**
 * Return the current tick, converted to microseconds.
 *   Tick is read on HW timer.
 *
 * MicroK:
 *     According to tick.h (line 69), ISR_HighTimerRead is task_node_cycle_get_32.
 *     Hence, this function always calls task_node_cycle_get_32, whatever the execution
 *     context.
 *     ticktime is set by nanotime.c
 *
 * NanoK:
 *     Neither the programmer's guide nor the source code state limitations
 *     related to the execution context concerning nano_node_cycle_get_32.
 *     ticktime and tickunit are set by NODE1.c
 *
 */


/**********************************************************
 ************** Private variables  ************************
 **********************************************************/
#ifdef CONFIG_NANOKERNEL
/** Semaphore used for the timer callbacks */
static struct nano_sem g_TimerSem ;
/** Stack for Timer fiber */
static uint32_t g_TimerFiberStack [TIMER_CBK_TASK_STACK_SIZE/sizeof(uint32_t)] ;
/** Single kernel timer used to implement all provided timers */
struct nano_timer g_NanoTimer;
/** Data word used by the nanoK to manage g_NanoTimer */
uint32_t     g_NanoTimerData;
#define NANO_TICK 10
#else /* CONFIG_MICROKERNEL */
/** Semaphore used for the timer callbacks */
static ksem_t g_TimerSem ;

#endif

/** Pool of timers */
DECLARE_BLK_ALLOC(g_TimerPool, T_TIMER_LIST_ELT, TIMER_POOL_SIZE) /* see common.h */

/** Head of the chained list that sorts active timers according to their expiration date */
T_TIMER_LIST_ELT* g_CurrentTimerHead;

/**********************************************************
 ************** Forward declarations **********************
 **********************************************************/
static void signal_timer_task (void);
static bool is_after_expiration (uint32_t tick, T_TIMER_DESC* tmrDesc);
static void add_timer (T_TIMER_LIST_ELT* newTimer);
static void remove_timer (T_TIMER_LIST_ELT* timerToRemove);
static void execute_callback (T_TIMER_LIST_ELT* expiredTimer);

void timer_task(int dummy1, int dummy2);

/**********************************************************
 ************** Private functions  ************************
 **********************************************************/
/**
 * Signal g_TimerSem to wake-up timer_task.
 *
 */
static void signal_timer_task (void)
{
    T_EXEC_LEVEL execLvl;

    execLvl = _getExecLevel();
    /* call the nanoK service that corresponds to the current execution level */
    switch ( execLvl )
    {
    case E_EXEC_LVL_ISR:
#ifdef   CONFIG_NANOKERNEL
        nano_fiber_timer_stop (&g_NanoTimer);
#else  /* -> CONFIG_MICROKERNEL */
        isr_sem_give ( g_TimerSem, NULL ) ;
#endif
        break;

    case E_EXEC_LVL_FIBER:
#ifdef   CONFIG_NANOKERNEL
        nano_fiber_timer_stop (&g_NanoTimer);
#else  /* -> CONFIG_MICROKERNEL */
        fiber_sem_give ( g_TimerSem, NULL ) ;
#endif
        break;

    case E_EXEC_LVL_TASK:
#ifdef   CONFIG_NANOKERNEL
        nano_task_timer_stop (&g_NanoTimer);
#else  /* -> CONFIG_MICROKERNEL */
        task_sem_give ( g_TimerSem ) ;
#endif
        break;
    default: /* will not do that from unknown context */
        panic (E_OS_ERR_NOT_SUPPORTED) ;
        break;
    }
}


#ifdef __DEBUG_OS_ABSTRACTION_TIMER
/**
 *  Print the list of active timers, from tail (first to expire) to head (last to expire).
 */
static void display_list (void)
{
    T_TIMER_LIST_ELT* tbv;
    tbv = g_CurrentTimerHead;
    if ( NULL == tbv)
    {
        _log (" list is empty");
    }
    else
    {
        _log (" Head = 0x%x", (uint32_t) tbv);
        while ( NULL != tbv )
        {
            _log (" -> 0x%x" , (uint32_t) tbv->next);
            tbv = tbv->next;
        }
    }
}
#endif

/**
 * Returns whether the provided tick
 *     is before or after the expiration
 *     of a timer.
 *
 * @param tick date to compare
 * @param tmrDesc timer descriptor
 *
 * @return true if tick parameter represents
 *     a date after the expiration of the
 *     timer
 *     or false if the timer descriptor is null
 *     or if tick represents a date before the
 *     expiration of the timer
 *
 */
static bool is_after_expiration (uint32_t tick, T_TIMER_DESC* tmrDesc)
{
    if ( NULL != tmrDesc )
    {
        /* timer will expire before the tick rolls over */
        return ( tick >= tmrDesc->expiration );
    }
    return false;
}


/**
 * Insert a timer in the list of active timer,
 *    according to its expiration date.
 *
 * @param newTimer pointer on the timer to insert
 *
 * WARNING: newTimer MUST NOT be null (rem: static function )
 *
 */
static void add_timer (T_TIMER_LIST_ELT* newTimer)
{
    T_TIMER_LIST_ELT* insertionPoint;
    T_TIMER_LIST_ELT* listTail;
    bool found;

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
    _log ("\nINFO : add_timer - start: adding 0x%x to expire at %d (now = %d - delay = %d - ticktime = %d)", (uint32_t) newTimer, newTimer->desc.expiration, _GET_TICK(), newTimer->desc.delay, sys_clock_us_per_tick);
    display_list();
#endif


    if ( NULL == g_CurrentTimerHead )
    {
        /* there is no active timer, make newTimer the head timer */
        g_CurrentTimerHead = newTimer;
        newTimer->prev = NULL ;
        newTimer->next = NULL ;
    }
    else
    {
        /* find the next timer to expire after newTimer->desc.expiration */
        insertionPoint = g_CurrentTimerHead ;
        listTail = g_CurrentTimerHead ;
        found  = false;
        while ( ( NULL != insertionPoint ) && ( false == found ) )
        {
            if ( !is_after_expiration(newTimer->desc.expiration, &(insertionPoint->desc) ) )
            { /* newTimer is due to expire before insertionPoint */
                if ( g_CurrentTimerHead == insertionPoint )
                {/*  insert newTimer before the head timer  */
                    g_CurrentTimerHead->prev  = newTimer;
                    newTimer->prev = NULL ;
                    newTimer->next = g_CurrentTimerHead;
                    g_CurrentTimerHead = newTimer;
                }
                else
                {/* insert newTimer between insertionPoint and insertionPoint->prev */
                    newTimer->prev = insertionPoint->prev ;
                    newTimer->next = insertionPoint;
                    insertionPoint->prev =  newTimer;
                    newTimer->prev->next = newTimer;
                }
                /* done */
                found = true ;
            }
            else
            { /* newTimer is due to expire after insertionPoint, continue searching */
                listTail = insertionPoint;/* update list head marker = last non null insertionPoint */
                /* move downward */
                insertionPoint = insertionPoint->next;
            }
        }

        if( ( false == found ) && ( NULL != listTail) )
        {
            /* newTimer is due to expire after all others have expired */
            listTail->next = newTimer;
            newTimer->prev = listTail;
        }
    }

    newTimer->desc.status = E_TIMER_RUNNING;

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
    _log ("\nINFO : add_timer - end ");display_list();
    display_list();
#endif

}

/**
 * Remove a timer from the list of active timers.
 *
 * @param timerToRemove pointer on the timer to remove
 *
 * WARNING: timerToRemove MUST NOT be null (rem: static function )
 *
 */
static void remove_timer (T_TIMER_LIST_ELT* timerToRemove)
{
    T_TIMER_LIST_ELT* removePoint;

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
    _log ("\nINFO : remove_timer - start: removing 0x%x to expire at %d (now = %d)", (uint32_t) timerToRemove, timerToRemove->desc.expiration, _GET_TICK());
    display_list();
#endif

    if ( NULL != g_CurrentTimerHead )
    {
        removePoint = g_CurrentTimerHead;
        while ( NULL != removePoint )
        {
            if ( timerToRemove == removePoint )
            {
                if ( NULL != timerToRemove->next )
                {
                    timerToRemove->next->prev = timerToRemove->prev ;
                }
                if ( NULL != timerToRemove->prev )
                {
                    timerToRemove->prev->next = timerToRemove->next ;
                }
            }
            removePoint = removePoint->next ;
        }

        if ( timerToRemove == g_CurrentTimerHead )
        {
            g_CurrentTimerHead = g_CurrentTimerHead->next;
            if  (NULL != g_CurrentTimerHead) {
                g_CurrentTimerHead->prev = NULL;
            }
        }
    }
    else
    {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
        _log ("\nERROR : remove_timer : list of active timer is empty ");
#endif
        panic (E_OS_ERR);
    }

    /* clean-up links */
    timerToRemove->prev = NULL;
    timerToRemove->next = NULL;
    timerToRemove->desc.status = E_TIMER_READY;


#ifdef __DEBUG_OS_ABSTRACTION_TIMER
    _log ("\nINFO : remove_timer - end ");
    display_list();
#endif
}



/**
 * Execute the callback of a timer.
 *
 * @param expiredTimer pointer on the timer
 *
 * WARNING: expiredTimer MUST NOT be null (rem: static function )
 */
static void execute_callback (T_TIMER_LIST_ELT* expiredTimer)
{
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
    _log ("\nINFO : execute_callback : executing callback of timer 0x%x  (now = %u - expiration = %u)", (uint32_t) expiredTimer, _GET_TICK(), expiredTimer->desc.expiration);
#endif

    /* if the timer was not stopped by its own callback */
    if (E_TIMER_RUNNING == expiredTimer->desc.status)
    {
        /* remove the timer */
        disable_scheduling();
        remove_timer(expiredTimer);
        enable_scheduling();

        /* add it again if repeat flag was on */
        if ( expiredTimer->desc.repeat )
        {
            disable_scheduling();
            expiredTimer->desc.expiration = expiredTimer->desc.expiration + expiredTimer->desc.delay;
            add_timer(expiredTimer);
            enable_scheduling();
        }
    }

    /* call callback back */
    if (NULL != expiredTimer->desc.callback)
    {
        expiredTimer->desc.callback(expiredTimer->desc.data);
    }
    else
    {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
        _log ("\nERROR : execute_callback : timer callback is null ");
#endif
        panic (E_OS_ERR);
    }
}


/**********************************************************
 ************** Exported functions ************************
 **********************************************************/

/*----- Initialization  */

/**
 * Initialize the resources used by the framework's timer services
 *
 * IMPORTANT : this function must be called during the initialization
 *             of the OS abstraction layer.
 *             this function shall only be called once after reset, otherwise
 *             it may cause the take/lock and give/unlock services to fail
 */
void framework_init_timer(void)
{
    uint8_t idx;

#ifdef CONFIG_NANOKERNEL
    nano_sem_init ( &g_TimerSem );
    nano_timer_init (&g_NanoTimer, &g_NanoTimerData);
#else
    g_TimerSem = OS_TIMER_SEM;
#endif

    /* start with empty list of active timers: */
    g_CurrentTimerHead = NULL;

    /* memset ( g_TimerPool_elements, 0 ):  */
    for (idx = 0; idx < TIMER_POOL_SIZE; idx++)
    {
        g_TimerPool_elements[idx].desc.callback = NULL;
        g_TimerPool_elements[idx].desc.data = NULL;
        g_TimerPool_elements[idx].desc.delay = 0;
        g_TimerPool_elements[idx].desc.expiration = 0;
        g_TimerPool_elements[idx].desc.repeat = false;
        g_TimerPool_elements[idx].prev = NULL;
        g_TimerPool_elements[idx].next = NULL;
        /* hopefully, the init function is performed before
         *  timer_create and timer_stop can be called,
         *  hence there is no need for a critical section
         *  here */
    }

    /* start "timer_task" in a new fiber for NanoK, or a new task for microK */
#ifdef CONFIG_NANOKERNEL
        fiber_fiber_start ((char *) g_TimerFiberStack, TIMER_CBK_TASK_STACK_SIZE,
                timer_task,0,0,
                TIMER_CBK_TASK_PRIORITY, TIMER_CBK_TASK_OPTIONS);
#else
        task_start(OS_TASK_TIMER);
#endif
}

/**
 * Return the current tick converted in milliseconds.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @return current tick converted in milliseconds
 */
uint32_t get_time_ms(void)
{
    return ( (uint32_t) CONVERT_TICKS_TO_MS(_GET_TICK()));
}

/**
 * Return the current tick converted in microseconds.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * ZEPHYR does not provide an API to read a high-precision timer, that
 * returns a 64-bits value.
 * The tick is read as a 32-bits value, then casted to 64-bits.
 *
 * @return current tick converted in microseconds
 */
uint64_t get_time_us(void)
{
    return ( (uint64_t) CONVERT_TICKS_TO_US(_GET_TICK()));
}

/**
 * Create  a timer object.
 *     This service may panic if err parameter is null and:
 *         callback parameter is null, or
 *         no timer is available.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param callback: pointer to the function to be executed.
 * @param privData: pointer to data that shall be passed to the callback
 * @param delay: number of milliseconds between function executions
 * @param repeat: specifies if the timer shall be re-started after each execution of the callback
 * @param startup : specifies if the timer shall be start immediately
 * @param err (out): execution status:
 *         E_OS_OK : callback is programmed
 *         E_OS_ERR: no timer is available, or callback parameter is null
 *
 * @return Handler on the timer, NULL if the service fails (e.g. no available
 *         timer or callback is a null pointer).
 */
T_TIMER timer_create(T_ENTRY_POINT callback, void* privData, uint32_t delay, bool repeat, bool startup, OS_ERR_TYPE* err)
{
    T_TIMER_LIST_ELT* timer = NULL;
#ifdef CONFIG_NANOKERNEL /********************** NANO KERNEL SPECIFIC:  */
    if (delay> 0 && delay < NANO_TICK ) {
      delay= NANO_TICK;
    }
#endif
    /* check input parameters */
    if ((NULL != callback) && (OS_WAIT_FOREVER != delay))
    {
        /* delay should be set to 0 if startup flag is false
         * otherwise delay should be a positive value if startup flag is true */
        if (((0 < delay) && (true == startup)) || ((0 <= delay) && (false == startup)))
        {
            /* find and reserve a timer resource from g_TimerPool_elements */
            /* rem: timer points to an element from the global g_TimerPool_elements */
            timer = g_TimerPool_alloc();
            if (timer != NULL)
            {
                /* initialize timer descriptor */
                timer->desc.callback = callback;
                timer->desc.data = privData;
                timer->desc.delay =  CONVERT_MS_TO_TICKS ( delay );
                timer->desc.repeat = repeat;
                timer->desc.status = E_TIMER_READY;

                /* insert timer in the list of active timers */
                if (startup)
                {
                    timer->desc.expiration = _GET_TICK() + timer->desc.delay;
                    disable_scheduling();
                    add_timer(timer);
                    if ( g_CurrentTimerHead == timer )
                    {
                        /* new timer is the next to expire, unblock timer_task to assess the change */
                        signal_timer_task();
                    }
                    enable_scheduling();
                }

    #ifdef __DEBUG_OS_ABSTRACTION_TIMER
                _log ("\nINFO : timer_create : new timer will expire at %u (now = %u ) - addr = 0x%x",timer->desc.expiration,_GET_TICK(), (uint32) timer);
    #endif
                error_management (err, E_OS_OK);
            }
            else
            {
                /* all timers from the pool are already being used */
                error_management (err, E_OS_ERR_NO_MEMORY);
            }
        }
        else
        { /* delay and startup parameter are inconsistent */
            error_management (err, E_OS_ERR);
        }
    }
    else
    { /* callback == NULL or delay == 0 : at least one parameter is invalid */
        error_management (err, E_OS_ERR);
    }
    return ((T_TIMER) timer);
}

/**
 * start the timer.
 *     This service may panic if err parameter is null and:
 *         no timer is available or timer is running .
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param tmr : handler on the timer (value returned by timer_create ).
 * @param err (out): execution status:
 */
void timer_start(T_TIMER tmr, uint32_t delay, OS_ERR_TYPE* err)
{
    T_TIMER_LIST_ELT* timer = (T_TIMER_LIST_ELT*) tmr;
    OS_ERR_TYPE localErr = E_OS_OK;

    /* if timer is created */
    if (NULL != timer)
    {
        if(timer->desc.status == E_TIMER_READY)
        {
            /* if timer parameter are valid */
            if ((NULL != timer->desc.callback) && (0 < delay))
            {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
                _log ("\nINFO : timer_start : starting  timer ");
#endif
                /* Update expiration time */
                timer->desc.delay = CONVERT_MS_TO_TICKS(delay);
                timer->desc.expiration = _GET_TICK() + timer->desc.delay;
                disable_scheduling();
                /* add the timer */
                add_timer(timer);

                /* new timer is the next to expire, unblock timer_task to assess the change */
                if (g_CurrentTimerHead == timer) {
                     signal_timer_task();
                }
                enable_scheduling();
            }
            else
            {
                 /* timer is not valid */
                 localErr = E_OS_ERR;
            }
        }
        else if(timer->desc.status == E_TIMER_RUNNING)
        {
            localErr = E_OS_ERR_BUSY;
        }
    }
    else
    { /* tmr is not a timer from g_TimerPool_elements */
        localErr = E_OS_ERR;
    }

    error_management(err, localErr);

}

/**
 *  Remove the timer in Chained list of timers.
 *     This service may panic if err parameter is null and:
 *         tmr parameter is is null, invalid, or timer is not running.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param tmr : handler on the timer (value returned by timer_create ).
 * @param err (out): execution status:
 */
void timer_stop(T_TIMER tmr, OS_ERR_TYPE* err)
{
    T_TIMER_LIST_ELT* timer = (T_TIMER_LIST_ELT*)tmr ;
    OS_ERR_TYPE localErr = E_OS_OK;
    bool doSignal = false;

    if ( NULL != timer )
    {
        /* if timer is active */
        if (timer->desc.status == E_TIMER_RUNNING)
        {
#ifdef __DEBUG_OS_ABSTRACTION_TIMER
            _log ("\nINFO : timer_stop : stopping timer at addr = 0x%x", (uint32_t) timer);
#endif
            /* remove the timer */
            disable_scheduling();

            if ( g_CurrentTimerHead == timer )
            {
                doSignal = true ;
            }

            remove_timer(timer);

            if ( doSignal )
            {
                /* the next timer to expire was removed, unblock timer_task to assess the change */
                signal_timer_task();
            }

            enable_scheduling();

        }
        else
        { /* tmr is not running */
            localErr = E_OS_OK;
        }
    }
    else
    { /* tmr is not a timer from g_TimerPool_elements */
        localErr = E_OS_ERR;
    }
    error_management(err, localErr);

}

/**
 * Delete the timer object.
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * @param tmr : handler on the timer (value returned by timer_create ).
 * @param err (out): execution status:
 *         E_OS_OK : timer is stopped and callback is disabled
 *         E_OS_ERR: tmr parameter is null, invalid, or timer running
 */
void timer_delete(T_TIMER tmr, OS_ERR_TYPE* err)
{
    T_TIMER_LIST_ELT* timer = (T_TIMER_LIST_ELT*) tmr;
    OS_ERR_TYPE localErr = E_OS_OK;

    if (NULL != timer)
    {
        /* check if timer is running and stop it */
        if (timer->desc.status == E_TIMER_RUNNING)
        {
            timer_stop(timer, err);
        }

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
        _log ("\nINFO : timer_delete : deleting  timer at addr = 0x%x", (uint32) timer);
#endif

        /*  free the timer   */
        g_TimerPool_free(timer);
    }
    else
    { /* tmr is not a timer from g_TimerPool_elements */
        localErr = E_OS_ERR;
    }


    error_management (err, localErr);

}


/**
 * Main function of the timer task. This function is in charge of
 *    calling the callbacks associated with the timers.
 *
 *    This function is run in a high priority task on microkernel builds.
 *    This function is run in a high priority fiber on nanokernel builds.
 *    It implements an infinite loop that waits for any of the semaphores from
 *    g_TimerSem.
 *    When a semaphore is signaled, this function fetches the pointer to the
 *    associated callback in g_TimerDesc, then calls it.
 *
 * @param dummy1 not used (required by ZEPHYR API)
 * @param dummy2 not used (required by ZEPHYR API) *
 *
 */
void timer_task(int dummy1, int dummy2)
{
    uint32_t timeout;
    uint32_t now;

    UNUSED(dummy1);
    UNUSED(dummy2);

    timeout = OS_WAIT_FOREVER;

    while (1) /* the Timer task shall never stop */
    {
#ifdef CONFIG_MICROKERNEL /********************** MICRO KERNEL SPECIFIC:  */
        /* block until g_TimerSem is signaled or until the next timeout expires */
        (void) task_sem_take_wait_timeout(g_TimerSem, timeout);
#else
        if (NULL == g_CurrentTimerHead)
        {
            /* Start a background timer with max positive delay */
            nano_fiber_timer_start (&g_NanoTimer, 0x7FFFFFFF);
            /* wait until the next timer expires or one is added or removed */
            nano_fiber_timer_wait (&g_NanoTimer);
        }
#endif
        now = _GET_TICK();
        /* task is unblocked: check for expired timers */
        while (is_after_expiration (now, &(g_CurrentTimerHead->desc)))
        {
            execute_callback(g_CurrentTimerHead);
        }
        /* Compute timeout until the expiration of the next timer */
        if ( NULL != g_CurrentTimerHead )
        {
            now = _GET_TICK();
            /* In micro kernel context, timeout = 0 or timeout < 0 works.
             * In nano kernel context timeout must be a positive value.
            */
#ifdef CONFIG_NANOKERNEL
            if (g_CurrentTimerHead->desc.expiration > now)
            {
#endif
                timeout = g_CurrentTimerHead->desc.expiration - now;
                if (OS_WAIT_FOREVER == timeout)
                { /* cannot have timeout = OS_WAIT_FOREVER while there is
                    still at least one active timer */
                    timeout++;
                }
#ifdef CONFIG_NANOKERNEL
                nano_fiber_timer_start (&g_NanoTimer, timeout);
                /* wait until the next timer expires or one is added or removed */
                nano_fiber_timer_wait (&g_NanoTimer);
                /* nano_fiber_timer_wait will wait until "natural" timer
                * expiration, or until nano_fiber_timer_stop(&g_NanoTimer)
                * is called by the timer_callback or by timer_stop() */
            }
#endif
        }
#ifdef CONFIG_MICROKERNEL
        else
        {
            timeout = OS_WAIT_FOREVER;
        }
#endif

#ifdef __DEBUG_OS_ABSTRACTION_TIMER
        if (NULL != g_CurrentTimerHead )
            _log ("\nINFO : timer_task : now = %u, next timer expires at %u, timeout = %u", _GET_TICK() , g_CurrentTimerHead->desc.expiration, timeout );
        else
            _log ("\nINFO : timer_task : now = %u, no next timer, timeout = OS_WAIT_FOREVER", _GET_TICK() );
#endif

    } /* end while(1) */
}

/** @} */
