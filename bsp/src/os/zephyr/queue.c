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

 /*
 * ZEPHYR QUEUE OS abstraction for Mciro & Nano Kernel, implemented with our own queue
 * We do not use VxMicro queues because send_msg_to_queue cannot be use from an ISR.
 *
 * Functions are exported by os.h
 *
 */




/******* Framework headers : */
#include "os/os.h"             /* framework export definitions */
#include "os/os_types.h"       /* framework-specific types */
#include "zephyr/os_specific.h" /* zephyr-specific definitions */
#include "zephyr/os_config.h"   /* zephyr- resource configuration */
#include "zephyr/common.h"



/******* ZEPHYR headers : */
/*
#include "res.h"
#include "allnodes.h"
#include "k_struct.h"
#include "allnodes.h"
#include "minik.h"
*/

#define MTX_POOL_GENERIC 0x00010021

#include "util/list.h"

/**********************************************************
 ************** Local definitions  ************************
 **********************************************************/




/* Warning 'next' must be the first element of the struct */
typedef struct                     // element for linked-list
{
    list_t * next;               //the next element in the list
  void * data;                      //generic pointer to any data type
}list_element;

typedef struct                    // a linked-list of list_element
{
  list_head_t _list;
  uint32_t current_size;
  uint32_t max_size;
  T_SEMAPHORE sema;         /* semaphore used by the listener to wait on new incoming data
                              and used by the producer to signal new incoming data in the queue */
}queue_impl_t;


// equivalent to : queue_impl_t list_elements[QUEUE_POOL_SIZE];
DECLARE_BLK_ALLOC(queue, queue_impl_t, QUEUE_POOL_SIZE) /* see common.h */

// equivalent to : list_element element[QUEUE_ELEMENT_POOL_SIZE];
DECLARE_BLK_ALLOC(element,  list_element, QUEUE_ELEMENT_POOL_SIZE) /* see common.h */







/**********************************************************
 ************** Private variables  ************************
 **********************************************************/
#if ((defined(CONFIG_NANOKERNEL)) && (!defined(VXMICRO_OS_ABSTRACTION_USE_SINGLE_POOL_LOCK)))
static struct nano_sem QueuePoolLockSem;
#endif


/**********************************************************
 ************** Forward declarations **********************
 **********************************************************/
void lock_pool(void);
void unlock_pool(void);
static OS_ERR_TYPE add_data(queue_impl_t * queue, void * data);                   // Append data to the queue
static OS_ERR_TYPE remove_data(queue_impl_t *queue, void **data);                 // Remove data to the queue


/**
 * Initialize the resources used by the framework's sync services.
 *
 * IMPORTANT : this function must be called during the initialization
 *             of the OS abstraction layer.
 *             this function shall only be called once after reset, otherwise
 *             it may cause the queue services to fail
 */
void framework_init_queue (void)
{
#if defined (CONFIG_NANOKERNEL) && ! defined (VXMICRO_OS_ABSTRACTION_USE_SINGLE_POOL_LOCK)
    nano_sem_init (&QueuePoolLockSem);
    nano_task_sem_give (&QueuePoolLockSem);
#endif

    ;   /* Nothing to do */
}





/**
 * Delete a queue.
 *
 * Free a whole queue.
 * @param queue - The queue to free.
 */
void queue_delete(T_QUEUE queue, OS_ERR_TYPE* err)
{
    OS_ERR_TYPE _err;
    queue_impl_t * q = (queue_impl_t*) queue;
    T_EXEC_LEVEL execLvl = _getExecLevel();
    uint32_t data;
    T_QUEUE_MESSAGE p_msg = &data;

    /* check execution level */
    if ((E_EXEC_LVL_FIBER == execLvl) || (E_EXEC_LVL_TASK == execLvl))
    {
        lock_pool();
        if (queue_used(q) && q->sema != NULL)
        {

            /* first empty the queue before to delete it to free all the elements */
            do{
                queue_get_message(q, &p_msg, OS_NO_WAIT, &_err);
            }while(_err == E_OS_OK);

            if(_err == E_OS_ERR_EMPTY)
            {
                semaphore_delete(q->sema, &_err);
                q->sema = NULL;
                if( _err == E_OS_OK)
                {
                    queue_free(q);
                    error_management (err, E_OS_OK);
                }
                else
                {
                    panic(E_OS_ERR_UNKNOWN);    // Panic because we should never reach this point.
                }
            }
            else
            {
                error_management (err, E_OS_ERR);
            }
        }
        else
        {
            error_management (err, E_OS_ERR);
        }
        unlock_pool();
    }
    else
    {
        error_management (err, E_OS_ERR_NOT_ALLOWED);
    }
    return;
}


/**
 * Create a message queue.
 *
 *     Create a message queue.
 *     This service may panic if err parameter is NULL and:
 *     -# no queue is available, or
 *     -# when called from an ISR.
 *
 *     Authorized execution levels:  task, fiber.
 *
 *     As for semaphores and mutexes, queues are picked from a pool of
 *     statically-allocated objects.
 *
 * @param maxSize: maximum number of  messages in the queue.
 *     (Rationale: queues only contain pointer to messages)
 *
 * @param err (out): execution status:
 *          -# E_OS_OK : queue was created
 *          -# E_OS_ERR: all queues from the pool are already being used
 *          -# E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 *
 * @return Handler on the created queue.
 *     NULL if all allocated queues are already being used.
 */
T_QUEUE queue_create(uint32_t max_size, OS_ERR_TYPE* err)
{
    queue_impl_t * q = NULL;
    T_EXEC_LEVEL execLvl = _getExecLevel();
    OS_ERR_TYPE _err;


    if(max_size==0 || max_size>QUEUE_ELEMENT_POOL_SIZE)
    {
        error_management (err, E_OS_ERR);
        return NULL;
    }

    /* check execution level */
    if ((E_EXEC_LVL_FIBER == execLvl) || (E_EXEC_LVL_TASK == execLvl))
    {
        /* Block concurrent accesses to the pool of queue_list */
        lock_pool();
        q = queue_alloc();
        unlock_pool();

        if (q != NULL)
        {
           list_init(&q->_list);  // replace the following commented code
           q->current_size = 0;
           q->max_size = max_size;
           q->sema = semaphore_create(0, &_err);
           error_management (err, _err);
        }
        else
        {
            error_management (err, E_OS_ERR);
        }
    }
    else
    {
        error_management (err, E_OS_ERR_NOT_ALLOWED);
    }

    return (T_QUEUE)q;
}




/**
 * Read a message from a queue.
 *
 *     Read and dequeue a message.
 *     This service may panic if err parameter is NULL and:
 *      -# queue parameter is invalid, or
 *      -# message parameter is NULL, or
 *      -# when called from an ISR.
 *
 *     Authorized execution levels:  task, fiber.
 *
 * @param queue : handler on the queue (value returned by queue_create).
 *
 * @param message (out): pointer to read message.
 *
 * @param timeout: maximum number of milliseconds to wait for the message. Special
 *                values OS_NO_WAIT and OS_WAIT_FOREVER may be used.
 *
 * @param err (out): execution status:
 *          -# E_OS_OK : a message was read
 *          -# E_OS_ERR_TIMEOUT: no message was received
 *          -# E_OS_ERR_EMPTY: the queue is empty
 *          -# E_OS_ERR: invalid parameter
 *          -# E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
void queue_get_message (T_QUEUE queue, T_QUEUE_MESSAGE* message, int timeout, OS_ERR_TYPE* err)
{
    OS_ERR_TYPE _err;
    T_EXEC_LEVEL execLvl = _getExecLevel();
    queue_impl_t * q = (queue_impl_t*) queue;

    /* check input parameters */
    if( (message == NULL) || (!queue_used(q)) || (q->sema==NULL) || (timeout<OS_WAIT_FOREVER) )
    {
        error_management (err, E_OS_ERR);
        return;
    }

    /* check execution level */
    if ((E_EXEC_LVL_FIBER == execLvl) || (E_EXEC_LVL_TASK == execLvl))
    {
        _err = semaphore_take(q->sema, timeout);
        switch(_err){

        case E_OS_OK:
        {
            uint32_t it_mask = interrupt_lock();
            _err = remove_data(q, message);
            interrupt_unlock(it_mask);
            error_management (err, E_OS_OK);
        }
            break;
        case E_OS_ERR_TIMEOUT:
            error_management (err, E_OS_ERR_TIMEOUT);
            break;
        case E_OS_ERR_BUSY:
            if(err){
                //QUEUE EMPTY, is a common use case, do not panic even if err == NULL
                *err = E_OS_ERR_EMPTY;
            }
            break;
        default:
            //unknown error
            panic(E_OS_ERR_UNKNOWN);
        }
    }
    else
    {
        error_management (err, E_OS_ERR_NOT_ALLOWED);
    }

    return;
}


/**
 * Send a message on a queue.
 *
 *     Send / queue a message.
 *     This service may panic if err parameter is NULL and:
 *      -# queue parameter is invalid, or
 *      -# the queue is already full, or
 *
 *     Authorized execution levels:  task, fiber, ISR.
 *
 * @param queue: handler on the queue (value returned by queue_create).
 *
 * @param message (in): pointer to the message to send.
 *
 * @param err (out): execution status:
 *          -# E_OS_OK : a message was read
 *          -# E_OS_ERR_OVERFLOW: the queue is full (message was not posted)
 *          -# E_OS_ERR: invalid parameter
 */
void queue_send_message(T_QUEUE queue, T_QUEUE_MESSAGE message, OS_ERR_TYPE* err )
{
    OS_ERR_TYPE _err;
    queue_impl_t * q = (queue_impl_t*) queue;

    /* check input parameters */
    if( queue_used(q) && q->sema != NULL )
    {
        uint32_t it_mask = interrupt_lock();
        _err = add_data(q, message);
        interrupt_unlock(it_mask);

        if(_err == E_OS_OK)
        {
            semaphore_give(q->sema, &_err);  // signal new message in the queue to the listener.
            error_management (err, E_OS_OK);
        }
        else
        {
            error_management (err, _err);
        }
    }
    else
    {   /* param invalid */
        error_management (err, E_OS_ERR);
    }
    return;
}


///////////////////////////// LOCAL FUNCTIONS /////////////////////////////


void lock_pool(void)
{
#ifdef VXMICRO_OS_ABSTRACTION_USE_SINGLE_POOL_LOCK

#ifdef CONFIG_NANOKERNEL
    _PoolLock();
#else
    task_mutex_lock_wait(MTX_POOL_GENERIC);
#endif

#else

#ifdef CONFIG_NANOKERNEL
    nano_fiber_sem_take_wait (&QueuePoolLockSem) ;
#else
    task_mutex_lock_wait(MTX_POOL_QUEUE);
#endif
#endif
}

void unlock_pool(void)
{
#ifdef VXMICRO_OS_ABSTRACTION_USE_SINGLE_POOL_LOCK

#ifdef CONFIG_NANOKERNEL
    _PoolUnlock();
#else
    task_mutex_unlock(MTX_POOL_GENERIC);
#endif

#else

#ifdef CONFIG_NANOKERNEL
    nano_fiber_sem_give (&QueuePoolLockSem) ;
#else
    task_mutex_unlock(MTX_POOL_QUEUE);
#endif
#endif
}


static OS_ERR_TYPE add_data(queue_impl_t * list, void * data)
{
    OS_ERR_TYPE err = E_OS_ERR_OVERFLOW;
    /* if linked-list not full */
    if(list->current_size < list->max_size)
    {
        list_element * element = element_alloc();
        if(element)
        {
            element->data = data;

            list_add(&(list->_list), (list_t*)element);
            list->current_size++;
            err = E_OS_OK;
        }
        else
        {
            panic(E_OS_ERR_NO_MEMORY); /* Panic if no memory available */
        }
    }
    else
    {
        panic(E_OS_ERR_OVERFLOW); /* Panic if max size reached */
    }
    return err;
}


static OS_ERR_TYPE remove_data(queue_impl_t *list, void **data)
{

    list_element* element = (list_element*)list_get(&(list->_list));
    if(element == NULL)
    {
        return E_OS_ERR_EMPTY;
    }
    else
    {
        *data = element->data;
        element_free(element);
        list->current_size--;
    }
    return E_OS_OK;
}

/** @} */
