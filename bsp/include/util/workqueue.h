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

#ifndef __UTIL_WORK_QUEUE_H__
#define __UTIL_WORK_QUEUE_H__

#include "os/os.h"

/**
 * @defgroup util Utilities
 * Convenience utilities modules.
 */

/**
 * @defgroup workqueue Work Queues
 *
 * Work Queues implementation.
 *
 * Workqueues are used to defer work in a fiber / task context.
 * Generally, an interrupt handler will use queue_work() or
 * queue_work_on() in order to execute a callback function in a
 * non-interrupt context.
 *
 * This requires an execution context to be setup. This execution
 * context should call workqueue_init() and either workqueue_run() if
 * the context is fully dedicated to workqueue, or workqueue_poll() if the
 * context is shared between workqueue execution and other things (typically
 * bootloader or other bare metal context only have one execution context and
 * it should be shared between several components.)
 *
 * @ingroup util
 * @{
 */

struct work_queue {
	T_QUEUE queue;
	int state;
};

#ifndef _UTIL_WORKQUEUE_C_
/**
 * Default workqueue used by workqueue_queue_work()
 *
 * This workqueue needs to be initialized with a
 * workqueue_init(queue, &default_work_queue); statement.
 */
extern struct work_queue default_work_queue;
#endif

#define WORKQUEUE_UNINITIALIZED      0
#define WORKQUEUE_INITIALIZED        1
#define WORKQUEUE_RUNNING            2

/**
 * \brief initialize a workqueue
 *
 * Initialize the default workqueue if specified workqueue is NULL.
 *
 * \param queue the queue associated with the workqueue
 * \param wq the workqueue to initialize.
 */
void workqueue_init(T_QUEUE queue, struct work_queue * wq);

/**
 * \brief run a workqueue.
 *
 * This function runs the workqueue, it should be called in the running
 * context of the workqueue.
 * This function will continuouly call workqueue_poll() with wait = true
 * and never returns.
 *
 * \param wq the workqueue structure to run
 *
 */
void workqueue_run(struct work_queue * wq);

/**
 * \brief poll workqueue
 *
 * This function will poll for work on the work queue. It will pop one work
 * and execute it and return.
 * If there is no work pending in the queue and wait == true, then it will
 * block until a work item is queued.
 *
 * \param wq the workqueue structure to poll
 * \param wait true if poll should block for the next work if queue is empty
 *             false if poll should just return if queue is empty.
 *
 * \return true if there is more work pending
 */
bool workqueue_poll(struct work_queue * wq, bool wait);

/**
 * \brief post work to be run in the specified workqueue.
 *
 * This function will post a request in the specified workqueue.
 * The callback function will be called in the running context of
 * the specified workqueue.
 *
 * \param wq the workqueue on which to queue the work on.
 * \param cb the callback to execute
 * \param cb_data the data passed to the callback
 *
 * \return E_OS_OK if work properly queued
 */
OS_ERR_TYPE workqueue_queue_work_on(struct work_queue * wq,
				    void (*cb)(void * data), void * cb_data);

/**
 * \brief post work to the default workqueue
 *
 * This function will post a request in the default system workqueue.
 *
 * \param cb the callback to execute
 * \param cb_data the data passed to the callback
 *
 * \return E_OS_OK if work properly queued
 */
OS_ERR_TYPE workqueue_queue_work(void (*cb)(void * data), void * cb_data);

/** @} */

#endif /* __INFRA_UTIL_WORK_QUEUE_H__ */
