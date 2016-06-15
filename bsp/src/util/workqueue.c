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

#include "infra/log.h"
#define _UTIL_WORKQUEUE_C_
#include "util/workqueue.h"

struct work_queue default_work_queue = { NULL, WORKQUEUE_UNINITIALIZED };

struct wq_elem {
	void (*cb) (void *cb_data);
	void *cb_data;
};

void workqueue_init(T_QUEUE queue, struct work_queue *wq)
{
	if (wq == NULL) {
		pr_error(LOG_MODULE_UTIL,
			 "wq should not be null");
		return;
	}
	if (wq->state != WORKQUEUE_UNINITIALIZED) {
		pr_error(LOG_MODULE_UTIL,
			 "Attempt to reinit workqueue");
	}
	wq->queue = queue;
	wq->state = WORKQUEUE_INITIALIZED;
}

void workqueue_run(struct work_queue *wq)
{
	if (wq == NULL) {
		pr_error(LOG_MODULE_UTIL,
			 "wq should not be null");
		return;
	}

	wq->state = WORKQUEUE_RUNNING;
	while (1) {
		workqueue_poll(wq, true);
	}
}

bool workqueue_poll(struct work_queue *wq, bool wait)
{
	T_QUEUE_MESSAGE m;
	OS_ERR_TYPE err;
	if (wq == NULL) {
		pr_error(LOG_MODULE_UTIL,
			 "wq should not be null");
		return 0;
	}
	if (wq->queue == NULL || wq->state == WORKQUEUE_UNINITIALIZED) {
		pr_error(LOG_MODULE_UTIL,
			 "Attempt to poll on uninitialized workqueue");
		return 0;
	}
	queue_get_message(wq->queue, &m,
			  wait ? OS_WAIT_FOREVER : OS_NO_WAIT, &err);
	if (m != NULL) {
		struct wq_elem *w = (struct wq_elem *)m;
		if (w->cb != NULL) {
			w->cb(w->cb_data);
		}
		bfree(w);
	}
	return !(err == E_OS_ERR_EMPTY);
}

OS_ERR_TYPE workqueue_queue_work_on(struct work_queue * wq,
				    void (*cb) (void *data), void *cb_data)
{
	OS_ERR_TYPE err = E_OS_OK;
	if (wq == NULL || wq->queue == NULL) {
		return E_OS_ERR;
	}
	struct wq_elem *w = (struct wq_elem *)balloc(sizeof(*w), &err);
	if (err == E_OS_OK) {
		w->cb = cb;
		w->cb_data = cb_data;
		queue_send_message(wq->queue, w, &err);
		if (err != E_OS_OK) {
			pr_error(LOG_MODULE_UTIL, "could not queue work");
			bfree(w);
		}
	}
	return err;
}

OS_ERR_TYPE workqueue_queue_work(void (*cb) (void *data), void *cb_data)
{
	return workqueue_queue_work_on(&default_work_queue, cb, cb_data);
}
