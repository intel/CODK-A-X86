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

#include "cfw/cfw.h"
#include "cfw/cfw_client.h"
#include "cfw/cfw_messages.h"
#include "cfw/cproxy.h"

/* Private types and variables */

struct _svc_cnx {
	struct _svc_cnx *next;
	void (*cb) (struct cfw_message*, void*);
	void *data;
	svc_client_handle_t *sh;
	uint16_t src_port;
	uint16_t service_id;
};

struct _open_service_ctx {
	int service_id;
};

static T_QUEUE _queue;
static cfw_handle_t _proxy_handle;
static list_head_t _services;

bool _cmp_port(list_t *item, void *data)
{
	struct _svc_cnx* cnx = (struct _svc_cnx*)item;
	return (cnx->src_port == *(uint16_t*)data);
}

bool _cmp_handle(list_t *item, void *data)
{
	struct _svc_cnx* cnx = (struct _svc_cnx*)item;
	return (cnx->sh == (svc_client_handle_t *)data);
}

static void _handle_client_message(struct cfw_message *msg, void *data)
{
	(void)data;
	switch (CFW_MESSAGE_ID(msg)) {
		case MSG_ID_CFW_OPEN_SERVICE:
		{
			/* We have passed the allocated cnx as an opaque data */
			struct _svc_cnx *cnx = CFW_MESSAGE_PRIV(msg);
			/* Get the service parameters from the message and store them locally */
			cfw_open_conn_rsp_msg_t *con_msg = (cfw_open_conn_rsp_msg_t *)msg;
			cnx->sh = (svc_client_handle_t *)(con_msg->client_handle);
			cnx->src_port = con_msg->port;
			cfw_msg_free(msg);
			list_add(&_services, (list_t *)cnx);
			break;
		}
		case MSG_ID_CFW_CLOSE_SERVICE:
		{
			struct _svc_cnx *cnx = CFW_MESSAGE_PRIV(msg);
			list_remove(&_services, (list_t *)cnx);
			bfree(cnx);
			cfw_msg_free(msg);
			break;
		}
		case MSG_ID_CFW_SVC_AVAIL_EVT:
		{
			struct _svc_cnx *cnx = CFW_MESSAGE_PRIV(msg);
			if (((cfw_svc_available_evt_msg_t*)msg)->service_id == cnx->service_id) {
				cfw_open_service(_proxy_handle, cnx->service_id, cnx);
			}
			cfw_msg_free(msg);
			break;
		}
		default:
		{
			/* Find the service connection based on the message source port */
			struct _svc_cnx *cnx = (struct _svc_cnx*)
			list_find_first(&_services, _cmp_port, &(CFW_MESSAGE_SRC(msg)));
			if (cnx) {
				cnx->cb(msg, cnx->data);
			} else {
				cfw_msg_free(msg);
			}
			break;
		}
	}
}

/* Public API */

void cproxy_disconnect(svc_client_handle_t *h)
{
	struct _svc_cnx *cnx = (struct _svc_cnx*)
			list_find_first(&_services, _cmp_handle, h);
	if (cnx) {
		cfw_close_service(h, cnx);
	}
}

svc_client_handle_t * cproxy_connect(int service_id,
		void(*cb)(struct cfw_message *, void*), void *data)
{
	svc_client_handle_t *sh = NULL;
	/* Allocate a service handle structure */
	struct _svc_cnx *cnx = balloc(sizeof(*cnx), NULL);
	cnx->cb = cb;
	cnx->data = data;
	cnx->sh = NULL;
	cnx->src_port = 0;
	cnx->service_id = service_id;
	cfw_register_svc_available(_proxy_handle, cnx->service_id, cnx);
	uint32_t start = get_time_ms();
	while (!(sh = cnx->sh) && (get_time_ms() < (start + 100))) {
		queue_process_message(_queue);
	}
	return sh;
}

void cproxy_init(T_QUEUE queue)
{
	_queue = queue;
	_proxy_handle = cfw_init(_queue, _handle_client_message, NULL);
}
