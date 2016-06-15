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

#include "util/list.h"
#include "os/os.h"
#include "infra/message.h"
#include "infra/port.h"
#include "infra/log.h"

#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "cfw/cfw_service.h"
#include "cfw/cfw_client.h"
#include "cfw_private.h"

/**
 * \file service_api.c implementation of the service_manager API
 */

/* Structure for holding if a service is local or remote. */

#define SEND_MESSAGE(_msg_) cfw_send_message(&(_msg_)->header)

void cfw_port_set_handler(uint16_t port_id, void (*handler)(
				  struct cfw_message *, void *), void *param)
{
#ifdef SVC_API_DEBUG
	pr_debug(LOG_MODULE_CFW, "%s: port: %p h: %p", __func__, port, handler);
#endif
	port_set_handler(port_id, (void (*)(struct message *,
					    void *))handler, param);
}

struct cfw_message *cfw_clone_message(struct cfw_message *msg)
{
	struct cfw_message *ret = (struct cfw_message *)
				  message_alloc(CFW_MESSAGE_LEN(msg), NULL);

	if (ret == NULL)
		pr_error(LOG_MODULE_CFW, "%s: Error allocating message",
			 __func__);
	else
		memcpy(ret, msg, CFW_MESSAGE_LEN(msg));
	return ret;
}

int cfw_register_service(T_QUEUE queue, service_t *svc,
			 handle_msg_cb_t handle_message, void *data)
{
	uint16_t port_id = port_alloc(queue);

	cfw_port_set_handler(port_id, handle_message, data);
	svc->port_id = port_id;
	return _cfw_register_service(svc);
}

int cfw_deregister_service(cfw_handle_t handle, service_t *svc)
{
	return _cfw_deregister_service(handle, svc);
}

struct cfw_rsp_message *cfw_alloc_rsp_msg(const struct cfw_message *	req,
					  int				msg_id,
					  int				size)
{
	struct cfw_rsp_message *rsp =
		(struct cfw_rsp_message *)cfw_alloc_message(size, NULL);

	CFW_MESSAGE_TYPE(&rsp->header) = TYPE_RSP;
	CFW_MESSAGE_ID(&rsp->header) = msg_id;
	CFW_MESSAGE_LEN(&rsp->header) = size;
	CFW_MESSAGE_DST(&rsp->header) = CFW_MESSAGE_SRC(req);
	CFW_MESSAGE_SRC(&rsp->header) = CFW_MESSAGE_DST(req);
	rsp->header.priv = req->priv;
	/* Substitute server-side with client-side conn */
	if (req->conn != NULL)
		rsp->header.conn = ((conn_handle_t *)req->conn)->client_handle;
	else
		rsp->header.conn = NULL;
	return rsp;
}

struct cfw_message *cfw_alloc_evt_msg(service_t *svc, int msg_id, int size)
{
	struct cfw_message *evt = (struct cfw_message *)cfw_alloc_message(size,
									  NULL);

	CFW_MESSAGE_TYPE(evt) = TYPE_EVT;
	CFW_MESSAGE_ID(evt) = msg_id;
	CFW_MESSAGE_LEN(evt) = size;
	CFW_MESSAGE_SRC(evt) = svc->port_id;
	/* 3 fields below whould be filed by send_event method*/
	CFW_MESSAGE_DST(evt) = 0;
	evt->priv = NULL;
	evt->conn = NULL;
	return evt;
}

struct cfw_message *cfw_alloc_internal_msg(int msg_id, int size, void *priv)
{
	struct cfw_message *evt = (struct cfw_message *)cfw_alloc_message(size,
									  NULL);

	CFW_MESSAGE_TYPE(evt) = TYPE_INT;
	CFW_MESSAGE_ID(evt) = msg_id;
	CFW_MESSAGE_LEN(evt) = size;
	CFW_MESSAGE_SRC(evt) = 0;
	/* 3 fields below whould be filed by send_event method*/
	CFW_MESSAGE_DST(evt) = 0;
	evt->priv = priv;
	evt->conn = NULL;
	return evt;
}

void default_msg_handler(struct cfw_message *msg, void *data)
{
	pr_error(LOG_MODULE_CFW, "Bug: %s should not be called data: %p",
		 __func__,
		 data);
	cfw_dump_message(msg);
}

struct conn_helper_data {
	list_t			list;
	uint16_t		service_id;
	svc_client_handle_t *	service_handle;
	int			event_count;
	void			(*cb)(svc_client_handle_t *handle, void *data);
	void *			cb_data;
	int			events[];
};

void cfw_open_service_helper(cfw_handle_t *handle, uint16_t service_id,
			     void (*cb)(svc_client_handle_t *,
					void *), void *cb_data)
{
	cfw_open_service_helper_evt(handle, service_id, NULL, 0, cb, cb_data);
}

void cfw_open_service_helper_evt(cfw_handle_t *handle, uint16_t service_id,
				 int *events, int event_count,
				 void (*cb)(svc_client_handle_t *,
					    void *), void *cb_data)
{
	_cfw_handle_t *h = (_cfw_handle_t *)handle;
	struct conn_helper_data *data = (struct conn_helper_data *)
					balloc(
		sizeof(struct conn_helper_data) + event_count * sizeof(int),
		NULL);

	data->service_id = service_id;
	data->event_count = event_count;
	data->cb = cb;
	data->cb_data = cb_data;
	memcpy(data->events, events, event_count * sizeof(int));
	list_add(&h->helper_list, &data->list);
	cfw_register_svc_available(handle, service_id, data);
}

static bool conn_helper_data_check(list_t *item, void *helper)
{
	return (struct conn_helper_data *)item ==
	       (struct conn_helper_data *)helper;
}

static struct conn_helper_data *get_conn_helper(
	_cfw_handle_t *h,
	struct conn_helper_data *
	helper)
{
	struct conn_helper_data *ret = NULL;
	list_head_t *helper_list = &h->helper_list;
	int flags = interrupt_lock();

	ret = (struct conn_helper_data *)
	      list_find_first(helper_list, conn_helper_data_check, helper);
	interrupt_unlock(flags);
	return ret;
}

static void cfw_open_service_helper_done(_cfw_handle_t *		h,
					 struct conn_helper_data *	helper)
{
	helper->cb(helper->service_handle, helper->cb_data);
	list_remove(&h->helper_list, &helper->list);
	bfree(helper);
}

void client_handle_message(struct cfw_message *msg, void *param)
{
	_cfw_handle_t *h = (_cfw_handle_t *)param;
	bool handled = false;

	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_CFW_REGISTER_SVC_AVAIL:
		if (get_conn_helper(h, msg->priv))
			handled = true;
		break;

	case MSG_ID_CFW_SVC_AVAIL_EVT:
	{
		cfw_svc_available_evt_msg_t *evt =
			(cfw_svc_available_evt_msg_t *)msg;
		if (get_conn_helper(h, evt->header.priv)) {
			cfw_open_service(h, evt->service_id, evt->header.priv);
			handled = true;
		}
	}
	break;

	case MSG_ID_CFW_OPEN_SERVICE:
	{
		cfw_open_conn_rsp_msg_t *cnf = (cfw_open_conn_rsp_msg_t *)msg;
		/** Make client handle point to server handle */
		((svc_client_handle_t *)cnf->client_handle)->server_handle =
			cnf->svc_server_handle;
		/** Initialize service port. */
		((svc_client_handle_t *)cnf->client_handle)->port = cnf->port;
#ifndef CONFIG_PORT_IS_MASTER
		/* Set local port and cpu id */
		if (get_cpu_id() != cnf->cpu_id) {
			port_set_port_id(cnf->port);
			port_set_cpu_id(cnf->port, cnf->cpu_id);
		}
#endif
		struct conn_helper_data *helper = get_conn_helper(
			h, cnf->rsp_header.header.priv);
		if (helper != NULL) {
			helper->service_handle =
				(svc_client_handle_t *)cnf->client_handle;
			if (helper->event_count > 0) {
				cfw_register_events(helper->service_handle,
						    helper->events,
						    helper->event_count,
						    helper);
			} else {
				/* call the helper callback and free it */
				cfw_open_service_helper_done(h, helper);
			}
			handled = true;
		}

		break;
	}
	case MSG_ID_CFW_REGISTER_EVT:
	{
		struct conn_helper_data *helper = get_conn_helper(
			h, CFW_MESSAGE_PRIV(msg));
		if (helper != NULL) {
			helper->cb(helper->service_handle, helper->cb_data);
			list_remove(&h->helper_list, &helper->list);
			bfree(helper);
			handled = true;
		}
	}
	break;
	case MSG_ID_CFW_CLOSE_SERVICE:
	{
		/* Free client-side conn */
		bfree(msg->conn);
		break;
	}
	default:
		//Nothing to do
		break;
	}
	if (handled)
		/* message handled here: free it */
		bfree(msg);
	else
		/* defer processing to message handler */
		h->handle_msg(msg, h->data);
}

cfw_handle_t cfw_init(void *queue, handle_msg_cb_t cb, void *cb_data)
{
	_cfw_handle_t *handle = (_cfw_handle_t *)balloc(sizeof(*handle), NULL);

	handle->handle_msg = cb;
	handle->data = cb_data;

	list_init(&handle->helper_list);

	handle->client_port_id = port_alloc(queue);

	cfw_port_set_handler(handle->client_port_id, client_handle_message,
			     handle);

	return (cfw_handle_t)handle;
}

int _cfw_send_message(struct cfw_message *message)
{
	return port_send_message(CFW_MESSAGE_HEADER(message));
}

void cfw_msg_free(struct cfw_message *msg)
{
	message_free(CFW_MESSAGE_HEADER(msg));
}
