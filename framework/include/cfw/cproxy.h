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

#ifndef _CPROXY_H_
#define _CPROXY_H_

#include "os/os.h"
#include "cfw/cfw_client.h"

/**
 * Disconnect from the proxy.
 *
 * \param h the service handle
 */
void cproxy_disconnect(svc_client_handle_t *h);

/**
 * Connect to the proxy for a specific service.
 *
 * This actually opens the specified service, creates a local context and
 * returns the service handle.
 * All responses received from the specified service using the returned
 * client handle will now be forwarded to the callback.
 * Warning: this is a blocking call.
 *
 * \param service_id The targeted service
 * \param cb         The messages callback for that service
 * \param data       Opaque data that will be passed back with messages
 *
 * \return the service handle to use to call the specified service
 */
svc_client_handle_t * cproxy_connect(int service_id,
		void(*cb)(struct cfw_message *, void*), void *data);

/**
 * Initialize the proxy.
 *
 * \param queue The queue the proxy will use
 */
void cproxy_init(T_QUEUE queue);

#endif /* _CPROXY_H_ */
