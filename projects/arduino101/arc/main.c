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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "drivers/eiaextensions.h"
#include "machine.h"

#include <nanokernel.h>
#include "os/os.h"
#include "zephyr/os_config.h"
#include "zephyr/common.h"
#include "util/workqueue.h"
#include "infra/log.h"
#include "infra/log_impl_cbuffer.h"

#ifdef ARC_OS_UNIT_TEST
#include "test_suite.h"
#endif

#include "infra/ipc.h"
#include "infra/port.h"
#include "infra/pm.h"
#include "infra/device.h"

#include "cfw/cfw.h"
#ifdef CONFIG_CFW_SERVICES_TEST
#include "services/test_service.h"
#endif
#include "services/gpio_service.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#if defined(CONFIG_PSH_CORE) || defined(CONFIG_OPEN_CORE)
#include "sc_exposed.h"
#endif
#ifdef CONFIG_SERVICES_SENSOR
#include "sensor_test_client.h"
#endif
#ifdef CONFIG_SERVICES_SENSOR_IMPL
#include "sensor_svc_api.h"
#endif

#include "machine/soc/quark_se/arc/soc_setup.h"

extern void *services;

#define LOG_STACK_SIZE 600
char log_fiber_stack[LOG_STACK_SIZE];

#define WQ_STACK_SIZE 512
char wq_stack[WQ_STACK_SIZE];
void workFiber(int p1, int p2)
{
	T_QUEUE queue = queue_create(10, NULL);
	workqueue_init(queue, &default_work_queue);
	workqueue_run(&default_work_queue);
}

static T_QUEUE service_mgr_queue;

#ifdef CONFIG_CFW_SERVICES_TEST

svc_client_handle_t * test_service_handle = NULL;


static void my_handle_message(struct cfw_message * msg, void * param)
{
	pr_debug(LOG_MODULE_MAIN, "%s:%s for param: %s conn:%p",
		 __FILE__, __func__, (char*)param, CFW_MESSAGE_CONN(msg));

	switch (CFW_MESSAGE_ID(msg)) {
	case MSG_ID_CFW_OPEN_SERVICE: {
		pr_info(LOG_MODULE_MAIN, "%s:%s for conn: %s",
			__FILE__, __func__, (char*)CFW_MESSAGE_PRIV(msg));
		cfw_open_conn_rsp_msg_t * cnf = (cfw_open_conn_rsp_msg_t*)msg;
		int events[1] = {MSG_ID_TEST_1_EVT};
		test_service_handle = cnf->client_handle;
		cfw_register_events(test_service_handle, events,
				    1, CFW_MESSAGE_PRIV(msg));
	}
		break;

	case MSG_ID_CFW_REGISTER_EVT:
		if (!strcmp(msg->priv, "conn1")) {
			test_service_test_1(test_service_handle, "Coucou");
		}
		break;

	case MSG_ID_TEST_1_RSP: {
		pr_info(LOG_MODULE_MAIN, "got MSG_ID_TEST_1_RSP Priv: %s",
			(char*)CFW_MESSAGE_PRIV(msg));
		test_service_test_2(test_service_handle, "Testing 2");
		break;
	}

	case MSG_ID_TEST_2_RSP: {
		pr_info(LOG_MODULE_MAIN, "got MSG_ID_TEST_2_RSP Priv: %s",
			(char*)CFW_MESSAGE_PRIV(msg));
		//test_service_test_1(test_service_handle, "Coucou");
		break;
	}
	}
	cfw_msg_free(msg);
}

static void test_client_init(T_QUEUE queue)
{
	cfw_handle_t *h = cfw_init(service_mgr_queue, my_handle_message, "client");

	pr_info(LOG_MODULE_MAIN, "\nclient init done");
	cfw_open_service(h, TEST_SERVICE_ID, "conn1");

}

#endif

static void fst_sys_init(void)
{
    uint32_t    sreg;

    /* Start AON Counter */
    SCSS_REG_VAL(SCSS_AONC_CFG) |= 0x00000001;

    /* Enable instruction cache - TODO fix magic numbers */
    sreg = _lr(0x11);
    sreg &= 0xfffffffe;
    _sr(sreg, 0x11);  /* Bit 0 of Aux Reg 0x11 */

    pr_debug(LOG_MODULE_MAIN, "IRQ Reg: %p\n", _lr(0xe));
}

static void services_init()
{
	set_cpu_message_sender(CPU_ID_QRK, ipc_async_send_message);
	set_cpu_free_handler(CPU_ID_QRK, ipc_async_free_message);

	service_mgr_queue = queue_create(64, NULL);
	pr_info(LOG_MODULE_MAIN, "Ports: %p services: %p %d", shared_data->ports,
		shared_data->services, shared_data->service_mgr_port_id);

#ifdef CONFIG_CFW_MASTER
	set_cpu_id(CPU_ID_ARC);
	cfw_service_mgr_init(service_mgr_queue);
	/* Initialized shared structure. */
	shared_data->ports = port_get_port_table();
	shared_data->services = services;
	shared_data->service_mgr_port_id = cfw_get_service_mgr_port_id();
#endif

	soc_setup();

#ifdef CONFIG_CFW_PROXY
	_cfw_init_proxy(service_mgr_queue, shared_data->ports,
			shared_data->services, shared_data->service_mgr_port_id);
	ipc_async_init(service_mgr_queue);
#endif
	pr_info(LOG_MODULE_MAIN, "cfw init done");
#ifdef CONFIG_CFW_SERVICES_TEST
	test_service_init(service_mgr_queue, TEST2_SERVICE_ID);
	pr_info(LOG_MODULE_MAIN, "test service init done");
#endif
#ifdef CONFIG_SERVICES_QRK_SE_GPIO_IMPL
	gpio_service_init(service_mgr_queue, SS_GPIO_SERVICE_ID);
#ifdef CONFIG_SOC_GPIO_AON
	gpio_service_init(service_mgr_queue, AON_GPIO_SERVICE_ID);
#endif
	pr_info(LOG_MODULE_MAIN, "test gpio service init done");
#endif
#ifdef CONFIG_SERVICES_QRK_SE_ADC_IMPL
	adc_service_init(service_mgr_queue, SS_ADC_SERVICE_ID);
	pr_info(LOG_MODULE_MAIN, "test adc service init done");
#endif
#ifdef CONFIG_CFW_SERVICES_TEST
	test_client_init(service_mgr_queue);
#endif
#if defined(CONFIG_PSH_CORE) || defined(CONFIG_OPEN_CORE)
	sensor_core_create();
#endif
#ifdef CONFIG_SERVICES_SENSOR_IMPL
	ss_service_init(service_mgr_queue);
	pr_info(LOG_MODULE_MAIN, "Score_Svc init done");
#endif
	pr_info(LOG_MODULE_MAIN, "Service init done");
}

void main(void)
{
	struct cfw_message * message;
	T_QUEUE_MESSAGE m;
	OS_ERR_TYPE err = E_OS_OK;

#ifdef TOGGLE_LED_ON_TICK
	/* For checking tick rate, toggle led on tick */
	unsigned int old = 0;
	unsigned int currentTick;
#endif
	/* Setup interrupt priorities, clocks etc ....*/
	fst_sys_init();

	framework_init_common();
	log_init();
	services_init();
	task_fiber_start(&log_fiber_stack[0], LOG_STACK_SIZE,(nano_fiber_entry_t)log_task, 0, 0, 51, 0);
	pr_info(LOG_MODULE_MAIN, "Platform init done");

#ifndef CONFIG_ARC_OS_UNIT_TESTS
	task_fiber_start(&wq_stack[0], WQ_STACK_SIZE, workFiber, 0, 0, 0, 0);
#else
	extern void run_os_test_suite();
	run_os_test_suite();
#endif

	SOC_MBX_INT_UNMASK(0);

	while(1) {
		m = NULL;
		queue_get_message(service_mgr_queue, &m, OS_NO_WAIT, &err);
		message = (struct cfw_message *) m;
		if ( err == E_OS_OK && message != NULL ) {
			port_process_message(&message->m);
		}
#ifdef CONFIG_PM_PUPDR
		pm_idle();
#endif

#ifdef TOGGLE_LED_ON_TICK
		currentTick = nano_node_tick_get_32();
		if (old != currentTick) {
			static int val = 0;
			old = currentTick;
			val = val ? 0 : 1;
			qrk_cxxxx_gpio_write(0, 0, val);
		}
#endif
	}
}

/* TODO: remove all this code */
void * cfw_alloc(int size, OS_ERR_TYPE * err)
{
	void * ptr;
	int flags = interrupt_lock();
	ptr = balloc(size, err);
	if (ptr == NULL) pr_error(LOG_MODULE_MAIN, "Cannot Allocate memory from: %p!!!!", __builtin_return_address(0));
	interrupt_unlock(flags);
	return ptr;
}

void cfw_free(void * ptr, OS_ERR_TYPE *err)
{
	int flags = interrupt_lock();
	bfree(ptr);
	interrupt_unlock(flags);
}
