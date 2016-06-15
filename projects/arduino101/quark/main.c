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

#include <stdbool.h>

/* VxMicro symbols */

#include <microkernel.h>
#include <arch/cpu.h>
#include "zephyr.h"

/* Operating System Abstraction */
#include "os/os.h"
#include "machine/soc/quark_se/quark/reboot.h"

/* Framework */
#include "cfw/cfw.h"
#include "cfw/cfw_debug.h"
#include "cfw/cfw_messages.h"
#include "cfw/cfw_internal.h"
#include "services_ids.h"

/* Infra */
#include "infra/log.h"
#include "infra/ipc.h"
#include "infra/time.h"
#include "infra/factory_data.h"
#include "util/workqueue.h"

/* Drivers */
#include "drivers/usb_api.h"
#ifdef CONFIG_USB_ACM
#include "drivers/usb_acm.h"
#endif

/* Services */
#include "services/gpio_service.h"

#include "drivers/soc_gpio.h"
#include "service_queue.h"

/* Board/SOC */
#include "board.h"
#include "machine.h"
#include "machine/soc/quark_se/quark/soc_setup.h"
#include "machine/soc/quark_se/quark/log_backend_uart.h"

#include "usb.h"

/* in order to shut off debug logs just make sure
   CONFIG_ARDUINO101_NO_DEBUG_PRINTS is set to y */
#ifdef CONFIG_ARDUINO101_NO_DEBUG_PRINTS
static void do_nothing(const char *x, uint16_t y)
{
	return;
}
struct log_backend null_log_backend = { do_nothing };

extern void __printk_hook_install(int (*hook)(int));
int ae_printk_null(int c) {
	return c;
}
#endif

extern void bs_init(void *batt_svc_queue, int service_id);

extern void *services;

/* Factory Data */
const struct customer_data* otp_data_ptr = (struct customer_data*)(FACTORY_DATA_ADDR + 0x200);

/* Local Functions declarations */
static void acm_rx_done(int actual, void *data);
static void acm_tx_done(int actual, void *data);

/**
 * Use the following defines just to make the tips of your finger happier.
 */
#define Rx_BUFF cdc_acm_shared_rx_buffer.data
#define Rx_HEAD cdc_acm_shared_rx_buffer.head
#define Rx_TAIL cdc_acm_shared_rx_buffer.tail
#define Tx_BUFF cdc_acm_shared_tx_buffer.data
#define Tx_HEAD cdc_acm_shared_tx_buffer.head
#define Tx_TAIL cdc_acm_shared_tx_buffer.tail
#define SBS     SERIAL_BUFFER_SIZE

/* Make sure BUFFER_LENGTH is not bigger then shared ring buffers */
#define BUFFER_LENGTH		64

#define LOOP_INTERVAL_MS 1
#define USB_ACM_TIMEOUT_MS (250)

#define USB_CONNECTED	    0x04
#define USB_DISCONNECTED    0x05

static struct cdc_ring_buffer cdc_acm_shared_rx_buffer;
static struct cdc_ring_buffer cdc_acm_shared_tx_buffer;
static struct cdc_acm_shared_data cdc_acm_buffers;

static uint8_t read_buffer[BUFFER_LENGTH*2];
static uint8_t write_buffer[BUFFER_LENGTH*2];

typedef enum {
	ACM_RX_DISABLED,
	ACM_RX_READY,
	ACM_RX_PENDING
} acm_rx_states;

typedef enum {
	ACM_TX_DISABLED,
	ACM_TX_READY,
	ACM_TX_PENDING
} acm_tx_states;

static volatile uint32_t acm_rx_state = ACM_RX_DISABLED;
static volatile uint32_t acm_tx_state = ACM_TX_DISABLED;

static volatile  int32_t acm_rx_data;

/* Indicates the state of the ACM port;
 *  acm_open == 1 => a serial client is listening at the other end
 *  acm_open == 0 => nobody listens us => drop data */
static volatile int acm_open = 0;
static int baud_rate;
static uint32_t reboot_timeout;
static int reboot_pending;

static usb_string_descriptor_t strings[] = {
      {
	/*String descriptor language, only one, so min size 4 bytes */
	/* 0x0409 English(US) language code used */
       .bLength = 4,	/*Descriptor size */
       .bDescriptorType = UDESC_STRING,	/*Descriptor type */
       .bString = {{0x09, 0x04}}
       },
      {
	/*Manufacturer String Descriptor "Intel" */
       .bLength = 0x0C,
       .bDescriptorType = UDESC_STRING,
       .bString = {{'I', 0}, {'n', 0}, {'t', 0}, {'e', 0}, {'l', 0},}
       },
      {
	/*Product String Descriptor "QRK_SE-Dev1.0" */
       .bLength = 0x17,
       .bDescriptorType = UDESC_STRING,
       .bString =
       {{'A', 0}, {'t', 0}, {'l', 0}, {'a', 0}, {'s', 0}, {' ', 0},
	       {'E', 0}, {'d', 0}, {'g', 0}, {'e', 0}, }
       },
      {
	/*Serial Number String Descriptor "00.01" */
       .bLength = 0x0C,
       .bDescriptorType = UDESC_STRING,
       .bString =
       {{'0', 0}, {'0', 0}, {'.', 0}, {'0', 0}, {'1', 0}, }
      },
};

bool convert2Unicode(const char * ascii_ptr,  uint32_t ascii_len, usb_string_descriptor_t *usb_desc_ptr)
{
    unsigned int counter = 0;

    if ((ascii_ptr == NULL) || (usb_desc_ptr == NULL))
    {
       return false;
    }

    // the current data struct does not have this problem but
    // keeping this checking only to avoid problems in the future changes
    if (ascii_len > USB_MAX_STRING_LEN)
    {
       ascii_len = USB_MAX_STRING_LEN - 1;
    }

    // clear the buffer
    memset((void*) usb_desc_ptr->bString, 0, sizeof(usb_desc_ptr->bString));

    for(counter = 0; counter < ascii_len + 1; counter++)
    {
         // no special characters in the descriptor, so simple conversion
         usb_desc_ptr->bString[counter][0] = *ascii_ptr++;
    }

    // the number of characters converted it is possible to update
    // the bLenght
    usb_desc_ptr->bLength = ascii_len * 2 + 2;

    return true;
}

void *cfw_alloc(int size, OS_ERR_TYPE * err)
{
	void *ptr;

	int flags = irq_lock();

	ptr = balloc(size, err);
	if (ptr == NULL)
		pr_error(LOG_MODULE_MAIN, "Cannot Allocate memory from: %p!!!!",
			 __builtin_return_address(0));
	irq_unlock(flags);
	return ptr;
}

void cfw_free(void *ptr, OS_ERR_TYPE * err)
{
	int flags = irq_lock();
	bfree(ptr);
	irq_unlock(flags);
}

static T_QUEUE queue;

int send_message_ipc_uart(struct message *message)
{
	return -1;
	/* TODO implementation to be done to send messages to the Nordic */
	/* use ipc_uart_send_message */
}

void free_message_ipc_uart(struct message *message)
{
	/* TODO implementation to be done to free messages received from Nordic */
	/* use ipc_uart_send_message */
}

int handle_ipc_uart_message(int channel, int len, unsigned char *data)
{
	return -1;
	/* TODO implementation to be done to handle the messages going over UART IPC */
}

void timer_main(void *priv_data)
{
	task_resume(TASKA);
}

void *ble_cfw_channel;

void workqueue_task(void *param)
{
	pr_info(LOG_MODULE_MAIN, "start workqueue");
	workqueue_run(&default_work_queue);
}

void init_workqueue_task()
{
	T_QUEUE q = queue_create(10, NULL);
	pr_info(LOG_MODULE_MAIN, "Initializing workqueue");
	workqueue_init(q, &default_work_queue);
	task_start(TASK_WORKQUEUE);
}

void acm_event(int event, int param)
{
	pr_info(LOG_MODULE_MAIN, "Got ACM event %d, param %d", event, param);
	log_flush();
	switch (event) {
		case ACM_EVENT_SET_BAUD: /* 2 */
			baud_rate = param;
			break;
		case ACM_EVENT_CONNECTED: /* 1 */
			acm_rx_state = ACM_RX_READY;
			pr_info(LOG_MODULE_MAIN, "acm_rx_state = ACM_RX_READY");
			break;
		case ACM_EVENT_DISCONNECTED: /* 0 */
			acm_rx_state = ACM_RX_DISABLED;
			acm_tx_state = ACM_TX_DISABLED;
			break;
		case ACM_SET_CONTROL_LINE_STATE: /* 3 */
			switch (param) {
				case ACM_CTRL_DCD | ACM_CTRL_CTS:
				case ACM_CTRL_DCD:
					acm_tx_state = ACM_TX_READY;
					// Inform the ARC that we have a connected host
					cdc_acm_buffers.host_open = true;
					break;
				case ACM_CTRL_CTS:
				case ACM_CTRL_DISC:
					acm_tx_state = ACM_TX_DISABLED;
					cdc_acm_buffers.host_open = false;
					break;
				default:
					pr_warning(LOG_MODULE_MAIN,
					"Unknown param for ACM_SET_CONTROL_LINE_STATE event: %d",
					param);
					break;
			}
			break;
		default:
			pr_debug(LOG_MODULE_MAIN,
				       "UNKNOWN event received: %d\n", event);
			break;
	}
	/* Check for board reset request from Arduino IDE */
	if ((event == ACM_EVENT_SET_BAUD) ||
			(event == ACM_SET_CONTROL_LINE_STATE)) {
		if ((1200 == baud_rate) && ((param & ACM_CTRL_DCD) == 0)) {
			reboot_pending = 1;
			reboot_timeout = get_uptime_ms() + USB_ACM_TIMEOUT_MS;
			pr_info(LOG_MODULE_MAIN,
			   "Baud rate change to 1200 detected, rebooting...");
		} else if (reboot_pending) {
			reboot_pending = 0;
			pr_info(LOG_MODULE_MAIN, "Reboot cancelled");
		}
	}
}

static void acm_tx_done(int actual, void *data)
{
	acm_tx_state = ACM_TX_READY;
}

static void acm_rx_done(int actual, void *data)
{
	if (acm_rx_data != 0) {
		pr_warning(LOG_MODULE_MAIN,
			"Unprocessed Rx data: %d, possible buffer overflow!",
			acm_rx_data);
	}
	acm_rx_state = ACM_RX_READY;
	pr_debug(LOG_MODULE_MAIN, "%s: acm_rx_state = ACM_RX_READY", __func__);
	if (actual <= 0) {
		pr_info(LOG_MODULE_MAIN, "No data!!! actual: %d", actual);
	}
	if (actual > BUFFER_LENGTH)
		pr_error(LOG_MODULE_MAIN, "!!To much data! actual: %d", actual);
	acm_rx_data = actual;
}

/**
 * Tx Flow:
 *  If ARC wrote something in the Tx buffer and the ACM is opened send data,
 *  otherwise just forward the tail.
 * Rx Flow:
 *  If read_buffer has data and there is room in Rx buffer, consume
 *  read_buffer into Rx buffer and queue a new read request to ACM driver.
 */
void cdc_acm_process_message(void)
{
	uint32_t ret;
	static uint32_t i = 0;
	uint32_t new_head;
	int flags;

/* TX Handling */
	if (Tx_HEAD == Tx_TAIL)
		goto rx_flow;
	flags = irq_lock();
	if (acm_tx_state == ACM_TX_READY) {
		/* Process Tx buffer */
		if ((Tx_HEAD != Tx_TAIL)) {
			int cnt = 0, index = Tx_TAIL;
			for (; (index != Tx_HEAD) &&
					(cnt < BUFFER_LENGTH);cnt++) {
				write_buffer[cnt] = Tx_BUFF[index];
				index = (index + 1) % SBS;
			}
			Tx_TAIL = (Tx_TAIL + cnt) % SBS;

			ret = acm_write(0, write_buffer, cnt, acm_tx_done,
							(void *)write_buffer);
			if (0 != ret) {
				pr_error(LOG_MODULE_MAIN,
					"acm_write() failed; ret: %d", ret);
			} else {
				acm_tx_state = ACM_TX_PENDING;
			}
		}
	} else if (acm_tx_state == ACM_TX_DISABLED) {
		/* Just move forward the tail of Tx buffer and drop the
		 * bytes */
		Tx_TAIL = Tx_HEAD;
	}
	irq_unlock(flags);

rx_flow:
/* RX Handling */
	/* Consume read_buffer into Rx ring buffer */
	while (acm_rx_data > 0) {
		if (!cdc_acm_buffers.device_open) {
			/* ARC is not ready to receive this data - discard it */
			acm_rx_data = 0;
			break;
		}

		/* Check room in Rx buffer */
		new_head = (Rx_HEAD + 1) % SBS;
		if (new_head != Rx_TAIL) {
			Rx_BUFF[Rx_HEAD] = *(read_buffer + i);
			Rx_HEAD = new_head;
			i++;
			acm_rx_data--;
		} else {
			/* Move on and give ARC time to consume
			 * some Rx buffer */
			break;
		}
	}
	/* read_buffer processed => reset Rx counter and queue a new read
	 * request */
	flags = irq_lock();
	if (acm_rx_state == ACM_RX_READY && !acm_rx_data) {
		i = 0;
		pr_debug(LOG_MODULE_MAIN, "queuing acm_read()");
		ret = acm_read(0, read_buffer, BUFFER_LENGTH,
				acm_rx_done, (void *)read_buffer);
		if (0 != ret) {
			pr_error(LOG_MODULE_MAIN,
					"acm_read() failed; ret: %d", ret);
		} else {
			acm_rx_state = ACM_RX_PENDING;
			pr_debug(LOG_MODULE_MAIN, "%s: acm_rx_state = ACM_RX_PENDING",
					__func__ );
		}
	}
	irq_unlock(flags);
}

void main_task(void *param)
{
	T_TIMER timer_task;
	int ret;

#ifdef CONFIG_ARDUINO101_NO_DEBUG_PRINTS
	__printk_hook_install(ae_printk_null);
#endif

	soc_setup(ipc_handle_message);

	/* Init logs */
	log_set_backend(
#ifdef CONFIG_ARDUINO101_NO_DEBUG_PRINTS
		null_log_backend
#else
		log_backend_uart
#endif
	);

	log_init();
	task_start(TASK_LOGGER);

        // need to check if there is something in the OTP area
        if ((otp_data_ptr->patternKeyStart == PATTERN_KEY_START) &&
            (otp_data_ptr->patternKeyEnd == PATTERN_KEY_END))
        {
          // PVT board with OTP are programmed
          // let's convert the vendor and the board name to unicode
          // and update the strings array with right data and lenght
          convert2Unicode((const char *)otp_data_ptr->vendor_name, otp_data_ptr->vendor_name_len, &strings[1]);
          convert2Unicode((const char *)otp_data_ptr->board_name , otp_data_ptr->board_name_len, &strings[2]);
          convert2Unicode((const char *)otp_data_ptr->product_sn , otp_data_ptr->product_sn_len, &strings[3]);

	}
         else if ((memcmp((const void *)otp_data_ptr->product_sn, INVALID_SN_F, sizeof(INVALID_SN_F)) != 0) &&
                  (memcmp((const void *)otp_data_ptr->product_sn, INVALID_SN_0, sizeof(INVALID_SN_0)) != 0))
	{
          // there is something written as product serial number, let's update the usb descriptor

          // checking if there is a valid product serial number
	  // there is not "magic" number in the customer data structure on DVT boards in order
          // to identify if the OTP customer area was written or not. The best we can do is check with
          // 0xFF or 0x00 sequences (blank flash)
          convert2Unicode((const char *)otp_data_ptr->product_sn , 15, &strings[3]);
	}

#ifdef CONFIG_USB_ACM
	pr_info(LOG_MODULE_MAIN, "Calling usb_acm_class_init()");
        usb_register_function_driver(usb_acm_class_init, NULL, strings);
	acm_init(0, acm_event);
//	acm_set_comm_state(0, 0x3f);
#endif
	usb_driver_init(SOC_USB_BASE_ADDR);

	interrupt_enable(SOC_USB_INTERRUPT);

	pr_info(LOG_MODULE_MAIN, "ipc_int()...");
	log_flush();
	/* Initialize IPC */
	ipc_init(0, 5, 1, 6, CPU_ID_ARC);

	pr_info(LOG_MODULE_MAIN, "Setting up cfw");
	/* Framework initializations */
	queue = queue_create(10, NULL);

#ifdef CONFIG_CFW_MASTER
	cfw_service_mgr_init(queue);
	ipc_async_init(queue);
#endif

	set_cpu_id(CPU_ID_QRK);
	set_cpu_message_sender(CPU_ID_ARC, ipc_async_send_message);
	set_cpu_free_handler(CPU_ID_ARC, ipc_async_free_message);
	set_cpu_message_sender(CPU_ID_BLE, send_message_ipc_uart);
	set_cpu_free_handler(CPU_ID_BLE, free_message_ipc_uart);

	init_workqueue_task();
	pr_info(LOG_MODULE_MAIN, "cfw init done");

#ifdef CONFIG_CFW_MASTER
	/* Initialized shared structure. */
	shared_data->ports = port_get_port_table();
	shared_data->services = services;
	shared_data->service_mgr_port_id = cfw_get_service_mgr_port_id();
#endif

	/* Configure CDC-ACM Shared Buffers prior to starting ARC */
	cdc_acm_buffers.rx_buffer = &cdc_acm_shared_rx_buffer;
	cdc_acm_buffers.tx_buffer = &cdc_acm_shared_tx_buffer;
	shared_data->cdc_acm_buffers = &cdc_acm_buffers;

	pr_info(LOG_MODULE_MAIN, "Start the ARC core");
	start_arc(0);

#ifdef CONFIG_CFW_PROXY
	_cfw_init_proxy(queue,
			shared_data->ports,
			shared_data->services,
			shared_data->service_mgr_port_id);
	ipc_async_init(queue);
#endif

	pr_info(LOG_MODULE_MAIN, "Create timer_main timer task...");
	timer_task = timer_create(timer_main, NULL, LOOP_INTERVAL_MS, 1, 1, NULL);

	/* we start GPIO service for sketch RESET button */
	gpio_service_init(queue, AON_GPIO_SERVICE_ID);

	struct device *gpio_dev = get_device(SOC_GPIO_AON_ID);
	gpio_cfg_data_t pin_cfg = {
		.gpio_type = GPIO_INPUT,
		.int_type = LEVEL,
		.int_polarity = ACTIVE_LOW,
		.int_debounce = DEBOUNCE_OFF,
		.int_ls_sync = LS_SYNC_OFF,
		.gpio_cb = NULL
	};

	bool gpio_value = 0;
	/* RESET button on Arduino101 board is gpio_aon[0] hence gpio_index 0 */
	uint8_t gpio_index = 0;

	/* set RESET GPIO button to be an input */
	ret = soc_gpio_set_config(gpio_dev, gpio_index, &pin_cfg);

	/* enable pullup on GPIO; Register PMUX_PULLUP[4], bit 8 - checked by testing  */
	SET_PIN_PULLUP(32*3 + 8, 1);

	pr_info(LOG_MODULE_MAIN, "soc_gpio_set_config returned %x", ret);

	pr_info(LOG_MODULE_MAIN, "QRK go to main loop");
	log_flush();
	while (1) {
		while(queue_process_message(queue));

		cdc_acm_process_message();

		if (reboot_pending) {
			if (get_uptime_ms() > reboot_timeout) {
				log_flush();
				reboot();
			}
		}

		ret = soc_gpio_read(gpio_dev, gpio_index, &gpio_value);
		if(ret !=  DRV_RC_OK) {
			pr_info(LOG_MODULE_MAIN, "soc_gpio_read error code: %x", ret);
		} else {
			if (gpio_value == 0) {
				pr_info(LOG_MODULE_MAIN, "GPIO_AON[0] - RESET triggered, rebooting");
				log_flush();
				reboot();
			}
		}

		task_suspend(TASKA);
	}
}
