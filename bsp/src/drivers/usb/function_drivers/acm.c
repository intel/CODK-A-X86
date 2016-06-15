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

#include <string.h>
#include <stdio.h>
#include "drivers/usb_acm.h"
#include "infra/log.h"
#include "util/workqueue.h"
#include "util/list.h"
#include "usb.h"
#include "usb_driver_interface.h"

#define MAX_OUT_XFER 64

/**
 * \file
 *	\brief USB interface test code.
 *
 *	This file implements a simple usb profile that will read and write
 *	to 2 bulk enpoints.
 *
 */
static usb_device_descriptor_t device_desc = {
	.bLength = USB_DEVICE_DESCRIPTOR_SIZE,
	.bDescriptorType = 0x01,
	.bcdUSB[0] = 0x10,
	.bcdUSB[1] = 0x01,
	.bDeviceClass = CDC_CLASS,
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x00,
	.bMaxPacketSize = 64,
	.idVendor[0] = (CONFIG_USB_VENDOR_ID & 0xff),
	.idVendor[1] = ((CONFIG_USB_VENDOR_ID >> 8) & 0xff),
	.idProduct[0] = (CONFIG_USB_PRODUCT_ID & 0xff),
	.idProduct[1] = ((CONFIG_USB_PRODUCT_ID >> 8) & 0xff),
	.bcdDevice[0] = 0x87,
	.bcdDevice[1] = 0x80,
	.iManufacturer = 1,/* STRING_MANUFACTURER */
	.iProduct = 2,	 /* STRING_PRODUCT */
	.iSerialNumber = 3,/* STRING_SERIAL */
	.bNumConfigurations = 1,
};

#ifdef CONFIG_ACM_DUAL
#define NUM_ACM_CHANNELS 2
#else
#define NUM_ACM_CHANNELS 1
#endif

struct custom_config_descriptor {
	usb_config_descriptor_t config_descriptor;
	usb_interface_descriptor_t comm_interface;
	unsigned char cdc_desc[19];
	usb_endpoint_descriptor_t notif_ep;
	usb_interface_descriptor_t data_interface;
	usb_endpoint_descriptor_t endpoints[2];
#ifdef CONFIG_ACM_DUAL
	usb_interface_descriptor_t comm_interface2;
	unsigned char cdc_desc2[19];
	usb_endpoint_descriptor_t notif_ep2;
	usb_interface_descriptor_t data_interface2;
	usb_endpoint_descriptor_t endpoints2[2];
#endif
} UPACKED;

#ifdef CONFIG_ACM_DUAL
usb_endpoint_descriptor_t ep_descs[6] = {
#else
usb_endpoint_descriptor_t ep_descs[3] = {
#endif
{
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN + 1,
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 10,
		.wMaxPacketSize[1] = 0,
		.bInterval = 1,
	},
	{
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN + 2,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = 0x40,
		.wMaxPacketSize[1] = 0,
		.bInterval = 0,
	},
	{
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_OUT + 1,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = MAX_OUT_XFER & 0xff,
		.wMaxPacketSize[1] = (MAX_OUT_XFER >> 8) & 0xff,
		.bInterval = 0,
	},
#ifdef CONFIG_ACM_DUAL
{
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN + 3,
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 10,
		.wMaxPacketSize[1] = 0,
		.bInterval = 1,
	},
	{
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN + 4,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = 0x40,
		.wMaxPacketSize[1] = 0,
		.bInterval = 0,
	},
	{
		.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_OUT + 2,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = MAX_OUT_XFER & 0xff,
		.wMaxPacketSize[1] = (MAX_OUT_XFER >> 8) & 0xff,
		.bInterval = 0,
	},
#endif
};

static struct custom_config_descriptor config_desc = {
	.config_descriptor = {
		.bLength = USB_CONFIG_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x2,
		.wTotalLength[0] = sizeof(struct custom_config_descriptor) & 0xff,
		.wTotalLength[1] = (sizeof(struct custom_config_descriptor) >> 8) & 0xff,
#ifdef CONFIG_ACM_DUAL
		.bNumInterface = 4,
#else
		.bNumInterface = 2,
#endif
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_BUS_POWERED | UC_SELF_POWERED,
		.bMaxPower = 250,	/* max current in 2mA units */
		},
	.comm_interface = {
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x4,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = CDC_CLASS,
		.bInterfaceSubClass = CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = CDC_PROTOCOL_V250,	/*Protocol */
		.iInterface = 0,
	 },
	.cdc_desc = {
		5,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_HEADER,
		0x20, 0x01,

		4,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_ACM,
		2,

		5,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_UNION,
		0,
		1,

		5,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_CALL_MANAGEMENT,
		0x00,
		1
	},
	.notif_ep = {
			.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
			.bDescriptorType = UDESC_ENDPOINT,
			.bEndpointAddress = UE_DIR_IN + 1,
			.bmAttributes = UE_INTERRUPT,
			.wMaxPacketSize[0] = 10,
			.wMaxPacketSize[1] = 0,
			.bInterval = 1,
	},
	.data_interface = {
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x4,
		.bInterfaceNumber = 1,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = CDC_DATA_INTERFACE_CLASS,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,	/*Protocol */
		.iInterface = 0,
	},
	.endpoints = {
		{
			.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
			.bDescriptorType = UDESC_ENDPOINT,
			.bEndpointAddress = UE_DIR_IN + 2,
			.bmAttributes = UE_BULK,
			.wMaxPacketSize[0] = 0x40,
			.wMaxPacketSize[1] = 0,
			.bInterval = 0,
		},
		{
			.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
			.bDescriptorType = UDESC_ENDPOINT,
			.bEndpointAddress = UE_DIR_OUT + 1,
			.bmAttributes = UE_BULK,
			.wMaxPacketSize[0] = MAX_OUT_XFER & 0xff,
			.wMaxPacketSize[1] = (MAX_OUT_XFER >> 8) & 0xff,
			.bInterval = 0,
		}
	},
#ifdef CONFIG_ACM_DUAL
	.comm_interface2 = {
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x4,
		.bInterfaceNumber = 2,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = CDC_CLASS,
		.bInterfaceSubClass = CDC_SUBCLASS_ACM,
		.bInterfaceProtocol = CDC_PROTOCOL_V250,	/*Protocol */
		.iInterface = 0,
	 },
	.cdc_desc2 = {
		5,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_HEADER,
		0x20, 0x01,

		4,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_ACM,
		2,

		5,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_UNION,
		2,
		3,

		5,
		CDC_DESCRIPTOR_TYPE_CS_INTERFACE,
		CDC_DESCRIPTOR_SUBTYPE_CALL_MANAGEMENT,
		0x00,
		3
	},
	.notif_ep2 = {
			.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
			.bDescriptorType = UDESC_ENDPOINT,
			.bEndpointAddress = UE_DIR_IN + 3,
			.bmAttributes = UE_INTERRUPT,
			.wMaxPacketSize[0] = 10,
			.wMaxPacketSize[1] = 0,
			.bInterval = 1,
	},
	.data_interface2 = {
		.bLength = USB_INTERFACE_DESCRIPTOR_SIZE,
		.bDescriptorType = 0x4,
		.bInterfaceNumber = 3,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = CDC_DATA_INTERFACE_CLASS,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,	/*Protocol */
		.iInterface = 0,
	},
	.endpoints2 = {
		{
			.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
			.bDescriptorType = UDESC_ENDPOINT,
			.bEndpointAddress = UE_DIR_IN + 4,
			.bmAttributes = UE_BULK,
			.wMaxPacketSize[0] = 0x40,
			.wMaxPacketSize[1] = 0,
			.bInterval = 0,
		},
		{
			.bLength = USB_ENDPOINT_DESCRIPTOR_SIZE,
			.bDescriptorType = UDESC_ENDPOINT,
			.bEndpointAddress = UE_DIR_OUT + 2,
			.bmAttributes = UE_BULK,
			.wMaxPacketSize[0] = MAX_OUT_XFER & 0xff,
			.wMaxPacketSize[1] = (MAX_OUT_XFER >> 8) & 0xff,
			.bInterval = 0,
		}
	},
#endif
};


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
	/*Product String Descriptor "QRK-Dev1.0" */
       .bLength = 0x16,
       .bDescriptorType = UDESC_STRING,
       .bString =
       {{'A', 0}, {'T', 0}, {'P', 0}, {'-', 0}, {'D', 0}, {'e', 0},
	       {'v', 0}, {'1', 0}, {'.', 0}, {'0', 0}, }
       },
      {
	/*Serial Number String Descriptor "00.01" */
       .bLength = 0x0C,
       .bDescriptorType = UDESC_STRING,
       .bString =
       {{'0', 0}, {'0', 0}, {'.', 0}, {'0', 0}, {'1', 0}, }
      },
};

static void (*acm_event_cb[2])(int, int);

static struct acm_line_coding {
	uint32_t dwDTERate;
	uint8_t bCharFormat;
	uint8_t vParityType;
	uint8_t bDataBits;
} acm_line_coding[NUM_ACM_CHANNELS];

static struct acm_interrupt_request {
	usb_device_request_t h;
	uint8_t state;
	uint8_t zero;
} acm_interrupt_request[NUM_ACM_CHANNELS] = {
	{
		.h = {
			.bmRequestType = 0xa1,
			.bRequest = ACM_NOTIFICATION_SERIAL_STATE,
			.wValue = {0, 0},
			.wIndex = {0, 0},
			.wLength = {2, 0},
		},
		.state = ACM_SERIAL_STATE_RX_CARRIER | ACM_SERIAL_STATE_TX_CARRIER,
		.zero = 0,
	},
#ifdef CONFIG_ACM_DUAL
	{
		.h = {
			.bmRequestType = 0xa1,
			.bRequest = ACM_NOTIFICATION_SERIAL_STATE,
			.wValue = {0, 0},
			.wIndex = {2, 0},
			.wLength = {2, 0},
		},
		.state = ACM_SERIAL_STATE_RX_CARRIER | ACM_SERIAL_STATE_TX_CARRIER,
		.zero = 0,
	}
#endif
};

static uint16_t acm_ctrl_line_state[NUM_ACM_CHANNELS] = {0};

struct acm_request {
	list_t list;
	void (*xfer_done)(int actual, void *data);
	void * data;
#define STATE_READY 0
#define STATE_PENDING 1
#define STATE_CLOSING 2
#define STATE_CLOSED 3
	uint8_t state;

#define DIRECTION_READ  0
#define DIRECTION_WRITE 1
	uint8_t direction;
	uint8_t ep;
	uint8_t channel;
};


list_head_t acm_read_requests[NUM_ACM_CHANNELS];
list_head_t acm_write_requests[NUM_ACM_CHANNELS];

void show_pending(void * elem, void *param)
{
	pr_info(LOG_MODULE_USB, "pending request: %s: %p", param, elem);
}

static int acm_class_handler(usb_device_request_t *pSetup, uint32_t * piLen,
			     uint8_t ** ppbData)
{
	*piLen = 0;
	int if_idx = UGETW(pSetup->wIndex) / 2;
	switch(pSetup->bRequest) {
		case ACM_REQUEST_SET_LINE_CODING:
			memcpy(&acm_line_coding[if_idx], *ppbData,
			       sizeof(struct acm_line_coding));
			pr_debug(LOG_MODULE_USB, "Set baud[%d]: %d",
				UGETW(pSetup->wIndex), acm_line_coding[if_idx].dwDTERate);
			if (acm_event_cb[if_idx]) {
				acm_event_cb[if_idx](ACM_EVENT_SET_BAUD,
						     acm_line_coding[if_idx].dwDTERate);
			}
			break;
		case ACM_REQUEST_GET_LINE_CODING:
			memcpy(*ppbData, &acm_line_coding[if_idx], UGETW(pSetup->wLength));
			*piLen = UGETW(pSetup->wLength);
			break;
		case ACM_REQUEST_SET_CONTROL_LINE_STATE:
			if ((acm_ctrl_line_state[if_idx] & 1) && !(UGETW(pSetup->wValue) & 1)) {
				list_foreach(&acm_read_requests[if_idx], show_pending, "r");
				list_foreach(&acm_write_requests[if_idx], show_pending, "w");
			}
			/* if event's param == 0, reset lines state */
			if (!(acm_ctrl_line_state[if_idx] & 0x01) && (UGETW(pSetup->wValue) & 1)) {
			}
			acm_ctrl_line_state[if_idx] = UGETW(pSetup->wValue);
			if (acm_event_cb[if_idx]) {
				acm_event_cb[if_idx](ACM_SET_CONTROL_LINE_STATE,
						pSetup->wValue[if_idx]);
			}
			break;
		default:
			pr_error(LOG_MODULE_USB, "Unhandled class request: %x",
				 pSetup->bRequest);
			break;
	}
	return 0;
}


static void acm_ep_complete(int ep_address, void *priv, int status, int actual)
{
	struct acm_request * req = (struct acm_request *) priv;

	pr_debug(LOG_MODULE_USB, "%s: status: %d actual: %d - %x", __func__,
		status, actual, ep_address);

	if (req) {
		if (req->direction == DIRECTION_READ) {
			list_remove(&acm_read_requests[req->channel], &req->list);
		} else {
			list_remove(&acm_write_requests[req->channel], &req->list);
		}
	}

	if (req && req->xfer_done) {
		req->state = STATE_READY;
		if (!status) {
			req->xfer_done(actual, req->data);
		} else {
			pr_debug(LOG_MODULE_USB, "status: %d", status);
		}
		bfree(req);
		return;
	}

	switch(ep_address) {
		case 0x81:
#ifdef CONFIG_ACM_DUAL
		case 0x83:
#endif
			break;
		default:
			pr_error(LOG_MODULE_USB,
				 "Unexpected endpoint complete (%x)", ep_address);
			break;
	}
}

static void acm_class_start()
{
	int ret;
	pr_debug(LOG_MODULE_USB, "trigger writes !!!");
	ret = usb_ep_write(0x81, (uint8_t*)&acm_interrupt_request[0],
		     sizeof(acm_interrupt_request), NULL);
	if (ret) pr_error(LOG_MODULE_USB, "write 81 ret: %d", ret);
	if (acm_event_cb[0]) {
		acm_event_cb[0](ACM_EVENT_CONNECTED, 0);
	}
#ifdef CONFIG_ACM_DUAL
	ret = usb_ep_write(0x83, (uint8_t*)&acm_interrupt_request[1],
		     sizeof(acm_interrupt_request), NULL);
	if (ret) pr_error(LOG_MODULE_USB, "write 83 ret: %d", ret);
	if (acm_event_cb[1]) {
		acm_event_cb[1](ACM_EVENT_CONNECTED, 0);
	}
#endif
}

static void acm_event_handler(struct usb_event * event)
{
	switch(event->event) {
		case USB_EVENT_SET_CONFIG:
			pr_debug(LOG_MODULE_USB, "Usb set config!");
			acm_class_start();
			break;
		case USB_EVENT_RESET:
			pr_debug(LOG_MODULE_USB, "Usb reset triggered!");
			break;
		case USB_EVENT_DISCONNECT: {
			int i;
			pr_info(LOG_MODULE_USB, "USB unplugged");
			for (i=0; i<sizeof(ep_descs)/sizeof(ep_descs[0]);i++) {
				pr_debug(LOG_MODULE_USB, "Disable ep %x",
						ep_descs[i].bEndpointAddress);
				usb_ep_disable(ep_descs[i].bEndpointAddress);
			}
			if(acm_event_cb[0]) {
				acm_event_cb[0](ACM_EVENT_DISCONNECTED, 0);
			}
#ifdef CONFIG_ACM_DUAL
			if(acm_event_cb[1]) {
				acm_event_cb[1](ACM_EVENT_DISCONNECTED, 0);
			}
#endif
		}
			break;
	}
}

/*
 * Interface implementation
 */
int acm_read(int idx, uint8_t * buffer, int len,
	     void (*xfer_done)(int actual, void *data), void *data)
{
	int ret;
	if (!list_empty(&acm_read_requests[idx])) {
		return DRV_RC_BUSY;
	}
	struct acm_request * req = (struct acm_request *)balloc(sizeof(*req), NULL);
	int flags = interrupt_lock();
	req->state = STATE_READY;
	req->direction = DIRECTION_READ;
	req->channel = idx;
	list_add(&acm_read_requests[idx], &req->list);
	interrupt_unlock(flags);
	req->xfer_done = xfer_done;
	req->data = data;
	req->ep = (idx == 0) ? 1 : 2;
	ret = usb_ep_read(req->ep, buffer,
			  len, req);
	if (ret) {
		/* read failed, remove the request from list */
		list_remove(&acm_read_requests[idx], &req->list);
		bfree(req);
	}
	return ret;
}

int acm_write(int idx, uint8_t * buffer, int len,
	      void (*xfer_done)(int actual, void *data), void *data)
{
	struct acm_request * req = (struct acm_request *)balloc(sizeof(*req), NULL);
	int flags = interrupt_lock();
	int ret;
	req->state = STATE_READY;
	req->direction = DIRECTION_WRITE;
	req->channel = idx;
	list_add(&acm_write_requests[idx], &req->list);
	interrupt_unlock(flags);
	req->xfer_done = xfer_done;
	req->data = data;
	req->ep = (idx == 0) ? 0x82 : 0x84;
	ret = usb_ep_write(req->ep, buffer,
			   len, req);
	if (ret) {
		/* write failed, remove the request from list */
		list_remove(&acm_write_requests[idx], &req->list);
		bfree(req);
	}
	return ret;
}


void acm_set_comm_state(int idx, uint8_t state)
{
	if (state != acm_interrupt_request[idx].state) {
		acm_interrupt_request[idx].state = state;
		usb_ep_write(ep_descs[3*idx].bEndpointAddress,
			     (uint8_t*)&acm_interrupt_request[idx],
			     sizeof(acm_interrupt_request), NULL);
	}
}

int acm_init(int idx, void (*event_cb)(int, int))
{
	acm_event_cb[idx] = event_cb;
	list_init(&acm_read_requests[idx]);
	list_init(&acm_write_requests[idx]);
	return 0;
}

#define ACM_EP0_BUF_SZ 128
static uint8_t ep0_buffer[ACM_EP0_BUF_SZ];

struct usb_interface_init_data init_data = {
	.ep_complete = acm_ep_complete,
	.class_handler = acm_class_handler,
	.usb_evt_cb = acm_event_handler,
	.ep0_buffer = ep0_buffer,
	.ep0_buffer_size = ACM_EP0_BUF_SZ,
	.dev_desc = &device_desc,
	.conf_desc = &config_desc.config_descriptor,
	.conf_desc_size = sizeof(config_desc),
	.strings_desc = strings,
	.num_strings = sizeof(strings)/sizeof(strings[0]),
	.eps = ep_descs,
	.num_eps = sizeof(ep_descs)/sizeof(ep_descs[0]),
};

void usb_acm_class_init(void *priv, usb_string_descriptor_t * alt_strings)
{
	if (alt_strings != NULL) {
		init_data.strings_desc = alt_strings;
	}
	usb_interface_init((struct usb_interface_init_data *)&init_data);

}
