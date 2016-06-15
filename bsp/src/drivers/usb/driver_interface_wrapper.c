/*
 * Copyright (c) 2016, Intel Corporation. See the bottom of the file for full license.
 */

#include "usb_api.h"

#include <stdarg.h>
#include <assert.h>

#include "os/os.h"
#include "infra/log.h"
#include "infra/device.h"
#include "usb.h"
#include "usb_driver_interface.h"
#include "quark_se_mapping.h"
#include "usb_shared_interface.h"
#include "drivers/usb_pm.h"
#include "machine.h"

/**
 * \file
 * \brief USB interface wrapper.
 *
 *	This file implements the wrapper to the USB driver that is implemented in
 *	the bootloader image.
 *
 */

/**
 * USB Interrupt routine.
 * Calls the driver's interrupt routine.
 */
void usb_ISR()
{
	usb_driver_intf->usb_isr();
}

static int usb_printk(int level, const char *fmt, ...)
{
	int ret;
	va_list args;
	va_start(args, fmt);
	ret = log_vprintk(level, LOG_MODULE_USB, fmt, args);
	va_end(args);
	return ret;
}

static uint32_t usb_tracked_alloc[2];
static uint32_t usb_tracked_address[64];

static inline void track_address(void *addr) {
	int i = 0;
	int j = 0;
	while (i<32 && j<2) {
		if((usb_tracked_alloc[j] >> i) & 1) {
			i++;
			if(i>=32) {
				i = 0;
				j++;
			}
			continue; /*buffer tracked*/
		}
		usb_tracked_alloc[j] |= (1 << i);
		usb_tracked_address[32*j + i] = (uint32_t)addr;
		break;
	}
}

static inline void untrack_address(void *addr) {

	int i = 0;
	int j = 0;
	while (i<32 && j<2) {
		if((usb_tracked_alloc[j] >> i) & 1) {
			if(usb_tracked_address[32*j + i] == (uint32_t)addr) {
				usb_tracked_alloc[j] &= ~(1 << i);
				break;
			}
		}
		i++;
		if(i>=32) {
			i = 0;
			j++;
		}
	}
}

static inline int garbage_collect(void) {
	int ret = 0;
	for (int j = 0; j<2; j++) {
		for (int i = 0; i<32; i++) {
			if((usb_tracked_alloc[j] >> i) & 1) {
				bfree((void*)usb_tracked_address[32*j + i]);
				ret = 32*j + i;
			}
		}
	}
	if (usb_tracked_alloc[0] || usb_tracked_alloc[1]) {
		usb_tracked_alloc[0] = 0;
		usb_tracked_alloc[1] = 0;
		return ret;
	} else
		return 0;
}

static int usb_initialized;
static void * usb_alloc(int size)
{
	void *tmp =  balloc(size, NULL);
	track_address(tmp);
	return tmp;
}

static void usb_free(void * ptr)
{
	bfree(ptr);
	untrack_address(ptr);
}

struct {
	function_driver_init_t function_init;
	void *priv;
	usb_string_descriptor_t * alt_strings;
} f;

void usb_plug_evt(bool plugged, void *param)
{
	if (plugged) {
		if (usb_initialized == 0) {
			usb_driver_intf->usb_connect();
			usb_driver_intf->usb_driver_init(SOC_USB_BASE_ADDR);
                        f.function_init(f.priv, f.alt_strings);
			interrupt_enable(SOC_USB_INTERRUPT);
			usb_initialized = 1;
		} else {
			pr_error(LOG_MODULE_USB,
				 "Trying to init already initialized driver");
		}
	} else {
		if (usb_initialized == 1) {
			interrupt_disable(SOC_USB_INTERRUPT);
			usb_driver_intf->usb_disconnect();
			usb_initialized = 0;
			int ret = garbage_collect();
			if (ret) {
				pr_error(LOG_MODULE_USB,
					 "Mem leak avoided (idx: %d)", ret);
			}
		} else {
			pr_error(LOG_MODULE_USB,
				 "Trying to free already freed driver");
		}
	}
}

int usb_register_function_driver(function_driver_init_t f_init, void *priv, usb_string_descriptor_t * alt)
{
	if (f_init == NULL) {
		pr_error(LOG_MODULE_DRV, "Missing function driver initialization function");
		return -1;
	}
	f.function_init = f_init;
	f.priv = priv;
        f.alt_strings = alt;
	return 0;
}


int usb_driver_init(uint32_t base_addr)
{
	int version = usb_driver_intf->version();
	usb_initialized = 0;
	usb_tracked_alloc[0] = 0;
	usb_tracked_alloc[1] = 0;
	if (version != USB_DRIVER_INTERFACE_VERSION) {
		pr_error(LOG_MODULE_DRV, "Incompatible USB interface (%d) expected %d",
				version, USB_DRIVER_INTERFACE_VERSION);
		return -1;
	}

	struct device * usbdev = get_device(USB_PM_ID);
	assert(usbdev);
	usb_pm_register_callback(usbdev, usb_plug_evt, NULL);

	usb_driver_os_dep->alloc = usb_alloc;
	usb_driver_os_dep->free = usb_free;
	usb_driver_os_dep->printk = usb_printk;

	/* set initial USB state regarding USB cable connection */
	struct usb_pm_info *usb_pm_info = (struct usb_pm_info*)usbdev->priv;
	if( usb_pm_info->is_plugged) {
		/* Explicitely call the callback */
		usb_plug_evt(1, NULL);
	}
	return 0;

}

int usb_interface_init(struct usb_interface_init_data * init_data)
{
	return usb_driver_intf->usb_interface_init(init_data);
}

int usb_ep_disable(int ep_address)
{
	return usb_driver_intf->usb_ep_disable(ep_address);
}

int usb_ep_read(int ep_address, uint8_t *buf, int len, void *priv)
{
	return usb_driver_intf->usb_ep_read(ep_address, buf, len, priv);
}

int usb_ep_write(int ep_address, uint8_t *buf, int len, void *priv)
{
	return usb_driver_intf->usb_ep_write(ep_address, buf, len, priv);
}

int usb_get_config()
{
	return usb_driver_intf->usb_get_config();
}

/*
 * The full license is here at the bottom of the file because
 * changing the top portion of the file modifies the final binaries.
 *
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