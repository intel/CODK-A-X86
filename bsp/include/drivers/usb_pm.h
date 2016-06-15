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

#ifndef _USB_PM_H_
#define _USB_PM_H_

#include <stdint.h>
#include <stdbool.h>
#include "infra/pm.h"
// Internal includes
#include "util/list.h"

/**
 * @defgroup usb_pm USB PM
 * USB driver wrapper that controls power management.
 *
 * This driver implements power management for USB driver.
 * It does not implement a full USB driver, because it is already done in the bootloader.
 * @ingroup usb_driver
 * @{
 */

/**
 * usb power management driver.
 */
extern struct driver usb_pm_driver;

/**
 * Interrupt sources supported by usb_pm driver
 */
enum {
	USB_AON_IRQ_SOURCE,
	USB_COMPARATOR_IRQ_SOURCE
};

/**
 * Structure to handle a usb_pm device
 */
struct usb_pm_info {
	uint8_t evt_dev_id;       /*!< Name of driver to use for USB */
	uint8_t interrupt_source; /*!< Type of source to use */
	uint8_t source_pin;       /*!< Pin to use for the selected source */
	uint8_t vusb_enable_port; /*!< Gpio port connected to enable VUSB regulator */
	uint8_t vusb_enable_pin;  /*!< Pin number connected to VUSB regulator */
	/* Internal driver fields */
	bool is_plugged;          /*!< USB plug status */
	struct device *evt_dev;   /*!< USB event source device */
	list_head_t cb_head;      /*!< List for user callback functions */
};

/**
 * Register a callback to handle USB plug event
 *
 * @param   dev  usb_pm device to use
 * @param   cb   callback to register
 * @param   priv user priv data (passed to callback)
 *
 * @return  TRUE if register succeeded.
 */
int usb_pm_register_callback(struct device *dev, void (*cb)(bool, void*), void *priv);

/**
 * Unregister a callback function
 *
 * @param   dev  usb_pm device to use
 * @param   cb   callback to unregister
 *
 * @return  TRUE if unregister succeeded.
 */
int usb_pm_unregister_callback(struct device *dev, void (*cb)(bool, void*));

/**
 * Get current USB plug status
 *
 * @param   dev usb_pm device to use
 *
 * @return  TRUE if USB is plugged else false
 */
bool usb_pm_is_plugged(struct device *dev);

/**
 * Check if suspend mode is allowed
 *
 * @param   dev   usb_pm device to use
 * @param   state state to transition to
 *
 * @return  TRUE if device allows transition to new state else false
 */
bool usb_pm_is_suspend_allowed(struct device *dev, PM_POWERSTATE state);

/** @} */

#endif /* _USB_PM_H_ */
