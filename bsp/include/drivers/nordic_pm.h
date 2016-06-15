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

#ifndef _NORDIC_PM_H_
#define _NORDIC_PM_H_

#include "drivers/data_type.h"
#include "infra/device.h"

/**
 * @defgroup nordic_pm NORDIC PM
 * handle PM communication from nordic to Qrk_se platform
 *
 * This driver prevents deep sleep mode when Nordic is active
 * and configure a gpio AON to wakeup platform from deepsleep
 * on Nordic event.
 * @ingroup common_drivers
 * @{
 */

/**
 * Nordic power management driver.
 */
extern struct driver nordic_pm_driver;

/**
 * Structure to handle a nordic_pm device
 */
struct nordic_pm_info {
	uint8_t gpio_dev_id;       /*!< id of gpio driver to use */
	uint8_t wakeup_pin;        /*!< gpio ble_qrk_se_pin to use */
	/* Internal driver fields */
	struct device *gpio_dev;   /*!< gpio device corresponding to gpio_dev_id */
};

/**
 * Get current nordic activity status
 *
 * @param   dev nordic_pm device to use
 *
 * @return  TRUE if nordic is active else false
 */
bool nordic_pm_is_active(struct device *dev);

/**
 * Check if suspend mode is allowed
 *
 * @param   dev   nordic_pm device to use
 * @param   state state to transition to
 *
 * @return  TRUE if device allows transition to new state else false
 */
bool nordic_pm_can_sleep(struct device *dev, PM_POWERSTATE state);

/** @} */

#endif //  _NORDIC_PM_H_
