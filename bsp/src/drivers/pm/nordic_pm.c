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

#include <errno.h>
#include <stdbool.h>
#include "infra/device.h"
#include "infra/log.h"
#include "drivers/nordic_pm.h"
#include "drivers/soc_gpio.h"

#define DRV_NAME "nordic_pm"

static void nordic_pm_callback(bool state, void *priv_data)
{
	struct device *dev = (struct device*)priv_data;
	struct nordic_pm_info *priv = (struct nordic_pm_info*)dev->priv;

	pr_debug(LOG_MODULE_DRV, "%s %d: connected aon %d", DRV_NAME, dev->id,
			priv->wakeup_pin);
}

static int nordic_pm_setup(struct device* dev)
{
	int ret = 0;
	struct nordic_pm_info *priv = (struct nordic_pm_info*)dev->priv;
	// Configure GPIO AON as wake up source when going high
	gpio_cfg_data_t config;
	config.gpio_type = GPIO_INTERRUPT;
	config.int_type = EDGE;
	config.int_polarity = ACTIVE_HIGH;
	config.int_debounce = DEBOUNCE_ON;
	config.int_ls_sync = LS_SYNC_OFF;
	config.gpio_cb = nordic_pm_callback;
	config.gpio_cb_arg = dev;
	soc_gpio_deconfig(priv->gpio_dev, priv->wakeup_pin);
	ret = soc_gpio_set_config(priv->gpio_dev, priv->wakeup_pin, &config);

	pr_debug(LOG_MODULE_DRV, "%s %d: use aon - %d (%d)", DRV_NAME, dev->id,
			priv->wakeup_pin, ret);
	return ret;
}

bool nordic_pm_is_active(struct device *dev)
{
	struct nordic_pm_info *priv = (struct nordic_pm_info*)dev->priv;
	bool active = false;
	soc_gpio_read(priv->gpio_dev, priv->wakeup_pin, &active);
	return active;
}

bool nordic_pm_can_sleep(struct device *dev, PM_POWERSTATE state)
{
	if (state > PM_SHUTDOWN) {
		if (nordic_pm_is_active(dev)) {
			pr_debug(LOG_MODULE_DRV, "%s %d prevents sleep",
					DRV_NAME, dev->id);
			return false;
		}
	}
	return true;
}

static int nordic_pm_init(struct device* dev)
{
	struct nordic_pm_info *priv = (struct nordic_pm_info*)dev->priv;

	priv->gpio_dev = get_device(priv->gpio_dev_id);
	if (!priv->gpio_dev)
		return -EINVAL;
	return nordic_pm_setup(dev);
}

static int nordic_pm_suspend(struct device* dev, PM_POWERSTATE state)
{
	if (nordic_pm_is_active(dev)) {
		pr_warning(LOG_MODULE_DRV, "%s %d: suspend while in use",
				DRV_NAME, dev->id);
	}
	return 0;
}

static int nordic_pm_resume(struct device* dev)
{
	return nordic_pm_setup(dev);
}

struct driver nordic_pm_driver = {
	.init = nordic_pm_init,
	.suspend = nordic_pm_suspend,
	.resume = nordic_pm_resume
};
