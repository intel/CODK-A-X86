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

#include "os/os.h"
#include "infra/device.h"
#include "infra/log.h"
#include "infra/time.h"
#include "drivers/usb_pm.h"
#include "drivers/usb_api.h"
#include "drivers/gpio.h"
#include "drivers/soc_gpio.h"
#include "drivers/soc_comparator.h"
#include "machine.h"
#include "drivers/usb_acm.h"

struct usb_pm_cb_list;

struct usb_pm_cb_list {
	list_t next;
	void (*cb)(bool, void*);
	void *priv;
};

static int usb_pm_init(struct device* dev);
static int usb_pm_suspend(struct device* dev, PM_POWERSTATE state);
static int usb_pm_resume(struct device* dev);

static void call_user_callback(void *item, void *param)
{
	((struct usb_pm_cb_list*)item)->cb((bool)param, ((struct usb_pm_cb_list*)item)->priv);
}

#ifdef CONFIG_USB

#define OSC_INTERNAL 1
#define OSC_EXTERNAL 0
#define INTERNAL_OSC_TRIM 0x240

/* get_uptime_32k returns always-on counter value running off 32KHz RTC clock */
static void delay_us(uint32_t us)
{
        int timeout = get_uptime_32k() + (us + 30 ) / 30;
        while(get_uptime_32k() < timeout) ;
}

void set_oscillator(int internal)
{
	if (internal) {
		/* Start internal oscillator (with trim) */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) |=
			INTERNAL_OSC_TRIM << OSC0_CFG1_INTERNAL_OSC_TRIM_BIT |
			OSC0_CFG1_INTERNAL_OSC_EN_MASK;
		/* Wait internal oscillator ready */
		while (!((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_STAT1) & OSC0_STAT1_LOCK_INTERNAL)))
			;
		/* Trim internal oscillator */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) =
			INTERNAL_OSC_TRIM << OSC0_CFG1_INTERNAL_OSC_TRIM_BIT |
			OSC0_CFG1_INTERNAL_OSC_EN_MASK;
	} else {
		/* Set clk to 32MHz, external oscillator, 5.5pF load */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) |=
			OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF |
			OSC0_CFG1_XTAL_OSC_EN_MASK;
		/* Wait internal regulator ready */
		while (!((MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_STAT1) & OSC0_STAT1_LOCK_XTAL)))
			;
		/* Switch to external oscillator */
		MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_OSC0_CFG1) =
			OSC0_CFG1_XTAL_OSC_TRIM_5_55_PF |
			(OSC0_CFG1_XTAL_OSC_EN_MASK | OSC0_CFG1_XTAL_OSC_OUT_MASK);
	}
}

void enable_vusb_regulator(struct usb_pm_info *usb_pm, bool enable){
	gpio_cfg_data_t config;
	struct device * gpiodev;
	if (usb_pm->vusb_enable_port){
		gpiodev = get_device(usb_pm->vusb_enable_port);
		if (!gpiodev)
			return;
		config.gpio_type = GPIO_OUTPUT;
		soc_gpio_set_config(gpiodev, usb_pm->vusb_enable_pin, &config);
		soc_gpio_write(gpiodev, usb_pm->vusb_enable_pin, enable);

                /* We need to wait some time here for USB voltage regulator to stabilize */
                delay_us(90);
	} else {
		pr_warning(LOG_MODULE_USB, "USB Voltage regulator enable not supported");
	}
}
#endif

/* Soc Specific initialization */
static inline void usb_generic_callback(struct device *dev)
{
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;
	// Toggle USB status
	priv->is_plugged = !priv->is_plugged;
#ifdef CONFIG_USB
#ifdef CONFIG_QUARK_SE_SWITCH_INTERNAL_OSCILLATOR
	set_oscillator(priv->is_plugged ? OSC_EXTERNAL:OSC_INTERNAL);
#endif
	enable_vusb_regulator(priv, priv->is_plugged);
#endif
	/* Call user callbacks */
	list_foreach(&priv->cb_head, call_user_callback, (void*)priv->is_plugged);
}

#ifdef CONFIG_SOC_GPIO_AON
static void usb_aon_callback(bool state, void *priv_data)
{
	gpio_cfg_data_t config;
	struct device *dev = (struct device*)priv_data;
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;

	usb_generic_callback(dev);

	// Configure AON as USB wake up source
	config.gpio_type = GPIO_INTERRUPT;
	config.int_type = LEVEL;
	config.int_polarity = priv->is_plugged ? ACTIVE_LOW : ACTIVE_HIGH;
	config.int_debounce = DEBOUNCE_ON;
	config.int_ls_sync = LS_SYNC_OFF;
	config.gpio_cb = usb_aon_callback;
	config.gpio_cb_arg = priv_data;

	soc_gpio_deconfig(priv->evt_dev, priv->source_pin);
	int ret = soc_gpio_set_config(priv->evt_dev, priv->source_pin, &config);
	if (ret != 0) {
		pr_error(LOG_MODULE_DRV, "usb_pm: cb config failed (%d)", ret);
	}
}
#endif

#ifdef CONFIG_SOC_COMPARATOR
static void usb_comp_callback(void *priv_data)
{
	struct device *dev = (struct device*)priv_data;
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;

	usb_generic_callback(dev);

	// Configure comparator as USB wake up source
	int ret = comp_configure(priv->evt_dev, priv->source_pin,
			priv->is_plugged ? 1:0, 1, usb_comp_callback, priv_data);

	if (ret != 0) {
		pr_error(LOG_MODULE_DRV, "usb_pm: cb config failed (%d)", ret);
	}
}
#endif

static int usb_pm_init(struct device* dev)
{
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;

	priv->is_plugged = false;
	list_init(&priv->cb_head);

	priv->evt_dev = get_device(priv->evt_dev_id);
	if (!priv->evt_dev) {
		return -EINVAL;
	}
	return usb_pm_resume(dev);
}

bool usb_pm_is_plugged(struct device *dev)
{
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;
	return priv->is_plugged;
}

#ifdef CONFIG_USB_PM_SUSPEND_BLOCKERS
bool usb_pm_is_suspend_allowed(struct device *dev, PM_POWERSTATE state)
{
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;
	if ((state > PM_SHUTDOWN) && priv->is_plugged) {
		pr_debug(LOG_MODULE_DRV, "usb_pm %d: USB plugged", dev->id);
		return false;
	}
	return true;
}
#endif

int usb_pm_register_callback(struct device *dev, void (*cb)(bool, void*), void *priv)
{
	struct usb_pm_info *usb_dev = (struct usb_pm_info*)dev->priv;

	// Alloc memory for cb and priv argument
	struct usb_pm_cb_list *item = (struct usb_pm_cb_list*)balloc(sizeof(struct usb_pm_cb_list), NULL);
	// Fill new allocated item
	item->priv = priv;
	item->cb = cb;
	// Add item in list
	list_add_head(&usb_dev->cb_head, (list_t*)item);
	return 0;
}

static bool check_item_callback(list_t *item, void *cb)
{
	return ((struct usb_pm_cb_list*)item)->cb == cb;
}

int usb_pm_unregister_callback(struct device *dev, void (*cb)(bool, void*))
{
	struct usb_pm_info *usb_dev = (struct usb_pm_info*)dev->priv;

	list_t *element = list_find_first(&usb_dev->cb_head, check_item_callback, (void*)cb);

	if (element == NULL) {
		// element not found
		return -1;
	}
	// Remove list element
	list_remove(&usb_dev->cb_head, element);
	bfree(element);

	return 0;
}

static int usb_pm_suspend(struct device* dev, PM_POWERSTATE state)
{
	struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;
	if (priv->is_plugged) {
		pr_warning(LOG_MODULE_DRV, "dev %d: suspend while USB is plugged", dev->id);
	}
	return 0;
}

static int usb_pm_resume(struct device* dev)
{
#ifdef CONFIG_SOC_GPIO_AON
	if (((struct usb_pm_info*)(dev->priv))->interrupt_source == USB_AON_IRQ_SOURCE) {
		int ret = 0;
		struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;
		// Configure AON as USB wake up source
		gpio_cfg_data_t config;
		config.gpio_type = GPIO_INTERRUPT;
		config.int_type = LEVEL;
		config.int_polarity = priv->is_plugged ? ACTIVE_LOW : ACTIVE_HIGH;
		config.int_debounce = DEBOUNCE_ON;
		config.int_ls_sync = LS_SYNC_OFF;
		config.gpio_cb = usb_aon_callback;
		config.gpio_cb_arg = dev;
		soc_gpio_deconfig(priv->evt_dev, priv->source_pin);
		ret = soc_gpio_set_config(priv->evt_dev, priv->source_pin, &config);

		pr_debug(LOG_MODULE_DRV, "device %d: use aon %d - %d (%d)", dev->id,
				priv->interrupt_source, priv->source_pin, ret);
		return ret;
	}
#endif
#ifdef CONFIG_SOC_COMPARATOR
	if (((struct usb_pm_info*)(dev->priv))->interrupt_source == USB_COMPARATOR_IRQ_SOURCE) {
		int ret = 0;
		struct usb_pm_info *priv = (struct usb_pm_info*)dev->priv;
		// Configure comparator as USB wake up source
		ret = comp_configure(priv->evt_dev, priv->source_pin,
					priv->is_plugged ? 1:0, 1, usb_comp_callback, dev);
		pr_debug(LOG_MODULE_DRV, "device %s: use comp %d - %d (%d)", dev->id,
				priv->interrupt_source, priv->source_pin, ret);
		return ret;
	}
#endif

	pr_info(LOG_MODULE_DRV, "%d: no irq source", dev->id);
	return -1;
}

struct driver usb_pm_driver = {
	.init = usb_pm_init,
	.suspend = usb_pm_suspend,
	.resume = usb_pm_resume
};
