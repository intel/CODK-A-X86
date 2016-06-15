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

#include <nanokernel.h>
#include <arch/cpu.h>
#include "dw_wdt.h"

void (*cb_fn)(void);

/**
 * Enables the clock for the peripheral watchdog
 */
static void dw_wdt_enable(void)
{
	SCSS_PERIPHERAL->periph_cfg0 |= SCSS_PERIPH_CFG0_WDT_ENABLE;
}


static void dw_wdt_disable(void)
{
	/* Disable the clock for the peripheral watchdog */
	SCSS_PERIPHERAL->periph_cfg0 &= ~SCSS_PERIPH_CFG0_WDT_ENABLE;
}


void dw_wdt_isr(void)
{
	if (cb_fn)
	{
		(*cb_fn)();
	}
}


static void dw_wdt_get_config(struct wdt_config *config)
{
}

IRQ_CONNECT_STATIC(dw_wdt, INT_WDT_IRQ, 0, dw_wdt_isr, 0);

static void dw_wdt_reload(void) {
	DW_WDT->wdt_crr = WDT_CRR_VAL;
}

static int dw_wdt_set_config(struct wdt_config *config)
{
	int ret = 0;

	dw_wdt_enable();
	/*  Set timeout value
	 *  [7:4] TOP_INIT - the initial timeout value is hardcoded in silicon,
	 *  only bits [3:0] TOP are relevant.
	 *  Once tickled TOP is loaded at the next expiration.
	 */
	uint32_t i;
	uint32_t ref = (1 << 16) / (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / 1000000); /* 2^16/FREQ_CPU */
	uint32_t timeout = config->timeout * 1000;
	for(i = 0; i < 16; i++){
		if (timeout <= ref) break;
		ref = ref << 1;
	}
	if (i > 15){
		ret = -1;
		i = 15;
	}
	DW_WDT->wdt_torr = i;

	/* Set response mode */
	if (DW_WDT_MODE_RESET == config->mode) {
		DW_WDT->wdt_cr &= ~WDT_CR_INT_ENABLE;
	} else {
		if (config->interrupt_fn)
		{
			cb_fn = config->interrupt_fn;
		} else {
			return -1;
		}

		DW_WDT->wdt_cr |= WDT_CR_INT_ENABLE;

		IRQ_CONFIG(dw_wdt, INT_WDT_IRQ);
		irq_enable(INT_WDT_IRQ);

		/* unmask WDT interrupts to qrk  */
		SCSS_INTERRUPT->int_watchdog_mask &= INT_UNMASK_IA;
	}

	/* Enable WDT, cannot be disabled until soc reset */
	DW_WDT->wdt_cr |= WDT_CR_ENABLE;

	dw_wdt_reload();
	return ret;
}


#if 0

static uint32_t dw_wdt_read_counter(void)
{
	return DW_WDT->wdt_ccvr;
}

static uint32_t dw_wdt_timeout(void)
{
	return DW_WDT->wdt_torr;
}

#endif

static struct wdt_driver_api dw_wdt_funcs = {
        .set_config = dw_wdt_set_config,
        .get_config = dw_wdt_get_config,
        .enable = dw_wdt_enable,
        .disable = dw_wdt_disable,
        .reload = dw_wdt_reload,
};

int dw_wdt_init(struct device *dev)
{
	dev->driver_api = &dw_wdt_funcs;
	return 0;
}

