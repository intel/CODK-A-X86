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
/* pinmux.c - general pinmux operation */

#include <device.h>
#include <pinmux.h>
#include "pinmux.h"


extern struct pin_config mux_config[];

static void _pinmux_set(uint32_t base, uint32_t pin, uint32_t mode)
{
	/*
	 * the registers are 32-bit wide, but each pin requires 2 bits
	 * to set the mode (A, B, C, or D).  As such we only get 16
	 * pins per register... hence the math for the register mask.
	 */
	uint32_t register_offset = (pin / 16) * 4;
	/*
	 * Now figure out what is the full address for the register
	 * we are looking for.  Add the base register to the register_mask
	 */
	volatile uint32_t *mux_register = (uint32_t*)(base + register_offset);

	/*
	 * Finally grab the pin offset within the register
	 */
	uint32_t pin_no = pin % 16;

	/*
	 * The value 3 is used because that is 2-bits for the mode of each
	 * pin.  The value 2 repesents the bits needed for each pin's mode.
	 */
	uint32_t pin_mask = 3 << (pin_no * 2);
	uint32_t mode_mask = mode << (pin_no * 2);
	/* (*((volatile uint32_t *)mux_address)) = ((*mux_address) & */
	(*(mux_register)) = ((*(mux_register)) & ~pin_mask) | mode_mask;
}


static uint32_t _pinmux_get(uint32_t base, uint32_t pin, uint32_t mode)
{
	/*
	 * the registers are 32-bit wide, but each pin requires 2 bits
	 * to set the mode (A, B, C, or D).  As such we only get 16
	 * pins per register... hence the math for the register mask.
	 */
	uint32_t register_offset = (pin / 16) * 4;
	/*
	 * Now figure out what is the full address for the register
	 * we are looking for.  Add the base register to the register_mask
	 */
	volatile uint32_t *mux_register = (uint32_t*)(base + register_offset);

	/*
	 * Finally grab the pin offset within the register
	 */
	uint32_t pin_no = pin % 16;

	/*
	 * The value 3 is used because that is 2-bits for the mode of each
	 * pin.  The value 2 repesents the bits needed for each pin's mode.
	 */
	uint32_t pin_mask = 3 << (pin_no * 2);
	uint32_t mode_mask = mode << (pin_no * 2);
	/* (*((volatile uint32_t *)mux_address)) = ((*mux_address) & */
	return ( ((*(mux_register)) & ~pin_mask) | mode_mask );
}


static uint32_t pinmux_dev_set(struct device *dev, uint32_t pin, uint8_t func)
{
	struct pinmux_config * const pmux = dev->config->config_info;

	_pinmux_set(pmux->base_address, pin, func);

	return 0;
}


static uint32_t pinmux_dev_get(struct device *dev, uint32_t pin, uint8_t *func)
{
	struct pinmux_config * const pmux = dev->config->config_info;
	uint32_t ret;

	ret = _pinmux_get(pmux->base_address, pin, *func);

	*func = ret;
	return 0;
}


static struct pinmux_driver_api api_funcs = {
	.set = pinmux_dev_set,
	.get = pinmux_dev_get
};


int pinmux_initialize(struct device *dev)
{
	struct device_config *dev_cfg = dev->config;
	struct pinmux_config *pmux = dev_cfg->config_info;
	int i;

	dev->driver_api = &api_funcs;

	for (i = 0; i < CONFIG_PINMUX_NUM_PINS; i++) {
		_pinmux_set(pmux->base_address,
			    mux_config[i].pin_num,
			    mux_config[i].mode);
	}

	return PINMUX_OK;
}
