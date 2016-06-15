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

#include <nanokernel.h>
#include <device.h>
#include <aio_comparator.h>

#include "dw_aio_comparator.h"

#define INT_COMPARATORS_MASK	0x7FFFF

static int dw_aio_cmp_disable(struct device *dev, uint8_t index)
{
	struct dw_aio_cmp_dev_cfg_t *config =
	    (struct dw_aio_cmp_dev_cfg_t *)dev->config->config_info;
	struct dw_aio_cmp_t *regs =
	    (struct dw_aio_cmp_t *)config->base_address;

	if (index >= DW_AIO_CMP_COUNT) {
		return DEV_INVALID_CONF;
	}

	/* Disable interrupt to host */
	SCSS_INTERRUPT->int_comparators_host_mask |= (1 << index);

	/* Disable comparator <index> */
	regs->en &= ~(1 << index);
	/* Disable power in comparator <index> */
	regs->pwr &= ~(1 << index);

	return DEV_OK;
}

static int dw_aio_cmp_configure(struct device *dev, uint8_t index,
			 enum aio_cmp_polarity polarity,
			 enum aio_cmp_ref refsel,
			 aio_cmp_cb cb, void *param)
{
	struct dw_aio_cmp_dev_cfg_t *config =
	    (struct dw_aio_cmp_dev_cfg_t *)dev->config->config_info;
	struct dw_aio_cmp_dev_data_t *dev_data =
	    (struct dw_aio_cmp_dev_data_t *)dev->driver_data;
	struct dw_aio_cmp_t *regs =
	    (struct dw_aio_cmp_t *)config->base_address;
	uint32_t reg_val;

	/* index out of range */
	if (index >= DW_AIO_CMP_COUNT)
		return DEV_INVALID_CONF;

	/* make sure reference makes sense */
	if ((refsel != AIO_CMP_REF_A) && (refsel != AIO_CMP_REF_B))
		return DEV_INVALID_CONF;

	/* make sure polarity makes sense */
	if ((polarity != AIO_CMP_POL_RISE) && (polarity != AIO_CMP_POL_FALL))
		return DEV_INVALID_CONF;

	dev_data->cb[index].cb = cb;
	dev_data->cb[index].param = param;

	/* Disable interrupt to host */
	SCSS_INTERRUPT->int_comparators_host_mask |= (1 << index);

	/* Disable comparator <index> before config */
	regs->en &= ~(1 << index);
	regs->pwr &= ~(1 << index);

	/**
	 * Setup reference voltage source
	 * REF_A: bit ==> 0, REF_B: bit ==> 1
	 */
	reg_val = regs->ref_sel;
	if (refsel == AIO_CMP_REF_A)
		reg_val &= ~(1 << index);
	else
		reg_val |= (1 << index);
	regs->ref_sel = reg_val;

	/**
	 * Setup reference polarity
	 * RISING: bit ==> 0, FALLING: bit ==> 1
	 */
	reg_val = regs->ref_pol;
	if (polarity == AIO_CMP_POL_RISE)
		reg_val &= ~(1 << index);
	else
		reg_val |= (1 << index);
	regs->ref_pol = reg_val;

	/* Enable power of comparator <index> */
	regs->pwr |= (1 << index);

	/* Enable comparator <index> */
	regs->en |= (1 << index);

	/* Enable interrupt to host */
	SCSS_INTERRUPT->int_comparators_host_mask &= ~(1 << index);

	return DEV_OK;
}

void dw_aio_cmp_isr(struct device *dev)
{
	struct dw_aio_cmp_dev_cfg_t *config =
	    (struct dw_aio_cmp_dev_cfg_t *)dev->config->config_info;
	struct dw_aio_cmp_dev_data_t *dev_data =
	    (struct dw_aio_cmp_dev_data_t *)dev->driver_data;
	struct dw_aio_cmp_t *regs =
	    (struct dw_aio_cmp_t *)config->base_address;
	int i;
	int reg_stat_clr;

	reg_stat_clr = regs->stat_clr;
	for (i = 0; i < dev_data->num_cmp; i++) {
		if (reg_stat_clr & (1 << i)) {
			dw_aio_cmp_disable(dev, i);
			if (dev_data->cb[i].cb != NULL) {
			    dev_data->cb[i].cb(dev_data->cb[i].param);
			}
		}
	}

	/* Clear interrupt status by writing 1s */
	regs->stat_clr = reg_stat_clr;
}

static struct aio_cmp_driver_api dw_aio_cmp_funcs = {
	.disable = dw_aio_cmp_disable,
	.configure = dw_aio_cmp_configure,
};

int dw_aio_cmp_init(struct device* dev)
{
	struct dw_aio_cmp_dev_cfg_t *config =
	    (struct dw_aio_cmp_dev_cfg_t *)dev->config->config_info;
	struct dw_aio_cmp_dev_data_t *dev_data =
	    (struct dw_aio_cmp_dev_data_t *)dev->driver_data;
	struct dw_aio_cmp_t *regs =
	    (struct dw_aio_cmp_t *)config->base_address;
	int i;

	if ((config->base_address == 0) || (config->interrupt_num == 0))
		return DEV_INVALID_CONF;

	if (config->config_func) {
		i =  config->config_func(dev);
		if (i != DEV_OK)
			return i;
	}

	dev->driver_api = &dw_aio_cmp_funcs;

	/* Clear host interrupt mask */
	SCSS_INTERRUPT->int_comparators_host_mask |= INT_COMPARATORS_MASK;

	/* Clear comparator interrupt status */
	regs->stat_clr |= INT_COMPARATORS_MASK;

	/* Disable all comparators */
	regs->en &= ~INT_COMPARATORS_MASK;

	/* Power down all comparators */
	regs->pwr &= ~INT_COMPARATORS_MASK;

	/* Clear callback pointers */
	for (i = 0; i < dev_data->num_cmp; i++) {
		dev_data->cb[i].cb = NULL;
		dev_data->cb[i].param = NULL;
	}

	irq_enable(config->interrupt_num);

	return DEV_OK;
}
