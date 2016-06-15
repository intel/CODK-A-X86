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

#ifndef __SOC_COMPARATOR_H__
#define __SOC_COMPARATOR_H__

#include "drivers/data_type.h"
#include "infra/device.h"



/**
 * @defgroup soc_comparator Quark_SE SOC Analog Comparator
 * Quark_SE SOC Analog Comparator driver API.
 *
 * @ingroup common_drivers
 * @{
 */

/**
 * Structure containing callback (and data) called at comparator interruption
 */
struct cmp_cb {
	void (*cb) (void *); /*!< Pointer to callback function */
	void *cb_data; /*!< Pointer to callback data */
};

/**
 * SOC Comparator driver.
 */

extern struct driver soc_comparator_driver;

/**
 * Configure a comparator input.
 *
 * When the comparator is configured, if the level of the pin triggers
 * the comparator, an interrupt is generated and the configured callback
 * is called by the driver in the context of interrupt.
 * As the comparator is level based and not edge based, the comparator is
 * disabled in the driver isr. In order to be able to get another interrupt
 * the user of the driver shall call comp_configure again.
 *
 * @param dev      pointer of the device
 * @param index    the index of the comparator
 * @param polarity the polarity of the comparator
 * @param refsel   the voltage reference selection
 *     - 0 for ref_a (external tension on AREF_PAD)
 *     - 1 for ref_b (internal tension VRef_OUT, between 0.98V and 1.2V)
 * @param cb       the callback function called when the interrupt is triggered
 * @param cb_data  the data passed to the callback function
 * @return DRV_RC_OK in case of success
 */
DRIVER_API_RC comp_configure(struct device *dev, int index, int polarity, int refsel,
			     void (*cb) (void *), void *cb_data);

/**
 * disable a comparator
 *
 * @param index the index of the comparator to disable.
 */
DRIVER_API_RC comp_disable(int index);

/** @} */

#endif /* __SOC_COMPARATOR_H__ */
