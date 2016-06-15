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
/* qrk_se_config.c - Qrk_SE Configuration. */


#include <device.h>
#include <init.h>
#include "board.h"

#ifdef CONFIG_DW_ADC

#include <adc.h>
#include <dw_adc.h>

struct adc_info adc_info_dev =
	{
		.rx_len = 0,
		.seq_size = 1,
		.state = ADC_STATE_IDLE
	};

struct adc_config adc_config_dev =
	{
		.reg_base = PERIPH_ADDR_BASE_ADC,
		.reg_irq_mask = SCSS_REGISTER_BASE + INT_SS_ADC_IRQ_MASK,
		.reg_err_mask = SCSS_REGISTER_BASE + INT_SS_ADC_ERR_MASK,
		.rx_vector = IO_ADC0_INT_IRQ,
		.err_vector = IO_ADC0_INT_ERR,
		.fifo_tld = IO_ADC0_FS/2,
		.in_mode      = CONFIG_ADC_INPUT_MODE,
		.out_mode     = CONFIG_ADC_OUTPUT_MODE,
		.capture_mode = CONFIG_ADC_CAPTURE_MODE,
		.seq_mode     = CONFIG_ADC_SEQ_MODE,
		.sample_width = CONFIG_ADC_WIDTH,
		.clock_ratio  = CONFIG_ADC_CLOCK_RATIO,
		.serial_dly   = CONFIG_ADC_SERIAL_DELAY
	};

DECLARE_DEVICE_INIT_CONFIG(adc,		/* config name*/
			ADC_DRV_NAME,	/* driver name*/
			&dw_adc_init,	/* init function*/
			&adc_config_dev); /* config options*/

pure_init(adc, &adc_info_dev);

#endif /* CONFIG_DW_ADC */
