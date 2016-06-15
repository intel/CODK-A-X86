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

#ifndef _DW_AIO_COMPARATOR_H_
#define _DW_AIO_COMPARATOR_H_

#include <board.h>
#include <device.h>
#include <aio_comparator.h>

#define DW_AIO_CMP_DRV_NAME		"dw_aio_cmp"

/**
 * @brief Number of AIO/Comparator on board
 */
#define DW_AIO_CMP_COUNT		(19)

/**
 * AIO/Comparator Register block type.
 */
struct dw_aio_cmp_t {
	volatile uint32_t en;		/**< Enable Register (0x00) */
	volatile uint32_t ref_sel;	/**< Reference Selection Register (0x04) */
	volatile uint32_t ref_pol;	/**< Reference Polarity Register (0x08) */
	volatile uint32_t pwr;		/**< Power Register (0x0C) */
	uint32_t reversed[6];
	volatile uint32_t stat_clr;	/**< Status Clear Register (0x28) */
};

struct dw_aio_cmp_cb
{
	aio_cmp_cb cb;
	void *param;
};

struct dw_aio_cmp_dev_cfg_t {
	/** Base register address */
	uint32_t base_address;

	/** Interrupt number */
	uint32_t interrupt_num;

	/** Config function */
	int (*config_func)(struct device *dev);
};

struct dw_aio_cmp_dev_data_t {
	/** Number of total comparators */
	uint8_t num_cmp;

	/** Callback for each comparator */
	struct dw_aio_cmp_cb cb[DW_AIO_CMP_COUNT];
};

extern int dw_aio_cmp_init(struct device* dev);

#endif /* _DW_AIO_COMPARATOR_H_ */
