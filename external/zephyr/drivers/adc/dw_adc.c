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
/* dw_adc.c - Designware ADC driver */


#include <nanokernel.h>
#include <stdlib.h>
#include <board.h>
#include <adc.h>
#include <arch/cpu.h>
#include <v2/aux_regs.h>
#include "dw_adc.h"
#include "dw_ss_adc.h"

#define ADC_CLOCK_GATE      (1 << 31)
#define ADC_STANDBY          0x02
#define ADC_NORMAL_WO_CALIB  0x04
#define ADC_MODE_MASK        0x07

#define ONE_BIT_SET     0x1
#define FIVE_BITS_SET   0x1f
#define SIX_BITS_SET    0x3f
#define ELEVEN_BITS_SET 0x7ff

#define INPUT_MODE_POS     5
#define CAPTURE_MODE_POS   6
#define OUTPUT_MODE_POS    7
#define SERIAL_DELAY_POS   8
#define SEQUENCE_MODE_POS  13
#define SEQ_ENTRIES_POS    16
#define THRESHOLD_POS      24

#define SEQ_DELAY_EVEN_POS 5
#define SEQ_MUX_ODD_POS    16
#define SEQ_DELAY_ODD_POS  21

#define ADC_INT_PRIORITY    0

static uint8_t adc_in_use = 0;

static struct device *adc_dev;

static void adc_goto_normal_mode_wo_calibration(void)
{
	uint32_t reg_value;
	uint32_t state;

	reg_value = _arc_v2_aux_reg_read(
		PERIPH_ADDR_BASE_CREG_SLV0 + SLV_OBSR);

	if ((reg_value & ADC_MODE_MASK) != ADC_NORMAL_WO_CALIB)
	{
		state = irq_lock();
		reg_value = _arc_v2_aux_reg_read(
			PERIPH_ADDR_BASE_CREG_MST0 + MST_CTL);

		reg_value &= ~(ADC_MODE_MASK);
		reg_value |= ADC_STANDBY | ADC_CLOCK_GATE;

		_arc_v2_aux_reg_write(
			PERIPH_ADDR_BASE_CREG_MST0 + MST_CTL, reg_value);
		irq_unlock(state);

		do{
			reg_value = _arc_v2_aux_reg_read(
				PERIPH_ADDR_BASE_CREG_SLV0 + SLV_OBSR) & 0x8;
		}
		while (reg_value == 0);

		state = irq_lock();
		reg_value = _arc_v2_aux_reg_read(
			PERIPH_ADDR_BASE_CREG_MST0 + MST_CTL);

		reg_value &= ~(ADC_MODE_MASK);
		reg_value |= ADC_NORMAL_WO_CALIB | ADC_CLOCK_GATE;

		_arc_v2_aux_reg_write(
			PERIPH_ADDR_BASE_CREG_MST0 + MST_CTL, reg_value);
		irq_unlock(state);

		do{
			reg_value = _arc_v2_aux_reg_read(
				PERIPH_ADDR_BASE_CREG_SLV0 + SLV_OBSR) & 0x8;
		}
		while (reg_value == 0);
	}
}

static void adc_goto_deep_power_down(void)
{
	uint32_t reg_value;
	uint32_t state;

	reg_value = _arc_v2_aux_reg_read(
		PERIPH_ADDR_BASE_CREG_SLV0+SLV_OBSR);
	if ((reg_value & ADC_MODE_MASK) != 0)
	{
		state = irq_lock();

		reg_value = _arc_v2_aux_reg_read(
			PERIPH_ADDR_BASE_CREG_MST0 + MST_CTL);

		reg_value &= ~(ADC_MODE_MASK);

		reg_value |= 0 | ADC_CLOCK_GATE;
		_arc_v2_aux_reg_write(
			PERIPH_ADDR_BASE_CREG_MST0 + MST_CTL, reg_value);

		irq_unlock(state);
		do
		{
			reg_value =_arc_v2_aux_reg_read(
				PERIPH_ADDR_BASE_CREG_SLV0 + SLV_OBSR ) & 0x8;
		}
		while (reg_value == 0);
	}
}

static void adc_rx_isr()
{
	dw_ss_adc_rx_ISR_proc(adc_dev);
}

static void adc_err_isr()
{
	dw_ss_adc_err_ISR_proc(adc_dev);
}

static uint8_t dw_adc_lock(void)
{
	uint32_t saved = irq_lock();
	if (adc_in_use)
	{
		irq_unlock(saved);
		return 1;
	}
	adc_in_use = 1;
	irq_unlock(saved);
	return 0;
}

static void dw_adc_unlock(void)
{
	adc_in_use = 0;
}

static void dw_adc_enable(struct device *dev)
{
	struct adc_info *info = dev->driver_data;
	struct adc_config *config = dev->config->config_info;
	adc_goto_normal_mode_wo_calibration( );
	_arc_v2_aux_reg_write( config->reg_base + ADC_CTRL,
		ADC_INT_ENABLE | ADC_CLK_ENABLE | ADC_SEQ_TABLE_RST );
	info->state = ADC_STATE_IDLE;
}

static void dw_adc_disable(struct device *dev)
{
	uint32_t saved;
	struct adc_info *info = dev->driver_data;
	struct adc_config *config = dev->config->config_info;

	adc_goto_deep_power_down();
	_arc_v2_aux_reg_write( config->reg_base + ADC_CTRL,
		ADC_INT_DSB|ADC_SEQ_PTR_RST );

	saved = irq_lock();
	info->err_cb = NULL;
	info->rx_cb = NULL;

	_arc_v2_aux_reg_write( config->reg_base + ADC_SET,
		_arc_v2_aux_reg_read( config->reg_base+ADC_SET )|ADC_FLUSH_RX);
	irq_unlock(saved);

	info->state = ADC_STATE_DISABLED;
}

static uint8_t dw_adc_read(struct device *dev, struct io_adc_seq_table *seq_tbl,
		uint32_t *data, uint32_t data_len)
{
	uint32_t i = 0;
	uint32_t ctrl = 0;
	uint32_t tmp_val = 0;
	uint32_t num_iters = 0;
	uint32_t saved;
	struct io_adc_seq_entry *entry = NULL;
	struct adc_info *info = dev->driver_data;
	struct adc_config *config = dev->config->config_info;
	uint32_t adc_base = config->reg_base;

	if (info->state != ADC_STATE_IDLE)
	{
		return 1;
	}

	saved = irq_lock();

	ctrl = _arc_v2_aux_reg_read(adc_base + ADC_CTRL);
	ctrl |= ADC_SEQ_PTR_RST;
	_arc_v2_aux_reg_write((adc_base + ADC_CTRL), ctrl);

	info->seq_size = seq_tbl->num_entries;

	tmp_val = _arc_v2_aux_reg_read(adc_base + ADC_SET);
	tmp_val &= ADC_SEQ_SIZE_SET_MASK;
	tmp_val |= ( ((seq_tbl->num_entries - 1) & SIX_BITS_SET)
		<< SEQ_ENTRIES_POS );
	tmp_val |= ( (info->seq_size - 1) << THRESHOLD_POS );
	_arc_v2_aux_reg_write(adc_base + ADC_SET, tmp_val);

	irq_unlock(saved);

	num_iters = seq_tbl->num_entries/2;

	for (i = 0, entry = seq_tbl->entries;
		i < num_iters; i++, entry += 2)
	{
		tmp_val = ((entry[1].sample_dly & ELEVEN_BITS_SET)
			<< SEQ_DELAY_ODD_POS);
		tmp_val |= ((entry[1].channel_id & FIVE_BITS_SET)
			<< SEQ_MUX_ODD_POS);
		tmp_val |= ((entry[0].sample_dly & ELEVEN_BITS_SET)
			<< SEQ_DELAY_EVEN_POS);
		tmp_val |= (entry[0].channel_id & FIVE_BITS_SET);
		_arc_v2_aux_reg_write( adc_base + ADC_SEQ, tmp_val );
	}

	if ((seq_tbl->num_entries % 2) != 0)
	{
		tmp_val = ((entry[0].sample_dly & ELEVEN_BITS_SET)
			<< SEQ_DELAY_EVEN_POS);
		tmp_val |= (entry[0].channel_id & FIVE_BITS_SET);
		_arc_v2_aux_reg_write( adc_base + ADC_SEQ, tmp_val );
	}

	_arc_v2_aux_reg_write( adc_base + ADC_CTRL, ctrl | ADC_SEQ_PTR_RST );

	if (info->state == ADC_STATE_IDLE)
	{
		for( i = 0; i < BUFS_NUM; i++ )
		{
			info->rx_buf[i] = NULL;
		}
		info->rx_buf[0] = data;
		info->rx_len =  data_len;
		info->index = 0;
		info->state = ADC_STATE_SAMPLING;
		_arc_v2_aux_reg_write( adc_base	+ ADC_CTRL,
			ADC_SEQ_START|ADC_ENABLE|ADC_CLK_ENABLE );
	}
	else if (config->seq_mode == IO_ADC_SEQ_MODE_REPETITIVE)
	{
		uint32_t idx = info->index;

		if (info->rx_buf[idx] == NULL)
		{
			info->rx_buf[idx] = data;
			info->rx_len = data_len;
		}
	}
	return 0;
}

static void dw_set_user_callbacks(struct device *dev,
		adc_callback cb_rx, adc_callback cb_err)
{
	struct adc_info *info = dev->driver_data;
	info->err_cb = cb_err;
	info->rx_cb  = cb_rx;
}

static struct adc_driver_api api_funcs = {
	.enable  = dw_adc_enable,
	.disable = dw_adc_disable,
	.read    = dw_adc_read,
	.lock    = dw_adc_lock,
	.unlock  = dw_adc_unlock,
	.set_cb  = dw_set_user_callbacks
};

int dw_adc_init(struct device *dev)
{
	uint32_t tmp_val = 0;
	uint32_t val = 0;
	uint32_t ctrl = 0;
	struct adc_config *config = dev->config->config_info;
	uint32_t adc_base = config->reg_base;

	adc_dev = dev;
	dev->driver_api = &api_funcs;

	ctrl = ADC_INT_DSB|ADC_CLK_ENABLE;
	_arc_v2_aux_reg_write( ADC_CTRL, ctrl );

	tmp_val = _arc_v2_aux_reg_read( adc_base + ADC_SET );
	tmp_val &= ADC_CONFIG_SET_MASK;
	val = (config->sample_width) & FIVE_BITS_SET;
	val |= ((config->in_mode & ONE_BIT_SET)     << INPUT_MODE_POS);
	val |= ((config->capture_mode & ONE_BIT_SET)<< CAPTURE_MODE_POS);
	val |= ((config->out_mode & ONE_BIT_SET)    << OUTPUT_MODE_POS);
	val |= ((config->serial_dly & FIVE_BITS_SET)<< SERIAL_DELAY_POS);
	val |= ((config->seq_mode & ONE_BIT_SET)    << SEQUENCE_MODE_POS);
	_arc_v2_aux_reg_write( adc_base + ADC_SET, tmp_val|val );

	_arc_v2_aux_reg_write( adc_base + ADC_DIVSEQSTAT,
		config->clock_ratio & ADC_CLK_RATIO_MASK );

	_arc_v2_aux_reg_write( adc_base + ADC_CTRL,
		ADC_INT_ENABLE & ~(ADC_CLK_ENABLE) );

	irq_connect( config->rx_vector, ADC_INT_PRIORITY, adc_rx_isr, NULL );
	irq_enable( config->rx_vector );

	irq_connect( config->err_vector, ADC_INT_PRIORITY, adc_err_isr, NULL );
	irq_enable( config->err_vector );

	(*((volatile uint32_t *) config->reg_irq_mask ))
		&= ENABLE_SSS_INTERRUPTS;
	(*((volatile uint32_t *) config->reg_err_mask ))
		&= ENABLE_SSS_INTERRUPTS;

	return 0;
}
