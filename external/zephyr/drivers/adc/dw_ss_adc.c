/* ==========================================================================
* Synopsys DesignWare Sensor and Control IP Subsystem IO Software Driver and
* documentation (hereinafter, "Software") is an Unsupported proprietary work
* of Synopsys, Inc. unless otherwise expressly agreed to in writing between
* Synopsys and you.
*
* The Software IS NOT an item of Licensed Software or Licensed Product under
* any End User Software License Agreement or Agreement for Licensed Product
* with Synopsys or any supplement thereto. You are permitted to use and
* redistribute this Software in source and binary forms, with or without
* modification, provided that redistributions of source code must retain this
* notice. You may not view, use, disclose, copy or distribute this file or
* any information contained herein except pursuant to this license grant from
* Synopsys. If you do not agree with this notice, including the disclaimer
* below, then you are not authorized to use the Software.
*
* THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
* ========================================================================== */
#include <stddef.h>
#include <toolchain.h>
#include <v2/aux_regs.h>
#include "dw_adc.h"

void dw_ss_adc_rx_ISR_proc(struct device *dev)
{
	struct adc_config *config;
	struct device_config *dev_config;
	struct adc_info   *info;
	uint32_t adc_base;
	uint32_t  i, reg_val, rx_cnt;
	uint32_t  rd = 0;
	uint32_t  idx;

	dev_config = dev->config;
	config = dev_config->config_info;
	info   = dev->driver_data;
	adc_base = config->reg_base;
	idx = info->index;

	if (IO_ADC_SEQ_MODE_REPETITIVE == config->seq_mode)
	{
		if (NULL == info->rx_buf[idx])
		{
			goto cli;
		}
		rx_cnt = (config->fifo_tld + 1);
	}
	else
	{
		rx_cnt = info->seq_size;
	}

	if (rx_cnt > info->rx_len)
	{
		rx_cnt = info->rx_len;
	}

	for (i = 0; i < rx_cnt; i++)
	{
		reg_val = _arc_v2_aux_reg_read( adc_base + ADC_SET );
		_arc_v2_aux_reg_write( adc_base + ADC_SET,
			reg_val|ADC_POP_SAMPLE);
		rd = _arc_v2_aux_reg_read( adc_base + ADC_SAMPLE );
		info->rx_buf[idx][i]  = rd;
	}

	info->rx_buf[idx] += i;
	info->rx_len -= i;

	if (0 == info->rx_len)
	{
		if (likely( NULL != info->rx_cb))
		{
			info->rx_cb(dev);
		}
		if (IO_ADC_SEQ_MODE_SINGLESHOT == config->seq_mode)
		{
			_arc_v2_aux_reg_write( adc_base + ADC_CTRL,
				ADC_INT_DSB|ADC_CLK_ENABLE|ADC_SEQ_PTR_RST);
			reg_val = _arc_v2_aux_reg_read( adc_base + ADC_SET );
			_arc_v2_aux_reg_write( adc_base + ADC_SET,
				reg_val | ADC_FLUSH_RX);
			info->state = ADC_STATE_IDLE;
			goto cli;
		}
		info->rx_buf[idx] = NULL;
		idx++;
		idx %= BUFS_NUM;
		info->index = idx;
	}
	else if (IO_ADC_SEQ_MODE_SINGLESHOT == config->seq_mode)
	{
		_arc_v2_aux_reg_write( adc_base + ADC_CTRL,
			ADC_INT_DSB|ADC_CLK_ENABLE|ADC_SEQ_PTR_RST);

		info->state = ADC_STATE_IDLE;
		if (likely( NULL != info->rx_cb))
		{
			info->rx_cb(dev);
		}

	}
cli:
	reg_val = _arc_v2_aux_reg_read( adc_base + ADC_CTRL );
	_arc_v2_aux_reg_write( adc_base + ADC_CTRL,
		reg_val | ADC_CLR_DATA_A);

}


void dw_ss_adc_err_ISR_proc(struct device *dev)
{
	struct adc_config  *config = dev->config->config_info;
	struct adc_info    *info   = dev->driver_data;
	uint32_t adc_base = config->reg_base;
	uint32_t reg_val = _arc_v2_aux_reg_read(adc_base + ADC_SET);

	_arc_v2_aux_reg_write( adc_base + ADC_CTRL,
		ADC_INT_DSB|ADC_CLK_ENABLE|ADC_SEQ_PTR_RST);
	_arc_v2_aux_reg_write( adc_base + ADC_CTRL,
		reg_val | ADC_FLUSH_RX);

	info->state = ADC_STATE_IDLE;

	_arc_v2_aux_reg_write( adc_base +ADC_CTRL,
		ADC_INT_DSB|ADC_CLK_ENABLE|ADC_CLR_OVERFLOW|ADC_CLR_UNDRFLOW);

	if (likely( NULL != info->err_cb))
	{
		info->err_cb( dev );
	}
}
