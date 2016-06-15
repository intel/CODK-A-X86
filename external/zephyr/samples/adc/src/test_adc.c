/* test_adc.c - ADC test */

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#include <device.h>
#include <adc.h>

#define MIN_CH          0
#define MAX_CH          18
#define CH_PARAM        2
#define TEST_DLY        50
#define TEST_RESOLUTION 12
#define TEST_CLK_RATIO  1024
#define LENGTH          80

struct adc_test_ctx {
	uint32_t channel;
	uint32_t data;
} adc_ctx;

static void rx_cbk(struct device *dev);
static void err_cbk(struct device *dev);

/**
 * @Brief Callback functions
 */
static void rx_cbk(struct device *dev)
{
	PRINT("ADC Sampling: %d %d", adc_ctx.channel, adc_ctx.data);
	adc_disable(dev);
	adc_unlock(dev);
}

static void err_cbk(struct device *dev)
{
	PRINT("ADC Sampling Error.");
	adc_disable(dev);
	adc_unlock(dev);
}


/**
 * @Brief Test command to get the channel 0 value.
 *
 */
void adc_get()
{
	struct device *dev;
	struct io_adc_seq_table seq_tbl;
	struct io_adc_seq_entry entrys;
	uint32_t data_len = 1;
	adc_ctx.channel = 0;
	adc_ctx.data = 0;
	entrys.channel_id = adc_ctx.channel;
	entrys.sample_dly = TEST_DLY;

	dev = device_get_binding(ADC_DRV_NAME);

	if(!dev)
	{
		PRINT("The device ADC does not exist.");
		return;
	}

	if ( adc_lock(dev)) {
		PRINT("Error: ADC in use.");
		return;
	}

	adc_set_cb(dev, rx_cbk, err_cbk);

	if (adc_ctx.channel >= MIN_CH && adc_ctx.channel <= MAX_CH) {
		adc_enable(dev);
		seq_tbl.entries = &entrys;
		seq_tbl.num_entries = data_len;
		if( adc_read(dev, &seq_tbl, &(adc_ctx.data), data_len) )
		{
			PRINT("Read status: OK");
			goto exit_disable;
		}
		else
		{
			PRINT("Invalid Channel %d", adc_ctx.channel);
		}
		goto exit_unlock;
	}

	return;

exit_disable:
	adc_disable(dev);

exit_unlock:
	adc_unlock(dev);
	return;
}

#define STACKSIZE 2000

char __stack fiberStack[STACKSIZE];

void fiberEntry(void)
{
	adc_get();
}

void main(void)
{
	task_fiber_start(&fiberStack[0], STACKSIZE,
			(nano_fiber_entry_t) fiberEntry, 0, 0, 7, 0);

	PRINT("%s: ADC Demo\n", __FUNCTION__);
}
