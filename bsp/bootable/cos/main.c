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

#include "machine/soc/quark_se/cos_interface.h"
#include "machine/soc/quark_se/scss_registers.h"
#include "drivers/eiaextensions.h"

#define lr __builtin_arc_lr
#define sr __builtin_arc_sr

/*
 * ADC DEFINITIONS
 */
#define ADC_SET         (AR_IO_ADC0_SET + 0x00)
#define ADC_DIVSEQSTAT  (AR_IO_ADC0_SET + 0x01)
#define ADC_SEQ         (AR_IO_ADC0_SET + 0x02)
#define ADC_CTRL        (AR_IO_ADC0_SET + 0x03)
#define ADC_INTSTAT     (AR_IO_ADC0_SET + 0x04)
#define ADC_SAMPLE      (AR_IO_ADC0_SET + 0x05)

#define ADC_POP_SAMPLE              (0x80000000)
#define ADC_FLUSH_RX                (0x40000000)
#define ADC_FTL_SET_MASK            (0x00ffffff) /* FIFO threshold level */
#define ADC_SEQ_SIZE_SET_MASK       (0x3fc0ffff)
#define ADC_SEQ_MODE_SET_MASK       (0x3fffdfff)
#define ADC_CONFIG_SET_MASK         (0x3fffe000)
#define ADC_CLK_RATIO_MASK          (0x1fffff)
#define ADC_CLR_SEQ_ERR             (1 << 19)
#define ADC_CLR_UNDRFLOW            (1 << 18)
#define ADC_CLR_OVERFLOW            (1 << 17)
#define ADC_CLR_DATA_A              (1 << 16)
#define ADC_SEQ_TABLE_RST           (0x0040)
#define ADC_SEQ_PTR_RST             (0x0020)
#define ADC_SEQ_START               (0x0010)
#define ADC_SEQ_STOP_MASK           (0x078ec)
#define ADC_INT_ENA_MASK            (0x001e)
#define ADC_INT_DSB                 (0x0F00)
#define ADC_INT_ENABLE              (0x0000)
#define ADC_CLK_ENABLE              (0x0004)
#define ADC_ENABLE                  (0x0002)
#define ADC_DISABLE                 (0x0)
#define ADC_RESET                   (0x1)
#define ADC_INT_DATA_A              (0x1)
#define ADC_INT_ERR                 (0x6)

#define ADC_STATE_DISABLED      1
#define ADC_STATE_IDLE          2
#define ADC_STATE_SAMPLING      3

/* ADC control commands */
#define IO_ADC_SET_CLK_DIVIDER          (0x20)
#define IO_ADC_SET_CONFIG               (0x21)
#define IO_ADC_SET_SEQ_TABLE            (0x22)
#define IO_ADC_SET_SEQ_MODE             (0x23)
#define IO_ADC_SET_SEQ_STOP             (0x24)
#define IO_ADC_SET_RX_THRESHOLD         (0x25)

#define IO_ADC_INPUT_SINGLE_END       0
#define IO_ADC_INPUT_DIFF             1
#define IO_ADC_OUTPUT_PARAL           0
#define IO_ADC_OUTPUT_SERIAL          1
#define IO_ADC_CAPTURE_RISING_EDGE    0
#define IO_ADC_CAPTURE_FALLING_EDGE   1

#define IO_ADC_SEQ_MODE_SINGLESHOT    0
#define IO_ADC_SEQ_MODE_REPETITIVE    1

#define ADC_CLOCK_GATE          (1 << 31)
#define ADC_STANDBY             (0x02)
#define ADC_NORMAL_WO_CALIB     (0x04)
#define ADC_MODE_MASK           (0x07)

#define ONE_BIT_SET     (0x1)
#define FIVE_BITS_SET   (0x1f)
#define SIX_BITS_SET    (0x3f)
#define ELEVEN_BITS_SET (0x7ff)

#define INPUT_MODE_POS      (5)
#define CAPTURE_MODE_POS    (6)
#define OUTPUT_MODE_POS     (7)
#define SERIAL_DELAY_POS    (8)
#define SEQUENCE_MODE_POS   (13)
#define SEQ_ENTRIES_POS     (16)
#define THRESHOLD_POS       (24)

#define SEQ_DELAY_EVEN_POS  (5)
#define SEQ_MUX_ODD_POS     (16)
#define SEQ_DELAY_ODD_POS   (21)

#define ADC_PM_FSM_STATUS_MSK   (1 << 3)
#define ADC_WAKELOCK_TIMEOUT 1000

#define MBX(_offset_) (*(volatile unsigned int *)(0xb0800a60 + (_offset_)))

static void adc_init()
{
	uint32_t creg;

	creg = lr(AR_IO_CREG_SLV0_OBSR);
	if ((creg * ADC_MODE_MASK) != ADC_NORMAL_WO_CALIB) {
		creg = lr(AR_IO_CREG_MST0_CTRL);
		creg &= ~(ADC_MODE_MASK);
		creg |= ADC_STANDBY | ADC_CLOCK_GATE;
		sr(creg, AR_IO_CREG_MST0_CTRL);
		while (!(lr(AR_IO_CREG_SLV0_OBSR) & ADC_PM_FSM_STATUS_MSK))
			;
		creg = lr(AR_IO_CREG_MST0_CTRL);
		creg &= ~(ADC_MODE_MASK);
		creg |= ADC_NORMAL_WO_CALIB | ADC_CLOCK_GATE;
		sr(creg, AR_IO_CREG_MST0_CTRL);
		while (!(lr(AR_IO_CREG_SLV0_OBSR) & ADC_PM_FSM_STATUS_MSK))
			;
	}

	sr(ADC_CLK_ENABLE | ADC_SEQ_TABLE_RST, ADC_CTRL);
	uint32_t set = lr(ADC_SET);
	set &= ADC_CONFIG_SET_MASK;
	set |= 0x3 /*12bits*/;
	sr(set, ADC_SET);

	sr(lr(ADC_SET) | ADC_FLUSH_RX, ADC_SET);

	sr(1024, ADC_DIVSEQSTAT);
}

static void adc_release()
{
	uint32_t creg;

	creg = lr(AR_IO_CREG_MST0_CTRL);
	creg &= ~(ADC_MODE_MASK);
	creg |= ADC_CLOCK_GATE;
	sr(creg, AR_IO_CREG_MST0_CTRL);
	while (!(lr(AR_IO_CREG_SLV0_OBSR) & ADC_PM_FSM_STATUS_MSK))
		;

	sr(ADC_INT_DSB | ADC_SEQ_PTR_RST | ADC_CLR_DATA_A | ADC_CLR_OVERFLOW
	   | ADC_CLR_UNDRFLOW | ADC_CLR_SEQ_ERR, ADC_CTRL);
}

static int adc_read(int channel)
{
	uint32_t ret;

	sr(lr(ADC_CTRL) | ADC_SEQ_PTR_RST, ADC_CTRL);
	uint32_t set = lr(ADC_SET);
	/* Only one entry */
	set &= ADC_SEQ_SIZE_SET_MASK;
	sr(set, ADC_SET);

	sr(100 << 5 | channel, ADC_SEQ);

	sr(lr(ADC_CTRL) | ADC_SEQ_PTR_RST, ADC_CTRL);


	sr(ADC_SEQ_START | ADC_ENABLE | ADC_CLK_ENABLE, ADC_CTRL);

	while (!lr(ADC_INTSTAT))
		;

	sr(lr(ADC_SET) | ADC_POP_SAMPLE, ADC_SET);
	ret = lr(ADC_SAMPLE);
	sr(lr(ADC_CTRL) | ADC_CLR_DATA_A, ADC_CTRL);
	sr(lr(ADC_CTRL) | ADC_CLR_SEQ_ERR, ADC_CTRL);

	sr(lr(ADC_SET) | ADC_FLUSH_RX, ADC_SET);

	return ret;
}

/* Implement platform specific setup if needed */
__attribute__((weak)) void board_init(void)
{
}

inline static void cos_service(void)
{
	board_init();
	adc_init();
	COS_ARC_READY = 1;

	while (1) {
		int command = COS_ARC_REQ;
		if (!command)
			continue;
		COS_ARC_REQ = 0;

		switch (command) {
		case COS_CMD_READ_ADC:
		{
			int channel = COS_ARC_DATA;
			COS_ARC_DATA = adc_read(channel);
			COS_ARC_ACK = 1;
			break;
		}
		case COS_CMD_EXIT:
		{
			adc_release();
			COS_ARC_ACK = 1;
		}
		}
	}
}

void main(void)
{
	cos_service();
}
