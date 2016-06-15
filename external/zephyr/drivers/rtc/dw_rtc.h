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

#ifndef _DW_RTC_H_
#define _DW_RTC_H_

#include <board.h>
#include <device.h>
#include <rtc.h>

#define RTC_DRV_NAME		"rtc"

/**
 * RTC Register block type.
 */
typedef struct {
        volatile uint32_t rtc_ccvr;         /**< Current Counter Value Register */
        volatile uint32_t rtc_cmr;          /**< Current Match Register */
        volatile uint32_t rtc_clr;          /**< Counter Load Register */
        volatile uint32_t rtc_ccr;          /**< Counter Control Register */
        volatile uint32_t rtc_stat;         /**< Interrupt Status Register */
        volatile uint32_t rtc_rstat;        /**< Interrupt Raw Status Register */
        volatile uint32_t rtc_eoi;          /**< End of Interrupt Register */
        volatile uint32_t rtc_comp_version; /**< End of Interrupt Register */
} dw_rtc_t;


/** RTC register block */
#define DW_RTC ((dw_rtc_t *)RTC_BASE_ADDR)


#define RTC_INTERRUPT_ENABLE        (1 << 0)
#define RTC_INTERRUPT_MASK          (1 << 1)
#define RTC_ENABLE                  (1 << 2)
#define RTC_WRAP_ENABLE             (1 << 3)

#define RTC_CLK_DIV_EN     	    (1 << 2)
#define RTC_CLK_DIV_MASK            (0xF << 3)
#define RTC_CLK_DIV_1_HZ            (0xF << 3)
#define RTC_CLK_DIV_32768_HZ        (0x0 << 3)
#define RTC_CLK_DIV_8192_HZ         (0x2 << 3)
#define RTC_CLK_DIV_4096_HZ         (0x3 << 3)

struct dw_rtc_dev_config {
        uint32_t        base_address;
};

int dw_rtc_init(struct device* dev);

#endif
