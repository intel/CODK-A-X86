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

#ifndef DW_WDT_H_
#define DW_WDT_H_

#include <board.h>
#include <device.h>
#include <watchdog.h>


/**
 * Watchdog timer register block type.
 */
struct dw_wdt {
        volatile uint32_t wdt_cr;           /**< Control Register */
        volatile uint32_t wdt_torr;         /**< Timeout Range Register */
        volatile uint32_t wdt_ccvr;         /**< Current Counter Value Register */
        volatile uint32_t wdt_crr;          /**< Current Restart Register */
        volatile uint32_t wdt_stat;         /**< Interrupt Status Register */
        volatile uint32_t wdt_eoi;          /**< Interrupt Clear Register */
        volatile uint32_t wdt_comp_param_5; /**<  Component Parameters */
        volatile uint32_t wdt_comp_param_4; /**<  Component Parameters */
        volatile uint32_t wdt_comp_param_3; /**<  Component Parameters */
        volatile uint32_t wdt_comp_param_2; /**<  Component Parameters */
        volatile uint32_t wdt_comp_param_1; /**<  Component Parameters Register 1 */
        volatile uint32_t wdt_comp_version; /**<  Component Version Register */
        volatile uint32_t wdt_comp_type;    /**< Component Type Register */
};

/** WDT register block */
#define DW_WDT ((struct dw_wdt *)WDT_BASE_ADDR)


#define WDT_CRR_VAL                 0x76
#define WDT_CR_ENABLE               (1 << 0)
#define WDT_CR_INT_ENABLE           (1 << 1)        /* interrupt mode enable - mode1 */


#define WDT_DRV_NAME "dw_wdt"

struct dw_wdt_dev_config {
        uint32_t        base_address;
};

int dw_wdt_init(struct device *dev);

#endif /* DW_WDT_H_ */
