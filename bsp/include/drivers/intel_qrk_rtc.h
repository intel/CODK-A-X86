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

#ifndef INTEL_QRK_RTC_H_
#define INTEL_QRK_RTC_H_

#include "drivers/data_type.h"
#include "infra/device.h"
#include "drivers/clk_system.h"

/**
 * @defgroup rtc Intel Quark RTC
 * Intel Quark Real Time Clock driver API.
 * @ingroup common_drivers
 * @{
 */

/**
 *  RTC driver.
 */
extern struct driver rtc_driver;

/**
 * RTC Power management structure
 */
struct rtc_pm_data {
	struct clk_gate_info_s* clk_gate_info; /*!< pointer to clock gate data */
};

struct pm_wakelock rtc_wakelock;
/**
 * RTC configuration.
 *
 * The user instantiates one of these with given parameters for the WDT,
 * configured using the "qrk_cxxxx_pwm_set_config" function
 */
struct qrk_cxxxx_rtc_config
{
    uint32_t initial_rtc_val;       /*!< initial configuration value for the 32bit RTC counter value */
                                    /*it is possible to add other fields like clock divider settings */
};

struct qrk_cxxxx_rtc_alarm
{
    bool alarm_enable;              /*!< enable/disable alarm  */
    uint32_t alarm_rtc_val;         /*!< initial configuration value for the 32bit RTC alarm value  */
    void (*callback_fn) (uint32_t); /*!< Pointer to function to call when alarm value matches current RTC value */
    uint32_t callback_param;        /*!< parameter for the rtc alarm callback */
};

/**
 * Enable clock gating for the RTC.
 *
 * @param  dev        pointer to the rtc device
 */
void qrk_cxxxx_rtc_enable(struct device *dev);

/**
 * Enable clock gating for the RTC.
 *
 * @param  dev        pointer to the rtc device
 */
void qrk_cxxxx_rtc_disable(struct device *dev);

/**
 * Configure the RTC.
 *
 * @param config     pointer to a RTC configuration structure
 *
 * @return
 *         - DRV_RC_OK on success
 *         - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC qrk_cxxxx_rtc_set_config(struct qrk_cxxxx_rtc_config *config);

/**
 *  Configure the RTC alarm.
 *
 *  @param alarm    pointer to a RTC alarm configuration structure
 *
 *  @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC qrk_cxxxx_rtc_set_alarm(struct qrk_cxxxx_rtc_alarm *alarm);

/**
 * Read the RTC.
 *
 *  @return uint32_t - epoch time
 */
uint32_t qrk_cxxxx_rtc_read(void);

/**
 * Disable the RTC clock.
 */
void qrk_cxxxx_rtc_clk_disable (void);

/** @} */

#endif /* INTEL_QRK_RTC_H_ */
