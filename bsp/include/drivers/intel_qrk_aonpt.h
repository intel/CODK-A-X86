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

#ifndef INTEL_QRK_AONPT_H_
#define INTEL_QRK_AONPT_H_

#include "infra/device.h"

/**
 * @defgroup AON Periodic Timer driver implementation.
 * Input callback function called when counter reachs to 0
 * @ingroup common_drivers
 * @{
 */

/**
 *  AONPT driver.
 */
extern struct driver aonpt_driver;

/**
 *  Starts or restarts the periodic timer. This will decrement the initial
 *  value set via qrk_aonpt_configure(...).
 *
 *  If the timer is already running, it will be stopped and restarted.
 *  If one_shot was set to true, the callback will be called only once.
 */
void qrk_aonpt_start(void);

/**
 *  Read the AON counter value.
 *
 *  The unit is value*(1/32768) secs, counter is loaded with 'value' and
 *  decremented each tick.
 *
 *  @return  AON counter current value.
 */
uint32_t qrk_aonpt_read(void);

/**
*  Stops the periodic timer.
*/
void qrk_aonpt_stop(void);

/**
 *  Configure the aonpt alarm and call to qrk_aon_start() is required to
 *  actually start the timer.
 *
 *  To have one shot feature available, and call back, please call
 *  qrk_aon_start() afterwards.
 *
 *  @param   period          initial value to trigger the alarm, unit is in (1/32768) secs.
 *  @param   on_timeout_cb   callback function pointer, this cb function is called in ISR context.
 *  @param   one_shot        If true, the callback function is called once.
 *
 */
void qrk_aonpt_configure(uint32_t period, void (*on_timeout_cb)(),
		bool one_shot);

/**
 * @}
 */
#endif //INTEL_QRK_AONPT_H_
