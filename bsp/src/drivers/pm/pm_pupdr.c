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

#include <errno.h>
#include "infra/pm.h"
#include "infra/ipc.h"

#include "drivers/intel_qrk_rtc.h"
#include "drivers/deep_sleep.h"
#include "drivers/usb_pm.h"

#include "machine.h"
#include "machine/soc/quark_se/quark/soc_setup.h"
#include "soc_config.h"
#ifdef CONFIG_UART_NS16550
#include "drivers/ipc_uart_ns16550.h"
#endif

#define PANIC_SHUTDOWN_FAILED		1	/*!< Panic error code for shutdown failures   */
#define PANIC_ARC_SUSPEND_TIMEOUT	2	/*!< Panic error code for arc suspend timeout */
#define PANIC_ARC_SHUTDOWN_TIMEOUT	3	/*!< Panic error code for arc suspend timeout */
#define PANIC_ARC_RESUME_TIMEOUT	4	/*!< Panic error code for arc resume timeout  */
#define PANIC_ARC_RESUME_ERROR		5	/*!< Panic error code for arc resume error    */

#include "infra/log.h"
#include "infra/time.h"

#define SEC_TO_EPOCH 1

#define UPDATE_DELAY 5

/*!< wait 10 s timeout for ARC suspend and wakelocks expiration (before PANIC) */
#define IPC_PM_SUSPEND_TIMEOUT (10000)


int pm_wait_suspend_request_ack(){
	uint32_t start_time = get_uptime_ms();
	while ((get_uptime_ms() - start_time) < IPC_PM_SUSPEND_TIMEOUT) {
		// Check for ack notification from ARC
		if      (PM_IS_ACK_OK(shared_data->pm_request))    return PM_ACK_OK;
		else if (PM_IS_ACK_ERROR(shared_data->pm_request)) return PM_ACK_ERROR;
		// Process IPC messages to avoid hard IPC timeout
		ipc_handle_message();
	}
	// Timeout occured
	return PM_ACK_TIMEOUT;
}

#ifdef CONFIG_DEEPSLEEP
extern struct pm_wakelock rtc_wakelock;
/**
 * \brief Callback function for RTC in case sleep fails
 */
static void rtc_cb_fn(uint32_t val)
{
}

static volatile int arc_ack_received = 0;
static volatile int arc_status = 0;

/**
 * \brief Configure RTC alarm
 */
static DRIVER_API_RC qrk_config_rtc_alarm(unsigned int delay)
{
	struct qrk_cxxxx_rtc_alarm alarm;
	DRIVER_API_RC ret;

	// Configure RTC alarm
	alarm.alarm_enable = true;
	alarm.callback_fn = rtc_cb_fn;
	alarm.alarm_rtc_val = (delay * SEC_TO_EPOCH) + qrk_cxxxx_rtc_read();
	uint32_t t =  qrk_cxxxx_rtc_read();
	ret = qrk_cxxxx_rtc_set_alarm(&alarm);
	while ((t + UPDATE_DELAY) !=  MMIO_REG_VAL_FROM_BASE(QRK_RTC_BASE_ADDR, QRK_RTC_CCVR));
	pm_wakelock_release(&rtc_wakelock);
	return ret;
}

int pm_core_specific_deepsleep(int auto_wake_time){
	DRIVER_API_RC ret = DRV_RC_FAIL;
	int ack;
	volatile uint32_t* slp_cfg = &SCSS_REG_VAL(SLP_CFG_BASE);
	const uint32_t iostate_ret_mask =
#ifdef CONFIG_QUARK_SE_DISABLE_INTERNAL_VREG
	(SLP_CFG_VRET_SEL_135 | SLP_CFG_IO_STATE_RET_EN | SLP_CFG_LPMODE_EN);
#else
	(SLP_CFG_VRET_SEL_135 | SLP_CFG_IO_STATE_RET_EN);
#endif

	if(pm_wakelock_are_other_cpu_wakelocks_taken()){
		return -EBUSY;
	}

	// Init pm request
	PM_INIT_REQUEST(shared_data->pm_request, PM_SUSPEND_REQUEST, PM_SUSPENDED);
	// Send the PM request IPC
	ipc_request_sync_int(IPC_REQUEST_INFRA_PM, 0, 0, NULL);
	// Wait for ARC acknowledge
	ack = pm_wait_suspend_request_ack();
	if (ack == PM_ACK_TIMEOUT) panic(PANIC_ARC_SUSPEND_TIMEOUT);
	if (ack != PM_ACK_OK) goto failed;

	ret = suspend_devices(PM_SUSPENDED);
	/* ensure arc is halted */
	uint32_t start_time = get_uptime_ms();
	while (!(SCSS_REG_VAL(SCSS_SS_STS) & SCSS_SS_STS_ARC_HALT)) {
		if ((get_uptime_ms() - start_time) < IPC_PM_SUSPEND_TIMEOUT) {
			// ARC halt timeout detected
			panic(PANIC_ARC_SUSPEND_TIMEOUT);
		}
	}

	if (ret != 0) {
		pr_error(LOG_MODULE_DRV, "QRK suspend failed (%d)", ret);
		goto resume_arc;
	}

	/* Configure IO State Retention to hold output states on pins */
	*slp_cfg = iostate_ret_mask;
	/* Loop until the bit is set */
	while (!(*slp_cfg & iostate_ret_mask));

	// Configure RTC wakeup alarm
	if (auto_wake_time > 0) qrk_config_rtc_alarm(auto_wake_time);
	// Platform Saved ctx
	soc_suspend();

	// Goto deep sleep mode
	qrk_deep_sleep();
	// Platform restore
	soc_resume();
	ret = resume_devices();

resume_arc:
	// Init resume request
	PM_INIT_REQUEST(shared_data->pm_request, PM_RESUME_REQUEST, PM_RUNNING);

	/* Put ARC out of reset */
	SCSS_REG_VAL(SCSS_SS_CFG) = ARC_RUN_REQ_A;

	ack = pm_wait_suspend_request_ack();
	if (ack == PM_ACK_TIMEOUT) panic(PANIC_ARC_RESUME_TIMEOUT);
	if (ack != PM_ACK_OK) panic(PANIC_ARC_RESUME_ERROR);

	/* Release IO State Retention to re-connect output pins to SoC */
#ifdef CONFIG_QUARK_SE_DISABLE_INTERNAL_VREG
	*slp_cfg = SLP_CFG_IO_STATE_RET_HOLD | SLP_CFG_VRET_SEL_135 | SLP_CFG_LPMODE_EN;
#else
	*slp_cfg = SLP_CFG_IO_STATE_RET_HOLD | SLP_CFG_VRET_SEL_135;
#endif
failed:
	return ret;
}
#endif

int pm_core_specific_shutdown(int latch_mode){
	int ack;

	PM_INIT_REQUEST(shared_data->pm_request, PM_SUSPEND_REQUEST, PM_SHUTDOWN);
	// Send the PM request IPC
	ipc_request_sync_int(IPC_REQUEST_INFRA_PM, 0, 0, NULL);

	ack = pm_wait_suspend_request_ack();
	if (ack == PM_ACK_TIMEOUT) panic(PANIC_ARC_SHUTDOWN_TIMEOUT);
	if (ack == PM_ACK_ERROR) return -1;

	suspend_devices(PM_SHUTDOWN);

#ifdef CONFIG_BOARD_SHUTDOWN
	board_shutdown_hook(latch_mode);
#endif

	SCSS_REG_VAL(SCSS_RSTC) = RSTC_COLD_RESET;
	while(1); /* wait for shut down to be effective */

	return -1;
}

void pm_core_specific_ack_error(){};

int pm_reboot(PM_REBOOT_MODE reboot_mode)
{
#ifdef CONFIG_UART_NS16550
	/* Disable message reception before reboot */
	ipc_uart_ns16550_disable(get_device(IPC_UART_ID));
#endif

	/* clear allocated bits of SCSS_GPS1 */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,SCSS_GPS1) &= (~PM_REBOOT_MODE_MASK);
	/* set allocated bits of SCSS_GPS1 */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE,SCSS_GPS1) |= (reboot_mode & PM_REBOOT_MODE_MASK);
	suspend_devices(PM_SHUTDOWN);
	SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_HALT_REQ_A;

	SCSS_REG_VAL(SCSS_RSTC) = RSTC_WARM_RESET;
	return 0;
}
