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

#include <stdlib.h>
#include "infra/log.h"
#include "misc/printk.h"
#include "infra/device.h"
#include "infra/ipc.h"
#include "infra/port.h"
#include "machine.h"
#include <drivers/system_timer.h>
#include <sys_clock.h>
#include "infra/time.h"

#include "os/os.h"

#define PANIC_ARC_NOT_RESUMED 0xdeadbeef /*!< Panic error code when ARC did not resumed */
#define PM_SUSPEND_DEFAULT    0          /*!< Default value for suspend/shutdown functions */
#define MSG_ID_PM_SLEEP       0xff99

// extern ASM functions to halt ARC
extern void halt_arc(void);

// OS init functions
extern void _arc_v2_irq_unit_init(void);
extern void _firq_stack_setup(void);

#ifdef CONFIG_DEEPSLEEP
// Context dump memory location
uint32_t _deep_sleep_registers[32];

// extern ASM functions to handle low level context save/restore
extern void deep_sleep_resume_hook(void);
extern void deep_sleep_dump_context(void);

int pm_core_specific_deepsleep(int auto_wake_time)
{
	uint32_t start;
	start = get_uptime_32k();
	// Suspend ARC devices
	int ret = suspend_devices(PM_SUSPENDED);
	if (ret != 0) {
		return ret;
	}

	shared_data->arc_start = (uint32_t)deep_sleep_resume_hook;

	// Send Acknoledge to QRK
	PM_ACK_SET_OK(shared_data->pm_request);
	// Dump context, ack QRK suspend req and suspend ARC core
	deep_sleep_dump_context();

	// ARC resumes here
	uint32_t icache_config = (
		CACHE_DIRECT | /* direct mapping (one-way assoc.) */
		CACHE_ENABLE   /* i-cache enabled */
	);
	uint32_t aux_irq_ctrl_value = (
		ARC_V2_AUX_IRQ_CTRL_LOOP_REGS | /* save lp_xxx registers */
		ARC_V2_AUX_IRQ_CTRL_BLINK     | /* save blink */
		ARC_V2_AUX_IRQ_CTRL_14_REGS     /* save r0 -> r13 (caller-saved) */
	);
	// Init instruction cache and IRQs
	WRITE_ARC_REG(icache_config, ARC_V2_IC_CTRL);
	WRITE_ARC_REG(aux_irq_ctrl_value, ARC_V2_AUX_IRQ_CTRL);
#ifndef CONFIG_QEMU_X86
	// Init firq stack register
	_firq_stack_setup();
	// Init interrupt unit device driver
	_arc_v2_irq_unit_init();
#endif



	if (PM_GET_REQUEST(shared_data->pm_request) != PM_RESUME_REQUEST) {
		// QRK did not requested a resume, panic
		panic(PANIC_ARC_NOT_RESUMED);
	}
	// Resume ARC devices
	ret = resume_devices();
	// Send resume ACK to QRK
	if (!ret) {
		PM_ACK_SET_OK(shared_data->pm_request);
	}
	uint32_t saved = interrupt_lock();
	uint32_t time = get_uptime_32k();

	uint32_t elapsed_ticks;
	if (time < start) {
		elapsed_ticks = (uint64_t)((0xffffffff - start + time) * sys_clock_ticks_per_sec) / 32768;
	} else {
		elapsed_ticks = (uint64_t)((time - start) * sys_clock_ticks_per_sec) / 32768;
	}
	extern uint32_t clock_accumulated_count;
	clock_accumulated_count += sys_clock_hw_cycles_per_tick * elapsed_ticks;
	_nano_sys_clock_tick_announce(elapsed_ticks);

	// Restore MBX interrupt
	interrupt_enable(SOC_MBX_INTERRUPT);
	// TODO: move to a MBX driver when implemented
	// Restore ARC OS timer
	_sys_clock_driver_init(NULL);
	interrupt_unlock(saved);

	return ret;
}
#endif

static void pm_handle_message(struct message * msg, void *param)
{
	switch(MESSAGE_ID(msg)) {
		case MSG_ID_PM_SLEEP:
			pm_idle();
			break;
	}
	message_free(msg);
}
static uint16_t pm_port_id;

void pm_notification_init(T_QUEUE queue)
{
	pm_port_id = port_alloc(queue);
	port_set_handler(pm_port_id, pm_handle_message, NULL);
}

int pm_notification_cb(uint8_t cpu_id, int param)
{
	struct message * msg = message_alloc(sizeof(*msg), NULL);
	MESSAGE_ID(msg) = MSG_ID_PM_SLEEP;
	MESSAGE_SRC(msg) = pm_port_id;
	MESSAGE_DST(msg) = pm_port_id;
	port_send_message(msg);
	return 0;
}

void pm_idle(void)
{
	uint32_t flags = interrupt_lock();
	if (PM_GET_REQUEST(shared_data->pm_request) == PM_SUSPEND_REQUEST &&
			!PM_IS_ACK_OK(shared_data->pm_request) &&
			!PM_IS_ACK_ERROR(shared_data->pm_request)) {
		switch (PM_GET_STATE(shared_data->pm_request)) {
#ifdef CONFIG_DEEPSLEEP
			case PM_SUSPENDED:
				pm_deepsleep(PM_SUSPEND_DEFAULT);
				break;
#endif
			case PM_SHUTDOWN:
				pm_shutdown(PM_SUSPEND_DEFAULT);
				break;
			default:
				pr_error(LOG_MODULE_DRV, "Unknown suspend type 0x%x", shared_data->pm_request);
				break;
		}
	}
	interrupt_unlock(flags);

}

int pm_core_specific_shutdown(int latch_mode){

	suspend_devices(PM_SHUTDOWN);
	PM_ACK_SET_OK(shared_data->pm_request);
	halt_arc();
	return -1;
}

void pm_core_specific_ack_error(){
	PM_ACK_SET_ERROR(shared_data->pm_request);
}
