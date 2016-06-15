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

#include "machine.h"

#include "infra/ipc.h"
#include "infra/port.h"
#include "infra/pm.h"
#include "infra/device.h"

/* This function is also defined in OS test suite */
#ifndef CONFIG_ARC_OS_UNIT_TESTS
void mbxIsr(void * param)
{
	ipc_handle_message();
}
#else
extern void mbxIsr(void * param);
#endif

void soc_setup()
{
#ifdef CONFIG_WAKELOCK
	/* Init wakelocks */
	pm_wakelock_init_mgr();
#endif

	/* Init devices */
	init_all_devices();

	ipc_init(5, 0, 6, 1, CPU_ID_QRK);

	SET_INTERRUPT_HANDLER(SOC_MBX_INTERRUPT, mbxIsr);

	set_cpu_id(CPU_ID_ARC);

	/* Notify QRK that ARC started. */
	shared_data->arc_ready = 1;
}
#ifdef CONFIG_WAKELOCK
bool pm_wakelock_are_other_cpu_wakelocks_taken(void){
	return shared_data->any_qrk_wakelock_taken;
}

void pm_wakelock_set_any_wakelock_taken_on_cpu(bool wl_status){
	shared_data->any_arc_wakelock_taken = wl_status;
}
#endif
