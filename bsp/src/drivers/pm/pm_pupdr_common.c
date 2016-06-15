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

#include "infra/pm.h"
#include "machine.h"
#include "os/os.h"
#include "infra/log.h"

/* Error values returned by pm_shutdown */
#define RET_FAIL	1
#define RET_SB		2
#define RET_WL		3

#ifdef CONFIG_DEEPSLEEP
int pm_deepsleep(int auto_wake_time)
{
	int ret = RET_FAIL;
	int flags = interrupt_lock();

	if (!pm_check_suspend_blockers(PM_SUSPENDED)) {
		ret = -1;
		goto failed;
	}
	if (!pm_wakelock_is_list_empty()) {
		ret = -2;
		goto failed;
	}
	if (pm_core_specific_deepsleep(auto_wake_time)) {
		ret = -3;
		goto failed;
	}
	ret = 0;
	goto terminate;

failed:
	// Cannot go in deep sleep mode
	pm_core_specific_ack_error();
terminate:
	interrupt_unlock(flags);
	return ret;
}
#endif
static void pm_shutdown_cb(void* priv);

int pm_shutdown(int latch_mode)
{
	int ret = 0;
	int flags;
	T_SEMAPHORE sema = semaphore_create(0, NULL);

	flags = interrupt_lock();

	if (!pm_check_suspend_blockers(PM_SHUTDOWN)) {
		// Cannot power off device
		ret = RET_SB;
	}

	if (!pm_wakelock_is_list_empty()){
		ret = RET_WL;
	}

	if (ret == RET_WL){
		pm_wakelock_set_list_empty_cb(&pm_shutdown_cb, sema);
		interrupt_unlock(flags);
		semaphore_take(sema, 5000);
		flags = interrupt_lock();
	}

	pr_info(LOG_MODULE_MAIN, "board shutting down");
	log_flush();

	if (pm_core_specific_shutdown(latch_mode)){
		ret = RET_FAIL;
	}

	interrupt_unlock(flags);
	if (ret != 0)
		pm_core_specific_ack_error();
	semaphore_give(sema, NULL);
	semaphore_delete(sema, NULL);
	return ret;
}

static void pm_shutdown_cb(void* priv){
	T_SEMAPHORE sema = (T_SEMAPHORE)priv;
	pm_wakelock_set_list_empty_cb(NULL, NULL);
	semaphore_give(sema, NULL);
}
