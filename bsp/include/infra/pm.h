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

#ifndef __PM_H__
#define __PM_H__

#include "util/list.h"
#include "os/os.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup infra_pm Power Management
 * Defines power management functions for the infrastructure.
 * @ingroup infra
 * @{
 */

#define WAKELOCK_MAX_TIMEOUT 5000 /*!< Maximum timeout delay in ms for a wakelock */
#define WAKELOCK_FOREVER     0    /*!< Infinite timeout code for wakelocks */

/**
 * List of all device power states (From shutdown to running).
 */
typedef enum {
    PM_NOT_INIT = 0, /*!< Device not initialized */
    PM_SHUTDOWN,     /*!< Device stopped */
    PM_SUSPENDED,    /*!< Device suspended */
    PM_RUNNING,      /*!< Device working properly */
    PM_COUNT
} PM_POWERSTATE;

struct device; // Declare device struct witch is used in pm_suspend_blocker
/**
 * Structure that contains driver callback to check that deep sleep is allowed.
 * Callback must return true if deep sleep is allowed.
 */
struct pm_suspend_blocker {
	int dev_id;
	bool (*cb)(struct device *dev, PM_POWERSTATE state);
};

/**
 * Reboot modes available.
 */
typedef enum {
    TARGET_MAIN = 0x0,
    TARGET_CHARGING,
    TARGET_WIRELESS_CHARGING,
    TARGET_RECOVERY,
    TARGET_FLASHING,
    TARGET_FACTORY,
    TARGET_OTA,
    TARGET_DTM,
    TARGET_CERTIFICATION,
    TARGET_RESERVED_0,
    TARGET_APP_1,
    TARGET_APP_2,
    TARGET_RESERVED_1,
    TARGET_RESERVED_2,
    TARGET_RESERVED_3,
    TARGET_RESERVED_4
} PM_REBOOT_MODE;

#define PM_REBOOT_MODE_MASK  0x0000000F

#define PM_SUSPEND_REQUEST   0x51EEb000
#define PM_RESUME_REQUEST    0xCAFFE000

#define PM_POWERSTATE_MASK   0x000000FF
#define PM_REQUEST_MASK      0xFFFFF000

#define PM_ACK_TIMEOUT       0
#define PM_ACK_OK            (1 << 8)
#define PM_ACK_ERROR         (1 << 9)

#define PM_GET_REQUEST(req)  ((req) & PM_REQUEST_MASK)
#define PM_GET_STATE(req)    ((req) & PM_POWERSTATE_MASK)

#define PM_IS_ACK_OK(req)    ((req) & PM_ACK_OK)
#define PM_IS_ACK_ERROR(req) ((req) & PM_ACK_ERROR)

#define PM_ACK_SET_OK(req)    (req) |= PM_ACK_OK
#define PM_ACK_SET_ERROR(req) (req) |= PM_ACK_ERROR

#define PM_INIT_REQUEST(req, type, state) ((req) = \
		(type & PM_REQUEST_MASK) | \
		(state & PM_POWERSTATE_MASK))

/**
 * Goes to the lowest possible power mode.
 *
 * Tries to go in deep sleep mode; or go to core low power mode.
 */
void pm_idle(void);

/**
 * Requests a deepsleep of the platform.
 *
 * Triggers the suspend resume procedure.
 *
 * @param auto_wake_time if different from 0, the number of seconds after which we
 *         want an automatic wakeup resume of the platform
 *
 * @return 0 if success else POSIX return code error
 */
int pm_deepsleep(int auto_wake_time);

/**
 * Requests a shutdown of the platform.
 *
 * Triggers the shutdown procedure of the platform and eventually
 * put the platform in the latched off mode.
 *
 * Note that the latched off mode needs specific hardware support to physically
 * disconnect the battery and re-connect it with a button press.
 *
 * @param latch_mode set to true if latched off mode is requested.
 *
 * @return 0 if success else POSIX return code error
 */
int pm_shutdown(int latch_mode);

/**
 * Requests a reboot of the platform.
 *
 * Triggers a platform reboot in the selected functional mode.
 *
 * @param reboot_mode  according to PM_REBOOT_MODE enum
 *
 * @return 0 if success else POSIX return code error
 */
int pm_reboot(PM_REBOOT_MODE reboot_mode);

/**
 * Shutdown procedure specific to each core.
 *
 * For ARC core: suspends drivers and halt CPU.
 * For Quark core: trigger arc shutdown procedure, suspends drivers and halt CPU.
 *
 * @param latch_mode set to true if latched off mode is requested.
 * @return 0 if success else POSIX return code error
 */
int pm_core_specific_shutdown(int latch_mode);

/**
 * Deepsleep procedure specific to each core.
 *
 * For ARC core: suspends drivers, halt CPU and resume.
 * For Quark core: trigger arc deepsleep procedure, suspends drivers,
 * enable RTC wakeup source, halt CPU and resume.
 *
 * @param auto_wake_time delay in seconds before RTC wake the board up
 * if 0 RTC wake up source is not activated.
 *
 * @return 0 if success else POSIX return code error
 */
int pm_core_specific_deepsleep(int auto_wake_time);

/**
 * Used by ARC core to signal shutdown procedure fails.
 */
void pm_core_specific_ack_error();

/**
 * Wakelock managing structure.
 *
 * - A wakelock is a lock used to prevent the platform to enter suspend/shutdown.
 * - When a device acquires a wakelock, it needs to release it (or wait for
 *   timeout) before platform can enter in suspend/resume mode.
 * - The platform is notified that all wakelocks have been released through the
 *   callback function configured using the function
 *   pm_wakelock_set_list_empty_cb.
 *
 * A component using wakelocks should allocate a struct pm_wakelock.
 */
struct pm_wakelock {
    list_t list;         /*!< Internal list management member */
    int id;              /*!< Client identifier for the hook. must be platform global */
    uint32_t timeout;    /*!< Wakelock timeout in ms (used to release the wakelock after a timeout) */
    uint32_t start_time; /*!< Start time in ms (used to compute expiration time) */
    unsigned int lock;   /*!< Lock to avoid acquiring a lock several times */
};

/**
 * Initializes wakelock management structure.
 *
 * @return 0 on success, -ENOMEM if device is out of memory
 */
int pm_wakelock_init_mgr();

/**
 * Initializes a wakelock structure.
 *
 * @param wli the wakelock structure pointer to initialize.
 * @param id the identifier of the wakelock.
 */
void pm_wakelock_init(struct pm_wakelock *wli, int id);

/**
 * Acquires a wakelock.
 *
 * Prevents the platform to sleep for timeout milliseconds. Wakelock
 * will be automatically released after timeout.
 *
 * @param wl the wakelock to acquire.
 * @param timeout the automatic release timeout in ms.
 *
 * @return 0 on success, -EINVAL if already locked or timeout is greater than WAKELOCK_MAX_TIMEOUT
 */
int pm_wakelock_acquire(struct pm_wakelock *wl, unsigned int timeout);

/**
 * Releases a wakelock.
 *
 * @param wl the wakelock to release
 *
 * @return 0 on success, -EINVAL if already released
 */
int pm_wakelock_release(struct pm_wakelock *wl);

/**
 * Checks if wakelock list is empty.
 *
 * @return true (1) if at least one wakelock is acquired else false (0)
 */
bool pm_wakelock_is_list_empty();

/**
 * Checks that all deep sleep blockers are released.
 *
 * @param state state to transition to
 * @return true (1) if deep sleep mode transition is allowed else false (0)
 */
bool pm_check_suspend_blockers(PM_POWERSTATE state);

#define DECLARE_SUSPEND_BLOCKERS(...) \
const struct pm_suspend_blocker pm_blockers[] = {__VA_ARGS__}; \
const uint8_t pm_blocker_size = sizeof(pm_blockers)/sizeof(*pm_blockers);

/**
 * Suspends blockers array.
 *
 * @warning DECLARE_SUSPEND_BLOCKERS macro must be used to declare this array
 */
extern const struct pm_suspend_blocker pm_blockers[];

/**
 * Suspends blockers array size.
 *
 * @warning DECLARE_SUSPEND_BLOCKERS macro must be used to declare this array
 */
extern const uint8_t pm_blocker_size;

/**
 * Sets callback to call when wakelock list is empty.
 *
 * It will replace the previous callback.
 *
 * @param cb   the callback function
 * @param priv the private data to pass to the callback function
 */
void pm_wakelock_set_list_empty_cb(void (*cb)(void*), void* priv);

/**
 * Initializes power management handler.
 *
 * Used by PM slaves nodes to set the context on which the
 * power management requests will be handled.
 *
 * Currently, the deep sleep request is the only power management request
 * flowing from master to slaves.
 *
 * It is used to wake up the slave in order to actually trigger deep sleep.
 *
 * @param queue the queue on which the power management messages will be handled.
 */
void pm_notification_init(T_QUEUE queue);

/**
 * Defines the function that pm slaves nodes needs to implement in order
 * to handle power management IPC requests.
 *
 * @param cpu_id the CPU triggering the pm request
 * @param param the IPC param for the pm request
 */
int pm_notification_cb(uint8_t cpu_id, int param);

/**
 * Board specific hook to provide shutdown.
 *
 * If a board requires a specific action to provide shutdown, it will need to
 * implement this function and set the CONFIG_HAS_BOARD_SHUTDOWN option. The
 * function will be called by master core's pm shutdown API.
 *
 * @param latch_mode hint to the board implementation: set to 1 if you want
 * to disconnect battery.
 */
void board_shutdown_hook(int latch_mode);

/**
 * Core specific function to set wakelock state shared variable.
 *
 * Set cpu specific wakelock state shared variable.
 * For example, when all wakelocks are released and when at least
 * one is taken.
 *
 * @param wl_status: current wakelock status to set.
 */
void pm_wakelock_set_any_wakelock_taken_on_cpu(bool wl_status);

/**
 * Check slaves wakelocks state
 *
 * @return false if all wakelocks are released
 */
bool pm_wakelock_are_other_cpu_wakelocks_taken(void);

/** @} */

#endif /* __PM_H_ */
