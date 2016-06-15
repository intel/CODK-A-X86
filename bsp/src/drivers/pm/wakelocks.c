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

#include "os/os.h"
#include "infra/pm.h"
#include "infra/device.h"
#include "infra/log.h"
#include <stdbool.h>
#include <errno.h>

/*! Wakelock management structure */
struct pm_wakelock_mgr {
    list_head_t wl_list;      /*!< List that contains locked wakelocks, ordered by timeout */
    T_TIMER     wl_timer;     /*!< Timer to wait for next wakelock to expire */
    uint8_t     is_init;      /*!< Init state of wakelock structure */
    void        (*cb)(void*); /*!< Callback function to call when wakelock list is empty */
    void*       cb_priv;      /*!< Argument to pass with the callback function */
};

// Internal device driver functions
static void pm_wakelock_timeout_callback(void* priv);

static volatile struct pm_wakelock_mgr pm_wakelock_inst = {
                            .is_init = 0,
                            .wl_list = {.head=NULL, .tail=NULL},
                            .cb = NULL,
                            .cb_priv = NULL};

static void pm_wakelock_timeout_callback(void* priv)
{
    int time_diff = 0;
    bool spurious_irq = true;

    struct pm_wakelock_mgr *wl_mgr = (struct pm_wakelock_mgr*)priv;
    uint32_t current_time = (uint32_t)get_time_ms();

    // Lock IRQs as this function runs in timer task context
    uint32_t saved = interrupt_lock();

    list_t *l = wl_mgr->wl_list.head;

    // Remove all expired wakelocks
    while (l) {
        time_diff = (int)(((struct pm_wakelock*)l)->timeout) - \
                    (int)(current_time - ((struct pm_wakelock*)l)->start_time);
        if (time_diff > 0) {
            // Unexpired wakelock found, stop
            break;
        }

        // Remove expired wakelock from list
        wl_mgr->wl_list.head = l->next;
        spurious_irq = false;

        if (((struct pm_wakelock*)l)->timeout == WAKELOCK_MAX_TIMEOUT+1) {
            // Infinite wakelock expired, insert it again at end of list
            pr_warning(LOG_MODULE_DRV, "infinite wakelock %d hold for %d ms", ((struct pm_wakelock*)l)->id, WAKELOCK_MAX_TIMEOUT);
            // Update start_time to reset infinite wakelock
            ((struct pm_wakelock*)l)->start_time = current_time;

            if (wl_mgr->wl_list.head != NULL) {
                // Other wakelocks are in the list, put infinite wl at end of list
                wl_mgr->wl_list.tail->next = l;
                l->next = NULL;
                wl_mgr->wl_list.tail = l;
                // Select next wakelock to check for timeout
                l = wl_mgr->wl_list.head;
            } else {
                // Infinite wakelock is the only one in the list
                // Restore list head and stop
                wl_mgr->wl_list.head = l;
                // Set time_diff to restart wl timer
                time_diff = ((struct pm_wakelock*)l)->timeout;
                break;
            }
        } else {
            // Normal wakelock expired, release it
            ((struct pm_wakelock*)l)->lock = 0;
            pr_warning(LOG_MODULE_DRV, "wakelock %d expired", ((struct pm_wakelock*)l)->id);
            // Select next wakelock to check for timeout
            l = l->next;
        }
    }

    if (l == NULL) {
        // all timers expired
        wl_mgr->wl_list.head = wl_mgr->wl_list.tail = NULL;
    }
    // Unlock IRQs
    interrupt_unlock(saved);

    if (l != NULL) {
        // Restart timer
        timer_start(wl_mgr->wl_timer, time_diff, NULL);
    } else {
        // Check for spurious IRQ else we will call the callback function for nothing
        if (spurious_irq) {
            pr_error(LOG_MODULE_DRV, "Wakelock: spurious IRQ detected");
        } else {
            pm_wakelock_set_any_wakelock_taken_on_cpu(false);
            // Call callback function to notify that all wakelocks are free
            if (pm_wakelock_inst.cb != NULL) {
                pm_wakelock_inst.cb(pm_wakelock_inst.cb_priv);
            }
        }
    }
}

int pm_wakelock_init_mgr()
{
    // Check if already init
    if(pm_wakelock_inst.is_init) {
        return 0;
    }

    // Create timer to expire wakelocks on timeout event
    if (!(pm_wakelock_inst.wl_timer =
        timer_create(pm_wakelock_timeout_callback, (void*)(&pm_wakelock_inst), WAKELOCK_MAX_TIMEOUT, false, false, NULL))) {
        return -ENOMEM;
    }
    // Init ok
    pm_wakelock_inst.is_init = 1; // Wakelocks init ok
    return 0;
}

// ***********************************
// *        Wakelock user API        *
// ***********************************

void pm_wakelock_init(struct pm_wakelock *wli, int id)
{
    wli->start_time = 0;
    wli->lock = 0;
    wli->id = id;
    wli->timeout = 0;
    wli->list.next = NULL;
}

int pm_wakelock_acquire(struct pm_wakelock *wl, unsigned int timeout)
{
    list_t *l;

    if (timeout > WAKELOCK_MAX_TIMEOUT) {
        // Error in input parameters
        return -EINVAL;
    }

    // Acquire wakelock
    uint32_t saved = interrupt_lock();
    if (wl->lock) {
        interrupt_unlock(saved);
        return -EINVAL;
    }
    pm_wakelock_set_any_wakelock_taken_on_cpu(true);
    if (timeout == WAKELOCK_FOREVER) {
        /* instead of using another variable to know if a wakelock
         * is infinite or not, we simply set the timeout to
         * WAKELOCK_MAX_TIMEOUT+1. Timeout for a normal wakelock cannot
         * exceed WAKELOCK_MAX_TIMEOUT.
         */
        timeout = wl->timeout = WAKELOCK_MAX_TIMEOUT + 1;
    } else {
        wl->timeout = timeout;
    }

    uint32_t current_time = (uint32_t)get_time_ms();
    wl->lock = 1;

    wl->start_time = current_time;

    if (!(l = pm_wakelock_inst.wl_list.head)) {
        // List empty, add first item in the list
        pm_wakelock_inst.wl_list.head = pm_wakelock_inst.wl_list.tail = (list_t*)wl;
        wl->list.next = NULL;
        // The list is empty so timer is not running: start it
        timer_start(pm_wakelock_inst.wl_timer, timeout, NULL);
    } else if (timeout < (int)(((struct pm_wakelock*)l)->timeout) -
                         (int)(current_time - ((struct pm_wakelock*)l)->start_time)) {
        // Insert before first item in the list
        wl->list.next = l;
        pm_wakelock_inst.wl_list.head = (list_t*)wl;
        // We need to restart the timer on the new timeout
        timer_stop(pm_wakelock_inst.wl_timer, NULL);
        timer_start(pm_wakelock_inst.wl_timer, timeout, NULL);
    } else {
        // Insert item anywhere in the list
        for (; l->next; (l = l->next)) {
            int time_diff = (int)(((struct pm_wakelock*)(l->next))->timeout) - \
                            (int)(current_time - ((struct pm_wakelock*)(l->next))->start_time);
            if (timeout < time_diff) {
                break;
            }
        }
        // Check if wl item is the last item
        if(!(l->next)) {
            pm_wakelock_inst.wl_list.tail = (list_t*)wl;
        }

        wl->list.next = l->next;
        l->next = (list_t*)wl;
    }

    interrupt_unlock(saved);
    return 0;
}

int pm_wakelock_release(struct pm_wakelock *wl)
{
    list_t *l;
    int ret = 0;

    // Lock IRQs
    uint32_t saved = interrupt_lock();
    // Check if wakelock is already released
    if (!wl->lock) {
        ret = -EINVAL;
        goto exit;
    }
    // Release wakelock
    wl->lock = 0;
    l = pm_wakelock_inst.wl_list.head;

    if (l == (list_t*)wl) {
        // release first item in the wakelock list
        pm_wakelock_inst.wl_list.head = wl->list.next;

        // Check if there is only one item in the wakelock list
        if (pm_wakelock_inst.wl_list.tail == (list_t*)wl) {
            pm_wakelock_inst.wl_list.head = pm_wakelock_inst.wl_list.tail = NULL;

            // Stop the timer as all wakelocks have expired
            timer_stop(pm_wakelock_inst.wl_timer, NULL);

            pm_wakelock_set_any_wakelock_taken_on_cpu(false);
            // Call callback function to notify that all wakelocks are free
            if (pm_wakelock_inst.cb != NULL) {
                pm_wakelock_inst.cb(pm_wakelock_inst.cb_priv);
            }
        } else {
            uint32_t current_time = (uint32_t)get_time_ms();
            int time_diff = (int)(((struct pm_wakelock*)(l->next))->timeout) -
                            (int)(current_time - ((struct pm_wakelock*)(l->next))->start_time);
            if (time_diff > 0) {
                // Restart timer for next wakelock to expire
                timer_stop(pm_wakelock_inst.wl_timer, NULL);
                timer_start(pm_wakelock_inst.wl_timer, time_diff, NULL);
            }
        }
        goto exit;
    }

    // Find wakelock in the list
    for (; (l->next) && (l->next != (list_t*)wl); (l = l->next));

    if (l->next != NULL) {
        // Item before wl found, remove wl item from the list
        l->next = wl->list.next;

        // Check if wl item is the last one in the list
        if (pm_wakelock_inst.wl_list.tail == (list_t*)wl) {
            pm_wakelock_inst.wl_list.tail = l;
        }
    } else {
        //wl list clear
        ret = -ENOENT;
    }
exit:
    interrupt_unlock(saved);
    return ret;
}

bool pm_wakelock_is_list_empty()
{
    return (pm_wakelock_inst.wl_list.head == NULL ? true : false);
}

void pm_wakelock_set_list_empty_cb(void (*cb)(void*), void* priv)
{
    // Lock IRQs
    uint32_t saved = interrupt_lock();

    // Set the new callback function
    pm_wakelock_inst.cb = cb;
    pm_wakelock_inst.cb_priv = priv;

    interrupt_unlock(saved);
}

bool pm_check_suspend_blockers(PM_POWERSTATE state)
{
	int i = 0;

	for (; i < pm_blocker_size; i++) {
		if (!pm_blockers[i].cb(get_device(pm_blockers[i].dev_id), state)) {
			return false;
		}
	}
	return true;
}
