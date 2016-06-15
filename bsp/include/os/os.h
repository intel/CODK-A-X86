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

#ifndef __OS_H__
#define __OS_H__

#include "os/os_types.h"
#include "infra/panic.h" /* To be provided by the platform */

/**
 * @defgroup os OS Abstraction Layer
 * Defines the OS Abstraction Layer API.
 *
 * This layer defines functions to handle many objects regardless of the
 * low-level OS. This allows inter-operability.
 * - Sempahores (count),
 * - Mutex (binary),
 * - Scheduling,
 * - Queues,
 * - Interrupts,
 * - Time / timer,
 * - Memory allocation,
 * - Tasks.
 *
 * @{
 *
 */

/**
 * @defgroup os_semaphore Semaphore
 * Defines control functions for semaphores.
 *
 * Semaphores are used to control concurrent accesses of data from multiple
 * processes.
 *
 * For example, an interrupt can fill a buffer, while a task reads the buffer
 * periodically.
 *
 * Not all the functions can be called from interrupt context. Here is a
 * summary of the compatible contexts for each function.
 *
 * Function name            | Task ctxt | Fiber ctxt| Interrupt |
 * -------------------------|:---------:|:---------:|:---------:|
 * @ref semaphore_create    |     X     |     X     |           |
 * @ref semaphore_delete    |     X     |     X     |           |
 * @ref semaphore_give      |     X     |     X     |     X     |
 * @ref semaphore_take      |     X     |     X     |           |
 * @ref semaphore_get_count |     X     |     X     |     X     |
 *
 * @{
 */

/**
 * Creates a semaphore.
 *
 * Creates or reserves a semaphore object. The service may fail if all allocated
 * semaphores are already being used.
 *
 * @warning This service may panic if err parameter is null and:
 * - no semaphore is available, or
 * - when called from an ISR.
 *
 * Semaphores are not dynamically allocated and destroyed. They are picked from
 * a (limited) static pool that is defined at configuration time.
 *
 * Concurrent accesses to this pool are serialized by a framework-specific
 * semaphore.
 *
 * The total number of semaphores that may be in use (created) at the same time
 * is 32.
 *
 * To reset a semaphore (TBC), use @ref semaphore_delete, then semaphore_create.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @param initialCount: initial count of the semaphore.
 *
 * @param[out] err execution status:
 *     - E_OS_OK:  semaphore was created,
 *     - E_OS_ERR: all semaphores from the pool are already being used,
 *     - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 *
 * @return
 *    - Handler of the created semaphore,
 *    - NULL if all allocated semaphores are already being used.
 */
extern T_SEMAPHORE semaphore_create (uint32_t initialCount, OS_ERR_TYPE* err);

/**
 * Deletes a semaphore.
 *
 * Disables a semaphore that was reserved by semaphore_create. Deleting a
 * semaphore while a task is waiting (or will wait) for it to be signaled
 * may create a deadlock.
 *
 * @warning This service may panic if err parameter is null and:
 * - semaphore parameter is invalid, or
 * - when called from an ISR.
 *
 * Authorized execution levels:  task, fiber.
 *
 * See also @ref semaphore_create.
 *
 * @param semaphore: handler of the semaphore to delete
 *                   (returned by @ref semaphore_create).
 *
 * @param[out] err  execution status:
 *         - E_OS_OK:  semaphore was deleted,
 *         - E_OS_ERR: semaphore parameter is invalid, or was not created,
 *         - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern void semaphore_delete ( T_SEMAPHORE semaphore, OS_ERR_TYPE* err );

/**
 * Gives/signals a semaphore.
 *
 * @warning This service may panic if err parameter is null and:
 * - semaphore parameter is invalid.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param semaphore: handler of the semaphore to delete
 *                   (returned by @ref semaphore_create).
 *
 * @param[out] err  execution status:
 *     - E_OS_OK semaphore was freed/signaled,
 *     - E_OS_ERR: semaphore parameter is invalid, or was not created.
 */
extern void semaphore_give(T_SEMAPHORE semaphore, OS_ERR_TYPE* err);

/**
 * Takes/blocks a semaphore.
 *
 * This service may block while waiting on the semaphore,
 * depending on the timeout parameter. This service shall not be called from
 * an ISR context.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @param semaphore: handler of the semaphore to delete
 *                   (value returned by @ref semaphore_create).
 *
 * @param timeout: maximum number of milliseconds to wait for the semaphore.
 *                 Special values OS_NO_WAIT and OS_WAIT_FOREVER may be used.
 *
 * @return execution status:
 *      - E_OS_OK: semaphore was successfully taken,
 *      - E_OS_ERR / INVALID_PARAM: semaphore parameter is invalid
 *                                  (semaphore was deleted or never created),
 *      - E_OS_ERR_TIMEOUT: could not take semaphore before timeout expiration,
 *      - E_OS_ERR_BUSY: could not take semaphore (did not wait),
 *      - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern OS_ERR_TYPE  semaphore_take(T_SEMAPHORE semaphore, int timeout);

/**
 * Gets the semaphore count.
 *
 * Returns the count of a semaphore.
 *
 * @warning This service may panic if err parameter is null and:
 * - semaphore parameter is invalid.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param semaphore: handler of the semaphore to delete
 *                   (returned by @ref semaphore_create).
 *
 * @param[out] err   execution status:
 *         - E_OS_OK:  returned count value is correct,
 *         - E_OS_ERR: semaphore parameter is invalid, or was not created.
 *
 * @return
 *        - negative value: number of clients waiting on the semaphore,
 *        - positive value: number of times the semaphore has been signaled.
 */
extern int32_t semaphore_get_count(T_SEMAPHORE semaphore, OS_ERR_TYPE* err);

/**
 * @}
 *
 * @defgroup os_mutex Mutex
 * Defines control functions for mutexes.
 *
 * A mutex is a synchronization object, used to protect shared data from
 * concurrent access by multiple tasks.
 *
 * @warning Mutexes can't be used inside an ISR (interrupt context).
 *
 * Function name            | Task ctxt | Fiber ctxt| Interrupt |
 * -------------------------|:---------:|:---------:|:---------:|
 * @ref mutex_create        |     X     |     X     |           |
 * @ref mutex_delete        |     X     |     X     |           |
 * @ref mutex_unlock        |     X     |     X     |           |
 * @ref mutex_lock          |     X     |     X     |           |
 *
 * @{
 */

/**
 * Creates a mutex.
 *
 * Creates or reserves a mutex object. The service may fail if all allocated
 * mutexes are already being used.
 *
 * @warning This service may panic if err parameter is null and:
 * - no mutex is available, or
 * - when called from an ISR.
 *
 * Authorized execution levels:  task, fiber.
 *
 * Mutexes are not dynamically allocated and destroyed. They are picked from a
 * (limited) static pool that is defined at configuration time.
 *
 * Concurrent accesses to this pool are serialized by a framework-specific
 * semaphore.
 *
 * The total number of mutexes that may be in use (created) at the same time
 * is 32.
 *
 * Mutexes shall not be recursive: consecutive calls to mutex_lock shall fail.
 *
 * @note ZEPHYR SPECIFIC:
 * - microKernel: mutexes shall be implemented as VxMicro resources.
 * - nanoKernel: mutexes shall be implemented as VxMicro semaphores.
 *
 * @param[out] err   execution status:
 *          - E_OS_OK:  mutex was created,
 *          - E_OS_ERR: all mutexes from the pool are already being used,
 *          - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 *
 * @return
 *     - Handler of the created mutex,
 *     - NULL if all allocated mutexes are already being used.
 */
extern T_MUTEX mutex_create(OS_ERR_TYPE* err);

/**
 * Deletes a mutex.
 *
 * Disables a mutex that was reserved using mutex_create. Deleting a mutex while
 * a task is waiting (or will wait) for it to be freed may create a deadlock.
 *
 * @warning This service may panic if err parameter is null and:
 * - mutex parameter is invalid, or
 * - when called from an ISR.
 *
 * Authorized execution levels: task, fiber.
 *
 * See also @ref mutex_create.
 *
 * @param mutex: handler of the mutex to delete (returned by @ref mutex_create).
 *
 * @param[out] err   execution status:
 *    - E_OS_OK:  mutex was deleted,
 *    - E_OS_ERR: mutex parameter is invalid, or mutex was not created,
 *    - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern void mutex_delete(T_MUTEX mutex, OS_ERR_TYPE* err );

/**
 * Unlocks/gives a mutex.
 *
 * @warning This service may panic if err parameter is null and:
 * - mutex parameter is invalid, or
 * - when called from an ISR.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @note ZEPHYR SPECIFIC:<br/>
 * Unlocking a free mutex will return E_OS_OK (because ZEPHYR primitives
 * do not return an execution status).
 *
 * @param mutex: handler of the mutex to unlock (value returned by mutex_create).
 *
 * @param[out]    err: execution status:
 *          - E_OS_OK:  mutex was unlocked,
 *          - E_OS_ERR: invalid parameter,
 *          - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern void mutex_unlock (T_MUTEX mutex, OS_ERR_TYPE* err);

/**
 * Locks/takes a mutex.
 *
 * This service may block while waiting on the mutex's availability,
 * depending on the timeout parameter.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @param mutex: handler of the mutex to lock (returned by @ref mutex_create).
 *
 * @param timeout: maximum number of milliseconds to wait for the mutex.
 *                 Special values OS_NO_WAIT and OS_WAIT_FOREVER may be used.
 *
 * @return execution status:
 *      - E_OS_OK: mutex was successfully taken,
 *      - E_OS_ERR: mut parameter is invalid (mutex was deleted or never created),
 *      - E_OS_ERR_TIMEOUT: could not take semaphore before timeout expiration,
 *      - E_OS_ERR_BUSY: could not take semaphore (did not wait),
 *      - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern OS_ERR_TYPE mutex_lock (T_MUTEX mutex, int timeout);

/**
 * @}
 *
 * @defgroup os_scheduling Scheduling
 * Defines control functions for enabling or disabling scheduling.
 *
 * The processor manages the scheduling between multiple tasks and interrupts,
 * based on their priorities.
 *
 * Temporarily disabling scheduling allows to perform critical portions of code
 * knowing that it won't be interrupted.
 *
 * Function name            | Task ctxt | Fiber ctxt| Interrupt |
 * -------------------------|:---------:|:---------:|:---------:|
 * @ref disable_scheduling  |     X     |     X     | No effect |
 * @ref enable_scheduling   |     X     |     X     | No effect |
 *
 * @{
 */

/**
 * Disables scheduling.
 *
 * Disables task preemption.
 *
 * @warning This service has no effect when called from an ISR context.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @note ZEPHYR SPECIFIC:<br/>
 * This service disables all interrupts.
 */
extern void disable_scheduling (void);

/**
 * Enables scheduling.
 *
 * Restores task preemption.
 *
 * @warning This service has no effect when called from an ISR context.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @note ZEPHYR SPECIFIC:<br/>
 * This service unmasks all interrupts that were masked by a previous
 * call to disable_scheduling.
 */
extern void enable_scheduling (void);

/**
 * @}
 *
 * @defgroup os_queue Queue
 * Defines control functions for message queues.
 *
 * Message Queues allow the services to send/receive messages from other
 * services.
 *
 * Function name            | Task ctxt | Fiber ctxt| Interrupt |
 * -------------------------|:---------:|:---------:|:---------:|
 * @ref queue_delete        |     X     |     X     |           |
 * @ref queue_create        |     X     |     X     |           |
 * @ref queue_get_message   |     X     |     X     |           |
 * @ref queue_send_message  |     X     |     X     |     X     |
 *
 * @{
 */

/**
 * Deletes a queue.
 *
 * Frees a whole queue.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @param queue The queue to free.
 * @param[out] err   execution status.
 *          - E_OS_OK : queue was successfully deleted,
 *          - E_OS_ERR: no queues from the pool are being used, or there were
 *                      errors while freeing this queue's elements;
 *          - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
void queue_delete(T_QUEUE queue, OS_ERR_TYPE* err);

/**
 * Creates a message queue.
 *
 * @warning This service may panic if err parameter is NULL and:
 * - no queue is available, or
 * - when called from an ISR.
 *
 * Authorized execution levels:  task, fiber.
 *
 * As for semaphores and mutexes, queues are picked from a pool of
 * statically-allocated objects.
 *
 * @param maxSize: maximum number of messages in the queue
 *                 (Rationale: queues only contain pointer to messages).
 *
 * @param[out] err   execution status:
 *          - E_OS_OK:  queue was created,
 *          - E_OS_ERR: all queues from the pool are already being used,
 *          - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 *
 * @return
 *    - Handler of the created queue,
 *    - NULL if all allocated queues are already being used.
 */
extern T_QUEUE queue_create(uint32_t maxSize, OS_ERR_TYPE* err);

/**
 * Reads a message from a queue.
 *
 * Reads and dequeues a message.
 *
 * @warning This service may panic if err parameter is NULL and:
 * - queue parameter is invalid, or
 * - message parameter is NULL, or
 * - when called from an ISR.
 *
 * Authorized execution levels:  task, fiber.
 *
 * @param queue:  handler of the queue (returned by @ref queue_create).
 *
 * @param[out] message: pointer to read message.
 *
 * @param timeout: maximum number of milliseconds to wait for the message.
 *                 Special values OS_NO_WAIT and OS_WAIT_FOREVER may be used.
 *
 * @param[out] err   execution status:
 *          - E_OS_OK:  a message was read,
 *          - E_OS_ERR_TIMEOUT: no message was received,
 *          - E_OS_ERR_EMPTY: the queue is empty,
 *          - E_OS_ERR: invalid parameter,
 *          - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern void queue_get_message (T_QUEUE queue, T_QUEUE_MESSAGE* message, int timeout, OS_ERR_TYPE* err);

/**
 * Sends a message on a queue.
 *
 * Sends / queues a message.
 *
 * @warning This service may panic if err parameter is NULL and:
 * - queue parameter is invalid, or
 * - the queue is already full.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param queue: handler of the queue (returned by @ref queue_create).
 *
 * @param[in] message:  pointer to the message to send.
 *
 * @param[out] err   execution status:
 *          - E_OS_OK:  a message was read,
 *          - E_OS_ERR_OVERFLOW: the queue is full (message was not posted),
 *          - E_OS_ERR: invalid parameter.
 */
extern void queue_send_message(T_QUEUE queue, T_QUEUE_MESSAGE message, OS_ERR_TYPE* err );

/**
 * @}
 *
 * @defgroup os_interrupts Interrupts
 * Defines control functions to manage interrupts.
 *
 * Interrupts can preempt the execution of a task, to perform peripheral data
 * processing, for example. They call a callback function, called
 * ISR (Interrupt Service Routine).
 *
 * Operations on interrupt control can be necessary for critical portions of
 * code.
 *
 * Function name            | Task ctxt | Fiber ctxt| Interrupt |
 * -------------------------|:---------:|:---------:|:---------:|
 * @ref interrupt_enable    |     X     |     X     |     X     |
 * @ref interrupt_disable   |     X     |     X     |     X     |
 * @ref interrupt_set_nmi   |     X     |     X     |     X     |
 * @ref interrupt_lock      |     X     |     X     |     X     |
 * @ref interrupt_unlock    |     X     |     X     |     X     |
 * @ref interrupt_set_isr   |     X     |     X     |           |
 *
 * @{
 */

/**
 * Enables an interrupt.
 *
 * Unmasks an interrupt request line.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param irq: interrupt request line number.
 */
extern void interrupt_enable(uint32_t irq);

/**
 * Disables an interrupt.
 *
 * Masks an interrupt request line.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param irq: interrupt request line number.
 */
extern void interrupt_disable(uint32_t irq);

/**
 * Routes interrupt line as NMI.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param irq: interrupt request line number.
 */
extern void interrupt_set_nmi(uint32_t irq);

/**
 * Disables all interrupts.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @return A key representing the IRQ that were enabled before calling this
 *         service.
 */
extern uint32_t interrupt_lock (void);

/**
 * Restores all interrupts.
 *
 * Re-enables the set of interrupts that were disabled by interrupt_disable_all.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param key: value returned by interrupt_disable_all.
 */
extern void interrupt_unlock (uint32_t key);

/**
 * Attaches an ISR to an IRQ.
 *
 * Attaches/connects an interrupt service routing to an interrupt request line.
 *
 * Specifying a NULL pointer, as isr parameter, will detach a previous ISR from
 * the IRQ.
 *
 * @warning This service may panic if err parameter is null and:
 * - irq parameter is invalid, or
 * - when called from an ISR.
 *
 * Authorized execution levels:  task, fiber.
 *
 * NB: IRQ priority is not a parameter used in quark context, as it is
 *     inferred from the IRQ number:
 * ~~~~~~~~~~~
 *     Priority = IRQ number / 16 ( rounded down )
 * ~~~~~~~~~~~
 *     --> See file loApicIntr.c
 *
 * @note ZEPHYR SPECIFIC:<br/>
 * ZEPHYR does not provide an API to detach an ISR from an IRQ.
 * When isr parameter is NULL, this function disables the IRQ (if valid)
 * instead of replacing the ISR pointer with zeros.
 *
 * @param irq:       interrupt request line number.
 * @param isr:       pointer on the interrupt service routine.
 * @param isrData:   pointer to the data passed as an argument to isr.
 * @param priority:  requested priority of interrupt.
 * @param[out] err   execution status:
 *       - E_OS_OK:  ISR was attached or detached to IRQ,
 *       - E_OS_ERR: irq parameter is invalid,
 *       - E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 */
extern void interrupt_set_isr (int irq, T_ENTRY_POINT isr, void* isrData , int priority, OS_ERR_TYPE* err);

/**
 * @}
 *
 * @defgroup os_time_timer Time and Timers
 * Defines control functions to get system time, and manager timers.
 *
 * The time functions return the elapsed time in milli-seconds or micro-seconds.
 *
 * The timers are used to perform an action after a specific amount of time.
 *
 * Function name       | Task ctxt | Fiber ctxt| Interrupt |
 * --------------------|:---------:|:---------:|:---------:|
 * @ref get_time_ms    |     X     |     X     |     X     |
 * @ref get_time_us    |     X     |     X     |     X     |
 * @ref timer_create   |     X     |     X     |     X     |
 * @ref timer_start    |     X     |     X     |     X     |
 * @ref timer_stop     |     X     |     X     |     X     |
 * @ref timer_delete   |     X     |     X     |     X     |
 *
 * @{
 */

/**
 * Gets the current tick in ms.
 *
 * Returns the current tick converted in milliseconds.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @return current tick converted in milliseconds.
 */
extern uint32_t get_time_ms (void);

/**
 * Gets the current tick in us.
 *
 * Returns the current tick converted in microseconds.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @return current tick converted in microseconds.
 */
extern uint64_t get_time_us (void);

/**
 * Creates a timer object.
 *
 * @warning This service may panic if err parameter is null and:
 * - callback parameter is null, or
 * - no timer is available.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param callback: pointer to the function to be executed.
 * @param privData: pointer to data that shall be passed to the callback.
 * @param delay: number of milliseconds between function executions.
 * @param repeat: specifies if the timer shall be re-started after each
                  execution of the callback.
 * @param startup:  specifies if the timer shall be start immediately.
 * @param[out] err   execution status:
 *        - E_OS_OK:  callback is programmed,
 *        - E_OS_ERR: no timer is available, or callback parameter is NULL.
 *
 * @return
 *       - Handler of the timer,
 *       - NULL if the service fails (e.g. no available timer
 *         or callback is a NULL pointer).
 */
T_TIMER timer_create(T_ENTRY_POINT callback, void* privData, uint32_t delay, bool repeat,bool startup , OS_ERR_TYPE* err);

/**
 * Starts the timer.
 *
 * @warning This service may panic if err parameter is null and:
 * - no timer is available.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param tmr:  handler of the timer (returned by @ref timer_create).
 * @param delay: number of milliseconds between function executions.
 * @param[out] err   execution status:
 *        - E_OS_OK:  timer is started,
 *        - E_OS_ERR: tmr parameter is null, invalid, or timer is running.
 */
void timer_start(T_TIMER tmr, uint32_t delay, OS_ERR_TYPE* err);

/**
 * Removes the timer in Chained list of timers.
 *
 * This service may panic if err parameter is null and:
 * - tmr parameter is null, invalid, or timer is not running.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param tmr:  handler of the timer (returned by @ref timer_create).
 * @param[out] err   execution status:
 *        - E_OS_OK:  timer is stopped,
 *        - E_OS_ERR: tmr parameter is null, invalid, or timer is not running.
 */
void timer_stop(T_TIMER tmr, OS_ERR_TYPE* err);

/**
 * Deletes the timer object.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param tmr:  handler of the timer (returned by @ref timer_create).
 * @param[out] err   execution status:
 *        - E_OS_OK:  timer is stopped and callback is disabled,
 *        - E_OS_ERR: tmr parameter is null, invalid.
 */
void timer_delete(T_TIMER tmr, OS_ERR_TYPE* err);


/**
 * @}
 *
 * @defgroup os_mem_alloc Memory Allocation
 * Defines balloc and bfree functions for dynamic memory allocation.
 *
 * Dynamic allocation requires a function to allocate and free memory.
 * If allocated memory is not freed, it causes a memory leak.
 *
 * Function name       | Task ctxt | Fiber ctxt| Interrupt |
 * --------------------|:---------:|:---------:|:---------:|
 * @ref balloc         |     X     |     X     |     X     |
 * @ref bfree          |     X     |     X     |     X     |
 *
 * @{
 */


/**
 * Reserves a block of memory.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * This function returns a pointer on the start of a reserved memory block
 * which size is equal or larger than the requested size.
 *
 * If there is not enough available memory, this function returns a null
 * pointer and sets "err" parameter to E_OS_ERR_NO_MEMORY, or panic
 * if "err" pointer is null.
 *
 * @warning This function may panic if err is null and
 * - size is null, or
 * - there is not enough available memory.
 *
 * @param size number of bytes to reserve.
 *
 * @param[out] err execution status:
 *    - E_OS_OK: block was successfully reserved,
 *    - E_OS_ERR: size is null,
 *    - E_OS_ERR_NO_MEMORY: there is not enough available memory.
 *
 * @return pointer to the reserved memory block
 *    or null if no block is available.
 */
extern void* balloc (uint32_t size, OS_ERR_TYPE* err);

/**
 * Frees a block of memory.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * This function frees a memory block that was reserved by malloc.
 *
 * The "buffer" parameter must point to the start of the reserved block
 * (i.e. it shall be a pointer returned by malloc).
 *
 * @param buffer pointer returned by malloc.
 *
 * @return execution status:
 *    - E_OS_OK: block was successfully freed,
 *    - E_OS_ERR: "buffer" param did not match any reserved block.
 */
extern OS_ERR_TYPE bfree(void* buffer);


/**
 * @}
 *
 * @defgroup os_task Task
 * Defines functions to manage tasks.
 *
 * Not all the functions can be called from interrupt context. Here is a
 * summary of the compatible contexts for each function.
 *
 * Function name            | Task ctxt | Fiber ctxt| Interrupt |
 * -------------------------|:---------:|:---------:|:---------:|
 * @ref task_create         |     X     |     X     |           |
 * @ref local_task_start    |     X     |     X     |           |
 * @ref local_task_suspend  |     X     |     X     |           |
 * @ref task_get_state      |     X     |     X     |           |
 * @ref local_task_sleep    |     X     |     X     |           |
 *
 * @{
 */


/**
 * Creates a task.
 *
 * The task may either be automatically started, or created
 * in suspended state.
 *
 * The tasks are not dynamically created, but picked from a pool that is
 * defined by the configuration of the OS Abstraction Layer (i.e. at
 * compilation time). Hence it is not possible to select the size of
 * the task stack size at runtime.
 *
 * @param[in] entryPoint:  address of the task's entry point.
 *
 * @param entryPointData:  data passed to entry point when task is started.
 *
 * @param[out] stackPtr:   address of task's stack.
 *
 * @param stackSize: task stack's size - if 0, selected by the OS abstraction.
 *
 * @param prio: priority level of the task.
 *
 * @param state: state of the task after creation (running or suspended).
 *
 * @param taskName: name of the task.
 *
 * @return
 *     - Handler of the created task.
 *     - NULL if all allocated tasks are already being used.
 */
extern T_TASK task_create (T_ENTRY_POINT entryPoint,
                            void* entryPointData, void* stackPtr,
                            uint32_t stackSize, T_TASK_PRIO prio,
                            T_TASK_STATE state, const char *taskName );

/**
 * Starts or resumes a suspended task.
 *
 * @param task: handler of the task (returned by @ref task_create).
 *
 * @return execution status:
 *      - E_OS_OK: task was successfully (re)started,
 *      - E_OS_ERR: task parameter is invalid,
 *      - E_OS_ERR_BUSY: task is already running.
 */
extern OS_ERR_TYPE local_task_start (T_TASK task);

/**
 * Suspends another task.
 *
 * Use task_start to resume a suspended task.
 *
 * @param task: handler of the task (returned by @ref task_create).
 *
 * @return execution status:
 *      - E_OS_OK: task was successfully suspended,
 *      - E_OS_ERR: task parameter is invalid,
 *      - E_OS_ERR_BUSY: task is already suspended.
 */
extern OS_ERR_TYPE local_task_suspend (T_TASK task);

/**
 * Gets the execution state of a task.
 *
 * @param task: handler of the task (returned by @ref task_create).
 *
 * @return execution status:
 *      - E_TASK_RUNNING: task is active,
 *      - E_TASK_SUSPENDED: task is suspended,
 *      - E_TASK_UNCREATED: task was never created, or task parameter is invalid.
 */
extern T_TASK_STATE  task_get_state (T_TASK task);

/**
 * Releases the execution of the current task for a limited time.
 *
 * @param time: number of milliseconds to sleep, negative values (e.g. OS_WAIT_FOREVER)
 *              shall be ignored ( the function shall return immediately).
 */
extern void local_task_sleep (int time);

/**
 * @}
 */

/**
 * Initializes the OS abstraction layer.
 *
 */
extern void os_init (void);

/**
 * @}
 */

#endif
