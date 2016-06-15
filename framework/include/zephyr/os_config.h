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

/**
 * Defines the configuration parameters
 * of the OS Abstraction layer for VxMicro.
 */

#ifndef __OS_CONFIG__
#define __OS_CONFIG__

#define VXMICRO_OS_ABSTRACTION_USE_SINGLE_POOL_LOCK

/* ------------ Settings for the QUEUES */
#define QUEUE_POOL_SIZE             (10)    /** Total number of queue */

#ifndef CONFIG_ARC_OS_UNIT_TESTS
#define QUEUE_ELEMENT_POOL_SIZE     (100)   /** Total number of element shared by the queues */
/*------------ Settings for the TIMERS */
#define TIMER_POOL_SIZE              20   /* Total number of timers provided by the abstraction layer */

#else
#define QUEUE_ELEMENT_POOL_SIZE     (30)   /** Total number of element shared by the queues */
/*------------ Settings for the TIMERS */
#define TIMER_POOL_SIZE              8   /* Total number of timers provided by the abstraction layer */
#endif

#define TIMER_CBK_TASK_STACK_SIZE   600   /* Size in bytes of the stack for the Timer callback task -- NOT USED BY ZEPHYR MICROKERNEL */
#define TIMER_CBK_TASK_PRIORITY      10   /* Priority of the Timer callback task  -- NOT USED BY ZEPHYR MICROKERNEL */
#define TIMER_CBK_TASK_OPTIONS        0   /* Fiber options for the Timer callback task - 0 -> floating point is not supported  -- NOT USED BY ZEPHYR MICROKERNEL */

/*------------ Settings for the INTERRUPTS */


/*-------- Settings for the SYNC. OBJECTS */
#define SEMAPHORE_POOL_SIZE          32 /** Total number of semaphores */
#define MUTEX_POOL_SIZE              32 /** Total number of mutexes */


/*------------ Settings for the MEMORY BLOCKS */

/** If defined, allow to use a block larger than required when all smaller blocks are already reserved */
#define MALLOC_ALLOW_OUTCLASS

/*-------- DEBUG settings */
#define __OS_ABSTRACTION_DEBUG  /* If defined, enable debug logs */

#ifdef __OS_ABSTRACTION_DEBUG
//#define __DEBUG_OS_ABSTRACTION_TIMER
#define __DEBUG_OS_ABSTRACTION_INTERRUPT
#define __DEBUG_OS_ABSTRACTION_SYNC
#endif





#endif
