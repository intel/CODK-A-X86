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
 * Uses VxMicro-specific definitions to implement generic framework definitions
 */

#ifndef _OS_SPECIFIC_H_
#define _OS_SPECIFIC_H_


#include "zephyr/os_config.h"


/*
 * Include generated file autoconf.h to retrieve the definitions from the kernel's configuration.
 * Notably whether it is a nano kernel or micro kernel
 */
//#include "autoconf.h"

/*------------------------------------------------*/


#include "nanokernel.h"

#ifdef CONFIG_MICROKERNEL
#include "microkernel.h"
#include "zephyr.h"
#endif

#include <misc/printk.h>


/***********************************************************
 ************** Generic utilities  *************************
 ***********************************************************/
#ifdef __OS_ABSTRACTION_DEBUG
#define _log(_fmt_, args...) printk(_fmt_, ##args)
#else
#define _log(_fmt_, args...)
#endif





/***********************************************************
 ************** Execution level detection  *****************
 ***********************************************************/

/**
 * \enum T_EXEC_LEVEL
 *  \brief Identifier for the level of execution
 */
typedef enum {
    E_EXEC_LVL_ISR   = NANO_CTX_ISR,
    E_EXEC_LVL_FIBER = NANO_CTX_FIBER,
    E_EXEC_LVL_TASK  = NANO_CTX_TASK,
    E_EXEC_LVL_UNKNOWN
} T_EXEC_LEVEL;


/**
 * \brief Returns the current execution level
 *   _getExecLevel returns the current execution level: task, fiber or isr
 */
#define _getExecLevel() ( (T_EXEC_LEVEL) context_type_get() )



/***********************************************************
 ******************* OS Timer macros  **********************
 ***********************************************************/
/**
 * Tick macros
 */

#define CONVERT_MS_TO_TICKS(ms)        (((ms) * sys_clock_ticks_per_sec) / 1000 )
#define CONVERT_TICKS_TO_MS(ticks)     ((ticks) * (sys_clock_us_per_tick / 1000))

#define CONVERT_US_TO_TICKS(us)        ( (us) / sys_clock_us_per_tick )
#define CONVERT_TICKS_TO_US(ticks)     ( (ticks) * sys_clock_us_per_tick)


/* INFORMATION:
 * ------------
 * _GET_TICK() returns current Tick value on 32 bits.
 * Therefore, @ 16MHz, according to the nano_tick_get_32() or task_tick_get_32()
 * routines, ticktime and tickunit values set in NODE1.c,
 * overflow will occur in:
 *
 * nano_tick_get_32():tick is 10ms so about 497 days.
 * task_tick_get_32():tick is 1ms so about 49 days.
 */
#ifdef CONFIG_NANOKERNEL
#define _GET_TICK()             ( nano_tick_get_32() )
#elif CONFIG_MICROKERNEL
int32_t convert_milliseconds_to_ticks(int time_in_ms);
#define _GET_TICK()             ( task_tick_get_32() )
#endif

#endif /* _OS_SPECIFIC_H_ */
