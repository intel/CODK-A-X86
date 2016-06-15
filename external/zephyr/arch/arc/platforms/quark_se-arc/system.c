/* system.c - system/hardware module for quark_se-arc BSP */

/*
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
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

/*
DESCRIPTION
This module provides routines to initialize and support board-level hardware
for the quark_se-arc BSP.
*/

#include <nanokernel.h>
#include <board.h>
#include <init.h>
#include <drivers/uart.h>

/* Cannot use microkernel, since only nanokernel is supported */
#if defined(CONFIG_MICROKERNEL)
#error "Microkernel support is not available"
#endif


#if defined(CONFIG_PRINTK) || defined(CONFIG_STDOUT_CONSOLE)
#include <console/uart_console.h>


#define MBX(_offset_) (*(volatile unsigned int *) (0xb0800a60 + (_offset_)))

int _mbxPollOut(int data) {
    //while((* (volatile unsigned int *)(0xB0800000 + 0xA60 + 20)) & 1)
    //    ; // STS
    int flags = irq_lock_inline();
    MBX(4) = (unsigned int)data; //DAT0
    MBX(8) = 1; // DAT1
    MBX(0) = 0x80000000;// CTRL
    while(!MBX(20) & 1)
        ;

    while(MBX(20) & 1)
        ; // STS
    irq_unlock_inline(flags);
    /* If end of line, delay to give quark
     * a chance to do some processing */
    if (data == '\r') {
        volatile int count = 10000;
        while (count --);
    }
    return data;
}

/**
 *
 * @brief initialize target-only console
 *
 * Only used for debugging, no host driver involved.
 *
 * RETURNS: N/A
 *
 */
extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));

static void consoleInit(void)
{
	__printk_hook_install(_mbxPollOut);
	__stdout_hook_install(_mbxPollOut);
}

#else
#define consoleInit() do { /* do nothing */ } while ((0))
#endif /* defined(CONFIG_PRINTK) || defined(CONFIG_STDOUT_CONSOLE) */

/**
 *
 * @brief perform basic hardware initialization
 *
 * Hardware initialized:
 * - interrupt unit
 * - serial port and console driver
 *
 * RETURNS: N/A
 */
static int qrk_se_init(struct sys_device *arg)
{
        ARG_UNUSED(arg);

	_arc_v2_irq_unit_init();
        consoleInit(); /* NOP if not needed */
        return 0;
}
DECLARE_DEVICE_INIT_CONFIG(qrk_se_0, "", qrk_se_init, NULL);
pure_early_init(qrk_se_0, NULL);
