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
 * @defgroup os_zephyr Zephyr OS Abstraction Layer
 * @ingroup os
 * @{
 */

/**
 * \file interrupt.c
 *
 * ZEPHYR OS abstraction / interrupt services
 *
 * Functions are exported by os.h
 *
 */


/******* Framework headers : */
#include "os/os.h"             /* framework export definitions */
#include "os/os_types.h"       /* framework-specific types */
#include "zephyr/os_specific.h" /* zephyr-specific definitions */
#include "zephyr/common.h"


/******* ZEPHYR headers : */

#include "nanokernel.h"   /* ZEPHYR nanokernel API declaration */

#ifdef   CONFIG_MICROKERNEL
#include "mutex.h"

#include "zephyr.h"
#endif

#include "board.h"
#if defined(CONFIG_ARC) && defined(CONFIG_QEMU_X86)
#include "drivers/io_config.h"
#endif
#if defined(CONFIG_QUARK)
#include "drivers/ioapic.h"
#endif

/********************/



/**********************************************************
 ************** Extern variables   ************************
 **********************************************************/

/**********************************************************
 ************** Local definitions  ************************
 **********************************************************/
#if defined(CONFIG_ARC) && defined(CONFIG_QEMU_X86)
#define NB_HW_INTERRUPTS      IOAPIC_NUM_RTES /* from board.h */
#define MIN_USER_IRQ          INT_VEC_IRQ0    /* from board.h */
#define MAX_USER_IRQ          ( MIN_USER_IRQ + NB_HW_INTERRUPTS )  /* defines the IRQ number of the last (sw) interrupt */
#elif CONFIG_ARC
#define MIN_USER_IRQ           IRQ_TIMER0  /* from board.h */
#define MAX_USER_IRQ           IRQ_ALWAYS_ON_GPIO /* defines the IRQ number of the last (sw) interrupt */
#else
#define NB_HW_INTERRUPTS      IOAPIC_NUM_RTES /* from board.h */
#define MIN_USER_IRQ          INT_VEC_IRQ0    /* from board.h */
#define MAX_USER_IRQ          ( MIN_USER_IRQ + NB_HW_INTERRUPTS )  /* defines the IRQ number of the last (sw) interrupt */
#endif
/**********************************************************
 ************** Private variables  ************************
 **********************************************************/
#ifdef CONFIG_NANOKERNEL
#else /* CONFIG_MICROKERNEL */
#endif

#ifdef CONFIG_X86_32
// static NANO_CPU_INT_STUB_DECL (g_InterruptStubPlaceHolder[NB_HW_INTERRUPTS]);
#endif

/**********************************************************
 ************** Forward declarations **********************
 **********************************************************/

/**********************************************************
 ************** Private functions  ************************
 **********************************************************/



/**********************************************************
 ************** Exported functions ************************
 **********************************************************/

/*----- Initialization  */


/**
 * Initialize the resources used by the framework's timer services.
 *
 * IMPORTANT : this function must be called during the initialization
 *             of the OS abstraction layer.
 *             this function shall only be called once after reset, otherwise
 *             it may cause the take/lock and give/unlock services to fail
 */
void framework_init_interrupt (void)
{
    /* g_InterruptStubPlaceHolder need not
     * to be initialized.
     *
     * Nothing to do here.
     *  */
}





/**
 * Unmask an interrupt request line.
 *  Authorized execution levels:  task, fiber, ISR.
 *
 * \param irq: interrupt request line number
 */
void interrupt_enable(uint32_t irq)
{
#if defined(CONFIG_ARC) && defined(CONFIG_QEMU_X86)
    irq = irq - IO_ADC0_INT_ERR;
#endif
    irq_enable(irq);
}

/**
 * Mask an interrupt request line.
 *  Authorized execution levels:  task, fiber, ISR.
 *
 * @param irq: interrupt request line number
 */
void interrupt_disable(uint32_t irq)
{
#if defined(CONFIG_ARC) && defined(CONFIG_QEMU_X86)
    irq = irq - IO_ADC0_INT_ERR;
#endif
    irq_disable(irq);
}

/**
 * Routes interrupt line as NMI.
 *
 * Authorized execution levels:  task, fiber, ISR.
 *
 * @param irq: interrupt request line number
 */
void interrupt_set_nmi(uint32_t irq)
{
#if defined(CONFIG_QUARK)
    _ioapic_irq_set(irq,(INT_VEC_IRQ0+irq),IOAPIC_NMI);
#endif
}


/**
 * Disable all (unmasked) interrupts.
 *  Authorized execution levels:  task, fiber, ISR.
 *
 * @return a key representing the IRQ that were enabled before calling this service.
 */
uint32_t interrupt_lock (void)
{
    return (irq_lock_inline());
}

/**
 * Enable the set of interrupts that were disabled by interrupt_disable_all.
 *  Authorized execution levels:  task, fiber, ISR.
 *
 * @param key: value returned by interrupt_disable_all.
 *
 */
void interrupt_unlock (uint32_t key)
{
    irq_unlock_inline (key);
}


/**
 * Attach an ISR to an IRQ.
 *
 *    Attach/connect an interrupt service routing to an interrupt request line.
 *    Specifying a NULL pointer, as isr parameter, will detach a previous ISR from
 *    the IRQ.
 *    This service may panic if err parameter is null and:
 *        irq parameter is invalid, or
 *        when called from an ISR.
 *  Authorized execution levels:  task, fiber.
 *
 *
 *  - ZEPHYR does not provide an API to detach an ISR from an IRQ.
 *     When isr parameter is NULL, this function disables the IRQ (if valid)
 *     instead of replacing the ISR pointer with zeros.
 *
 * @param irq:       interrupt request line number.
 * @param isr:       pointer on the interrupt service routine
 * @param isrData:   pointer to the data passed as an argument to isr
 * @param priority:  requested priority of interrupt
 * @param err (out): execution status:
 *        E_OS_OK : ISR was attached or detached to IRQ
 *        E_OS_ERR: irq parameter is invalid
 *        E_OS_ERR_NOT_ALLOWED: service cannot be executed from ISR context.
 *
 *  NB: IRQ priority is not a parameter used in quark context, as it is inferred from the IRQ number:
 *     Priority = IRQ number / 16 ( rounded down )
 *     --> \ref loApicIntr.c
 */
void interrupt_set_isr (int irq, T_ENTRY_POINT isr, void* isrData , int priority, OS_ERR_TYPE* err)
{
    int32_t vxErr=0;

    if ( E_EXEC_LVL_ISR != _getExecLevel() )
    {
#if defined(CONFIG_QUARK)
	irq = irq + MIN_USER_IRQ;
#endif
#if defined(CONFIG_ARC) && defined(CONFIG_QEMU_X86)
	irq = irq - IO_ADC0_INT_ERR + MIN_USER_IRQ;
#endif

        if (( irq >= MIN_USER_IRQ ) && ( irq <= MAX_USER_IRQ ))
        {
            if (NULL != isr )
            {
                /* attach isr to irq */
#ifdef CONFIG_QUARK
                irq = irq - MIN_USER_IRQ ;
#endif
#if defined(CONFIG_ARC) && defined(CONFIG_QEMU_X86)
                irq = irq - MIN_USER_IRQ ;
#endif
#if defined(CONFIG_ARC) && !defined(CONFIG_QEMU_X86)
                vxErr = irq_connect( irq, priority, isr, isrData);
#else
                vxErr = irq_connect( irq, (irq  + MIN_USER_IRQ)/ 16 , isr, isrData);
#endif
                if ( -1 == vxErr )
                {
                    error_management(err, E_OS_ERR);
                }
                else
                {
                    error_management(err, E_OS_OK);
                }
            }
            else
            { /* "detach isr from irq" :
             * ZEPHYR does not provide an API to detach dynamic ISRs,
             * => disable the IRQ  */
#if defined(CONFIG_QUARK)
                irq = irq - MIN_USER_IRQ ;
#endif
                irq_disable(irq);
            }
        }
        else
        {/* invalid irq number */
            error_management(err, E_OS_ERR);
        }
    }
    else
    { /* service is not allowed in ISR context */
        error_management(err, E_OS_ERR_NOT_ALLOWED);
    }
}

/** @} */
