/* mvic.c - Mint Valley Interrupt Controller (MVIC) */

/*
 * Copyright (c) 1997-1998, 2000-2002, 2004, 2006-2008, 2011-2015 Wind River
 * Systems, Inc.
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

This module is based on the standard Local APIC and IO APIC source modules.
This modules combines these modules into one source module that exports the
same APIs defined by the Local APIC and IO APIC header modules. These
routine have been adapted for the Mint Valley Interrupt Controller which has
a cutdown implementation of the Local APIC & IO APIC register sets.

The MVIC (Mint Valley Interrupt Controller) is configured by default
to support 32 external interrupt lines.
Unlike the traditional IA LAPIC/IOAPIC, the interrupt vectors in MVIC are fixed
and not programmable.
The larger the vector number, the higher the priority of the interrupt.
Higher priority interrupts preempt lower priority interrupts.
Lower priority interrupts do not preempt higher priority interrupts.
The MVIC holds the lower priority interrupts pending until the interrupt
service routine for the higher priority interrupt writes to the End of
Interrupt (EOI) register.
After an EOI write, the MVIC asserts the next highest pending interrupt.

INCLUDE FILES: ioapic.h loapic.h

*/

/* includes */

#include <nanokernel.h>
#include <arch/cpu.h>
#include <misc/__assert.h>

#include "board.h"

#include <toolchain.h>
#include <sections.h>

#include <drivers/ioapic.h> /* IO APIC public API declarations. */
#include <drivers/loapic.h> /* Local APIC public API declarations.*/

/* defines */

/* IO APIC direct register offsets */

#define IOAPIC_IND 0x00   /* Index Register - MVIC IOREGSEL register */
#define IOAPIC_DATA 0x10  /* IO window (data) - pc.h */

/* MVIC IOREGSEL register usage defines */
#define MVIC_LOW_NIBBLE_MASK 0x07
#define MVIC_HIGH_NIBBLE_MASK 0x18

/* MVIC Local APIC Vector Table Bits */

#define LOAPIC_VECTOR 0x000000ff /* vectorNo */

/* MVIC Local APIC Spurious-Interrupt Register Bits */

#define LOAPIC_ENABLE 0x100	/* APIC Enabled */

/* forward declarations */

static void MvicRteSet(unsigned int irq, uint32_t value);
static uint32_t MvicRteGet(unsigned int irq);
static void MvicRteUpdate(unsigned int irq, uint32_t value,
					uint32_t mask);

/*
 * The functions irq_enable() and irq_disable() are implemented
 * in the BSPs that incorporate this interrupt controller driver due to the
 * IRQ virtualization imposed by the BSP.
 */

/*******************************************************************************
*
* _mvic_init - initialize the MVIC IO APIC and local APIC register sets.
*
* This routine initializes the Mint Valley Interrupt Controller (MVIC).
* This routine replaces the standard Local APIC / IO APIC init routines.
*
* RETURNS: N/A
*/

void _mvic_init(void)
{
	int32_t ix;	/* Interrupt line register index */
	uint32_t rteValue; /* value to copy into interrupt line register */

	/*
	 * The BSP must define the IOAPIC_NUM_RTES macro to indicate the number
	 * of redirection table entries supported by the IOAPIC on the board.
	 *

	 * By default mask all interrupt lines and set default sensitivity to edge.
	 *
	 */

	rteValue = IOAPIC_EDGE | IOAPIC_INT_MASK;

	for (ix = 0; ix < IOAPIC_NUM_RTES; ix++) {
		MvicRteSet(ix, rteValue);
	}

	/* enable the MVIC Local APIC */

	_loapic_enable();

	/* reset the TPR, and TIMER_ICR */

	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_TPR) = (int)0x0;
	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_TIMER_ICR) = (int)0x0;

	/* program Local Vector Table for the Virtual Wire Mode */

	/* lock the MVIC timer interrupt */

	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_TIMER) = LOAPIC_LVT_MASKED;

	/* discard a pending interrupt if any */

	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_EOI) = 0;

}

/*******************************************************************************
*
* _ioapic_eoi - send EOI (End Of Interrupt) signal to IO APIC
*
* This routine sends an EOI signal to the IO APIC's interrupting source.
*
* All line interrupts on Mint Valley are EOI'ed with local APIC EOI register.
*
* RETURNS: N/A
*/

void _ioapic_eoi(unsigned int irq /* INT number to send EOI */
			  )
{
	_loapic_eoi(irq);
}

/*******************************************************************************
*
* _ioapic_eoi_get - get EOI (End Of Interrupt) information
*
* This routine returns EOI signalling information for a specific IRQ.
*
* RETURNS: address of routine to be called to signal EOI;
*          as a side effect, also passes back indication if routine requires
*          an interrupt vector argument and what the argument value should be
*/

void *_ioapic_eoi_get(unsigned int irq,  /* INTIN number of interest */
		      char *argRequired, /* ptr to "argument required" result
					    area */
		      void **arg /* ptr to "argument value" result area */
		      )
{

	/* indicate that an argument to the EOI handler is required */

	*argRequired = 1;

	/*
	 * The parameter to the ioApicIntEoi() routine is the vector programmed
	 * into the redirection table.  The BSPs _SysIntVecAlloc() routine
	 * must invoke _IoApicIntEoiGet() after _IoApicRedVecSet() to ensure the
	 * redirection table contains the desired interrupt vector.
	 *
	 * Vectors fixed on this CPU Arch, no memory location on this CPU
	 * arch with this information.
	 */

	*arg = NULL;


	/* lo eoi always used on this CPU arch. */

	return _loapic_eoi;
}

/*******************************************************************************
*
* _ioapic_irq_enable - enable a specified APIC interrupt input line
*
* This routine enables a specified APIC interrupt input line.
*
* RETURNS: N/A
*/

void _ioapic_irq_enable(unsigned int irq /* INTIN number to enable */
				 )
{
	MvicRteUpdate(irq, 0, IOAPIC_INT_MASK);
}

/*******************************************************************************
*
* _ioapic_irq_disable - disable a specified APIC interrupt input line
*
* This routine disables a specified APIC interrupt input line.
*
* RETURNS: N/A
*/

void _ioapic_irq_disable(unsigned int irq /* INTIN number to disable */
				  )
{
	MvicRteUpdate(irq, IOAPIC_INT_MASK, IOAPIC_INT_MASK);
}

/*******************************************************************************
*
* _ioapic_irq_set - programs Rte interrupt line register.
*
* Always mask interrupt as part of programming like standard IOAPIC
* version of this routine.
* Vector is fixed by this HW and is unused.
* Or in flags for trigger bit.
*
* RETURNS: N/A
*/
void _ioapic_irq_set(unsigned int irq, /* virtualized IRQ */
		     unsigned int vector, /* vector number */
		     uint32_t flags    /* interrupt flags */
		     )
{
	uint32_t rteValue;   /* value to copy into Rte register */

	ARG_UNUSED(vector);

	rteValue = IOAPIC_INT_MASK | flags;
	MvicRteSet(irq, rteValue);
}

/*******************************************************************************
*
* _ioapic_int_vec_set - program interrupt vector for specified irq
*
* Fixed vector on this HW. Nothing to do.
*
* RETURNS: N/A
*/
void _ioapic_int_vec_set(unsigned int irq, /* INT number */
				  unsigned int vector /* vector number */
				  )
{
}

/*******************************************************************************
*
* _loapic_enable - enable the MVIC Local APIC
*
* This routine enables the MVIC Local APIC.
*
* RETURNS: N/A
*/

void _loapic_enable(void)
{
	int32_t oldLevel = irq_lock(); /* LOCK INTERRUPTS */

	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_SVR) |= LOAPIC_ENABLE;

	irq_unlock(oldLevel); /* UNLOCK INTERRUPTS */
}

/*******************************************************************************
*
* _loapic_disable - disable the MVIC Local APIC.
*
* This routine disables the MVIC Local APIC.
*
* RETURNS: N/A
*/

void _loapic_disable(void)
{
	int32_t oldLevel = irq_lock(); /* LOCK INTERRUPTS */

	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_SVR) &= ~LOAPIC_ENABLE;

	irq_unlock(oldLevel); /* UNLOCK INTERRUPTS */
}

/*******************************************************************************
*
* _loapic_eoi -  send EOI (End Of Interrupt) signal to MVIC Local APIC
*
* This routine sends an EOI signal to the MVIC Local APIC's interrupting source.
*
* RETURNS: N/A
*/

void _loapic_eoi(unsigned int irq)
{
	ARG_UNUSED(irq);
	*(volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_EOI) = 0;
}

/*******************************************************************************
*
* _loapic_int_vec_set - set the vector field in the specified RTE
*
* This routine is utilized by the BSP provided routined _SysIntVecAllocate()
* which in turn is provided to support the irq_connect() API.  Once
* a vector has been allocated, this routine is invoked to update the LVT
* entry associated with <irq> with the vector.
*
* RETURNS: N/A
*/

void _loapic_int_vec_set(unsigned int irq, /* IRQ number of the
										   interrupt */
										   unsigned int vector /* vector to copy
															   into the LVT */
															   )
{
	volatile int *pLvt; /* pointer to local vector table */
	int32_t oldLevel;   /* previous interrupt lock level */

	/*
	* irq is actually an index to local APIC LVT register.
	* ASSERT if out of range for MVIC implementation.
	*/
	__ASSERT_NO_MSG(irq < LOAPIC_IRQ_COUNT);

	/*
	* The following mappings are used:
	*
	*   LVT0 -> LOAPIC_TIMER
	*
	* It's assumed that LVTs are spaced by LOAPIC_LVT_REG_SPACING bytes
	*/

	pLvt = (volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_TIMER + (irq * LOAPIC_LVT_REG_SPACING));

	/* update the 'vector' bits in the LVT */

	oldLevel = irq_lock();
	*pLvt = (*pLvt & ~LOAPIC_VECTOR) | vector;
	irq_unlock(oldLevel);

}

/*******************************************************************************
*
* _loapic_irq_enable - enable an individual LOAPIC interrupt (IRQ)
*
* This routine clears the interrupt mask bit in the LVT for the specified IRQ
*
* RETURNS: N/A
*/

void _loapic_irq_enable(unsigned int irq /* IRQ number of
										 the interrupt */
										 )
{
	volatile int *pLvt; /* pointer to local vector table */
	int32_t oldLevel;   /* previous interrupt lock level */

	/*
	* irq is actually an index to local APIC LVT register.
	* ASSERT if out of range for MVIC implementation.
	*/
	__ASSERT_NO_MSG(irq < LOAPIC_IRQ_COUNT);

	/*
	* See the comments in _LoApicLvtVecSet() regarding IRQ to LVT mappings
	* and ths assumption concerning LVT spacing.
	*/

	pLvt = (volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_TIMER + (irq * LOAPIC_LVT_REG_SPACING));

	/* clear the mask bit in the LVT */

	oldLevel = irq_lock();
	*pLvt = *pLvt & ~LOAPIC_LVT_MASKED;
	irq_unlock(oldLevel);

}

/*******************************************************************************
*
* _loapic_irq_disable - disable an individual LOAPIC interrupt (IRQ)
*
* This routine clears the interrupt mask bit in the LVT for the specified IRQ
*
* RETURNS: N/A
*/

void _loapic_irq_disable(unsigned int irq /* IRQ number of the
										  interrupt */
										  )
{
	volatile int *pLvt; /* pointer to local vector table */
	int32_t oldLevel;   /* previous interrupt lock level */

	/*
	* irq is actually an index to local APIC LVT register.
	* ASSERT if out of range for MVIC implementation.
	*/
	__ASSERT_NO_MSG(irq < LOAPIC_IRQ_COUNT);

	/*
	* See the comments in _LoApicLvtVecSet() regarding IRQ to LVT mappings
	* and ths assumption concerning LVT spacing.
	*/

	pLvt = (volatile int *)(LOAPIC_BASE_ADRS + LOAPIC_TIMER + (irq * LOAPIC_LVT_REG_SPACING));

	/* set the mask bit in the LVT */

	oldLevel = irq_lock();
	*pLvt = *pLvt | LOAPIC_LVT_MASKED;
	irq_unlock(oldLevel);

}

/*******************************************************************************
*
* MvicRteGet - read a 32 bit MVIC IO APIC register
*
* RETURNS: register value
*/
static uint32_t MvicRteGet(unsigned int irq /* INTIN number */
	)
{
	uint32_t value; /* value */
	int key;	/* interrupt lock level */
	volatile unsigned int *rte;
	volatile unsigned int *index;
	unsigned int low_nibble;
	unsigned int high_nibble;

	index = (unsigned int *)(IOAPIC_BASE_ADRS + IOAPIC_IND);
	rte = (unsigned int *)(IOAPIC_BASE_ADRS + IOAPIC_DATA);

	/* Set index in the IOREGSEL */
	__ASSERT(irq < IOAPIC_NUM_RTES, "INVL");

	low_nibble = ((irq & MVIC_LOW_NIBBLE_MASK) << 0x1);
	high_nibble = ((irq & MVIC_HIGH_NIBBLE_MASK) << 0x2);

	/* lock interrupts to ensure indirect addressing works "atomically" */

	key = irq_lock();

	*(index) = high_nibble | low_nibble;
	value = *(rte);

	irq_unlock(key);

	return value;
}

/*******************************************************************************
*
* MvicRteSet -  write to 32 bit MVIC IO APIC register
*
*
* RETURNS: N/A
*/

static void MvicRteSet(unsigned int irq, /* INTIN number */
	uint32_t value  /* value to be written */
	)
{
	int key; /* interrupt lock level */
	volatile unsigned int *rte;
	volatile unsigned int *index;
	unsigned int low_nibble;
	unsigned int high_nibble;

	index = (unsigned int *)(IOAPIC_BASE_ADRS + IOAPIC_IND);
	rte = (unsigned int *)(IOAPIC_BASE_ADRS + IOAPIC_DATA);

	/* Set index in the IOREGSEL */
	__ASSERT(irq < IOAPIC_NUM_RTES, "INVL");

	low_nibble = ((irq & MVIC_LOW_NIBBLE_MASK) << 0x1);
	high_nibble = ((irq & MVIC_HIGH_NIBBLE_MASK) << 0x2);

	/* lock interrupts to ensure indirect addressing works "atomically" */

	key = irq_lock();

	*(index) = high_nibble | low_nibble;
	*(rte) = (value & IOAPIC_LO32_RTE_SUPPORTED_MASK);

	irq_unlock(key);
}

/*******************************************************************************
*
* MvicRteUpdate - modify interrupt line register.
*
*
* RETURNS: N/A
*/

static void MvicRteUpdate(
	unsigned int irq, /* INTIN number */
	uint32_t value,   /* value to be written */
	uint32_t mask     /* mask of bits to be modified */
	)
{
	MvicRteSet(irq, (MvicRteGet(irq) & ~mask) | (value & mask));
}

