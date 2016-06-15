/* board.h - board configuration macros for the Mint Valley BSP */

/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
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
This header file is used to specify and describe board-level aspects for
the Mint Valley BSP.
*/

#ifndef __INCboardh
#define __INCboardh

#include <stdint.h>
#include <misc/util.h>
#include <drivers/uart.h>
#include <drivers/ioapic.h>

#define INT_VEC_IRQ0  0x20 /* Vector number for IRQ0 */
#define FIXED_HARDWARE_IRQ_TO_VEC_MAPPING(x) (INT_VEC_IRQ0 + x)
#define IOAPIC_LO32_RTE_SUPPORTED_MASK (IOAPIC_INT_MASK | IOAPIC_TRIGGER_MASK)

/*
 * IO APIC (IOAPIC) device information (Intel ioapic)
 */
#define IOAPIC_NUM_RTES         32              /* Number of IRQs = 32 */

#define IOAPIC_BASE_ADRS_PHYS   0xFEC00000      /* base physical address */
#define IOAPIC_SIZE             MB(1)
#define IOAPIC_BASE_ADRS        IOAPIC_BASE_ADRS_PHYS

/*
 * Local APIC (LOAPIC) device information (Intel loapic)
 */

#define LOAPIC_BASE_ADRS_PHYS   0xFEE00000      /* base physical address */
#define LOAPIC_SIZE             KB(4)
#define LOAPIC_BASE_ADRS        LOAPIC_BASE_ADRS_PHYS
#define LOAPIC_TIMER_VEC	    CONFIG_MVIC_LOAPIC_TIMER_VEC
#define LOAPIC_TIMER_VEC_LIMIT	((INT_VEC_IRQ0 + 16) - 1)
#if ((LOAPIC_TIMER_VEC < INT_VEC_IRQ0) || (LOAPIC_TIMER_VEC > LOAPIC_TIMER_VEC_LIMIT))
#error CONFIG_MVIC_LVTTIMER_VECTOR out of valid range.
#endif
#define LOAPIC_TIMER_IRQ	    (CONFIG_MVIC_LOAPIC_TIMER_VEC - INT_VEC_IRQ0)
#define LOAPIC_TIMER_INT_PRI 0 /* Ignored for this BSP but required for OS build. */
#define LOAPIC_IRQ_BASE			LOAPIC_TIMER_IRQ
#define LOAPIC_IRQ_COUNT		1
#define LOAPIC_LVT_REG_SPACING  0x10

/* serial port (aka COM port) information */
#define SYNOPSIS_UART_DLF_OFFSET		0xc0
#define SYNOPSIS_UART_DLF_115200_VAL	0x0d

#define COM1_BASE_ADRS		0xB0002000
#define COM1_INT_LVL		0x08		/* UART_A connected to IRQ8 */
#define COM1_INT_VEC		(INT_VEC_IRQ0 + COM1_INT_LVL)
#define COM1_INT_PRI		3
#define COM1_BAUD_RATE		115200
#define COM1_DLF			SYNOPSIS_UART_DLF_115200_VAL

#define COM2_BASE_ADRS		0xB0002400
#define COM2_INT_LVL		0x06		/* UART_B connected to IRQ6 */
#define COM2_INT_VEC		(INT_VEC_IRQ0 + COM2_INT_LVL)
#define COM2_INT_PRI		3
#define COM2_BAUD_RATE		115200
#define COM2_DLF			SYNOPSIS_UART_DLF_115200_VAL

#define UART_REG_ADDR_INTERVAL  4       /* address diff of adjacent regs. */

/*
 * On the board the UART works on the same clock frequency as CPU.
 */
#define UART_XTAL_FREQ	        CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC

/* UART uses level triggered interrupt, low level */
#define UART_IOAPIC_FLAGS       (IOAPIC_LEVEL)

/* uart configuration settings */

/* Generic definitions */

#define CONFIG_UART_NUM_SYSTEM_PORTS   2
#define CONFIG_UART_NUM_EXTRA_PORTS    0
#define CONFIG_UART_NUM_PORTS \
	(CONFIG_UART_NUM_SYSTEM_PORTS + CONFIG_UART_NUM_EXTRA_PORTS)

/* Setup console from config value, */

#if (CONFIG_UART_CONSOLE_INDEX == 0)

#define CONFIG_UART_BAUDRATE            COM1_BAUD_RATE
 /* Console definitions */
#define CONFIG_UART_CONSOLE_REGS        COM1_BASE_ADRS
#define CONFIG_UART_CONSOLE_IRQ         COM1_INT_LVL
#define CONFIG_UART_CONSOLE_INT_PRI     COM1_INT_PRI

#else /* CONFIG_UART_CONSOLE_INDEX */

#if (CONFIG_UART_CONSOLE_INDEX == 1)
#define CONFIG_UART_BAUDRATE	        COM2_BAUD_RATE
 /* Console definitions */
#define CONFIG_UART_CONSOLE_REGS	    COM2_BASE_ADRS
#define CONFIG_UART_CONSOLE_IRQ	        COM2_INT_LVL
#define CONFIG_UART_CONSOLE_INT_PRI     COM2_INT_PRI
#endif  /* CONFIG_UART_CONSOLE_INDEX == 1 */

#endif  /* CONFIG_UART_CONSOLE_INDEX == 0 */

#define UART_PORTS_CONFIGURE(__type, __name)                    \
        static __type __name[CONFIG_UART_NUM_PORTS] = {         \
                {                                               \
                        .port = CONFIG_UART_CONSOLE_REGS,        \
                        .irq = CONFIG_UART_CONSOLE_IRQ           \
                },                                              \
        }

#ifndef _ASMLANGUAGE

/*
 * The <pri> parameter is deliberately ignored. For this BSP, the macro just has
 * to make sure that unique vector numbers are generated.
 */
#define SYS_INT_REGISTER(s, irq, pri) \
	NANO_CPU_INT_REGISTER(s, INT_VEC_IRQ0 + (irq), 0)
#endif

#ifndef _ASMLANGUAGE

/**************************************************************************
*
* outByte - output byte to memory location
*
* RETURNS: N/A
*
* NOMANUAL
*/

static __inline__ void outByte(uint8_t data, uint32_t addr)
{
	*(volatile uint8_t *)addr = data;
}

/**************************************************************************
*
* inByte - obtain byte value from memory location
*
* This function issues the 'move' instruction to read a byte from the
* specified memory address.
*
* RETURNS: the byte read from the specified memory address
*
* NOMANUAL
*/

static __inline__ uint8_t inByte(uint32_t addr)
{
	return *((volatile uint8_t *)addr);
}

/*
 * Device drivers utilize the macros PLB_WORD_REG_WRITE() and
 * PLB_WORD_REG_READ() to access shortword-wide registers on the processor
 * local bus (PLB), as opposed to a PCI bus, for example.  Boards are
 * expected to provide implementations of these macros.
 */

/**************************************************************************
*
* outWord - output word to memory location
*
* RETURNS: N/A
*
* NOMANUAL
*/

static __inline__ void outWord(uint16_t data, uint32_t addr)
{
	*(volatile uint16_t *)addr = data;
}

/**************************************************************************
*
* inWord - obtain word value from memory location
*
* This function issues the 'move' instruction to read a word from the
* specified memory address.
*
* RETURNS: the word read from the specified memory address
*
* NOMANUAL
*/

static __inline__ uint16_t inWord(uint32_t addr)
{
	return *((volatile uint16_t *)addr);
}

/**************************************************************************
*
* outLong - output long word to memory location
*
* RETURNS: N/A
*
* NOMANUAL
*/

static __inline__ void outLong(uint32_t data, uint32_t addr)
{
	*(volatile uint32_t *)addr = data;
}

/**************************************************************************
*
* inLong - obtain long word value from memory location
*
* This function issues the 'move' instruction to read a word from the
* specified memory address.
*
* RETURNS: the long word read from the specified memory address
*
* NOMANUAL
*/

static __inline__ uint32_t inLong(uint32_t addr)
{
	return *((volatile uint32_t *)addr);
}
#endif /* !_ASMLANGUAGE */

extern void _SysIntVecProgram(unsigned int vector, unsigned int);

#endif /* __INCboardh */
