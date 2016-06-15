/* board.h - board configuration macros for the Quark_SE BSP */

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
the Quark_SE BSP.
*/

#ifndef __INCboardh
#define __INCboardh

#include <stdint.h>
#include <misc/util.h>
#include <drivers/uart.h>

#define INT_VEC_IRQ0  0x20	/* Vector number for IRQ0 */
#define HPET_TIMER0_IRQ INT_VEC_IRQ0

/*
 * IO APIC (IOAPIC) device information (Intel ioapic)
 */
#define IOAPIC_NUM_RTES         64              /* Number of IRQs = 32 */
#define IOAPIC_BASE_ADRS_PHYS   0xFEC00000      /* base physical address */
#define IOAPIC_SIZE             MB(1)
#define IOAPIC_BASE_ADRS      IOAPIC_BASE_ADRS_PHYS

/*
 * Local APIC (LOAPIC) device information (Intel loapic)
 */

#define LOAPIC_BASE_ADRS_PHYS   0xFEE00000      /* base physical address */
#define LOAPIC_SIZE             KB(4)
#define LOAPIC_BASE_ADRS      LOAPIC_BASE_ADRS_PHYS

/* serial port (aka COM port) information */

#define COM1_BASE_ADRS		0xB0002000
#define COM1_INT_LVL		0x05		/* COM1 connected to IRQ5 */
#define COM1_INT_VEC		(INT_VEC_IRQ0 + COM1_INT_LVL)
#define COM1_INT_PRI		3
#define COM1_BAUD_RATE		115200

#define COM2_BASE_ADRS		0xB0002400
#define COM2_INT_LVL		0x06		/* COM2 connected to IRQ6 */
#define COM2_INT_VEC		(INT_VEC_IRQ0 + COM2_INT_LVL)
#define COM2_INT_PRI		3
#define COM2_BAUD_RATE		115200

#define UART_REG_ADDR_INTERVAL  4       /* address diff of adjacent regs. */

#define UART_XTAL_FREQ	32000000
#define LOAPIC_TIMER_FREQ	CONFIG_LOAPIC_TIMER_FREQ

/* UART uses level triggered interrupt, low level */
#define UART_IOAPIC_FLAGS       (IOAPIC_LEVEL | IOAPIC_LOW)

/* uart configuration settings */

/* Generic definitions */
#define CONFIG_UART_NUM_SYSTEM_PORTS   2
#define CONFIG_UART_NUM_EXTRA_PORTS    0
#define CONFIG_UART_BAUDRATE	        COM2_BAUD_RATE
#define CONFIG_UART_NUM_PORTS \
	(CONFIG_UART_NUM_SYSTEM_PORTS + CONFIG_UART_NUM_EXTRA_PORTS)

 /* Console definitions */
#define CONFIG_UART_CONSOLE_REGS	    COM2_BASE_ADRS
#define CONFIG_UART_CONSOLE_IRQ	    COM2_INT_LVL
#define CONFIG_UART_CONSOLE_INT_PRI    COM2_INT_PRI

/* Host driver definitions */
#define CONFIG_UART_HOSTDRV_INDEX                   0
#define CONFIG_UART_HOSTDRV_INTERRUPT_DRIVEN   1
#define CONFIG_HOSTDRV_RX_EVENT             2
#define CONFIG_HOSTDRV_TX_EVENT             3
#define CONFIG_UART_HOSTDRV_REGS                    COM2_BASE_ADRS
#define CONFIG_UART_HOSTDRV_IRQ             COM2_INT_LVL
#define CONFIG_UART_HOSTDRV_INT_PRI         COM2_INT_PRI

#if (CONFIG_UART_NUM_SYSTEM_PORTS == 1)
#define UART_PORTS_CONFIGURE(__type, __name)                    \
        static __type __name[CONFIG_UART_NUM_PORTS] = {         \
                {                                               \
                        .port = CONFIG_UART_CONSOLE_REGS,        \
                        .irq = CONFIG_UART_CONSOLE_IRQ           \
                },                                              \
        }
#else
#define UART_PORTS_CONFIGURE(__type, __name)                    \
        static __type __name[CONFIG_UART_NUM_PORTS] = {         \
                {                                               \
                        .port = COM1_BASE_ADRS,                 \
                        .irq = COM1_INT_LVL                     \
                },                                              \
                {                                               \
                        .port = COM2_BASE_ADRS,                 \
                        .irq = COM2_INT_LVL                     \
                },                                              \
        }
#endif

/* uart interface */
#define UART_POLL_IN            uart_poll_in
#define UART_POLL_OUT           uart_poll_out
#define UART_FIFO_FILL          uart_fifo_fill
#define UART_FIFO_READ          uart_fifo_read
#define UART_IRQ_TX_ENABLE      uart_irq_tx_enable
#define UART_IRQ_TX_DISABLE     uart_irq_tx_disable
#define UART_IRQ_TX_READY       uart_irq_tx_ready
#define UART_IRQ_RX_ENABLE      uart_irq_rx_enable
#define UART_IRQ_RX_DISABLE     uart_irq_rx_disable
#define UART_IRQ_RX_READY       uart_irq_rx_ready
#define UART_IRQ_ERR_ENABLE     uart_irq_err_enable
#define UART_IRQ_ERR_DISABLE    uart_irq_err_disable
#define UART_IRQ_ERR_DETECTED   uart_irq_err_detected
#define UART_IRQ_IS_PENDING     uart_irq_is_pending
#define UART_IRQ_HW_UPDATE      uart_irq_update
#define UART_IRQ_INT_CONNECT    uart_int_connect
#define UART_LINE_STATUS        uart_line_status
#define UART_BREAK_CHECK        uart_break_check
#define UART_BREAK_SEND         uart_break_send

/*
 * The nanoCpuIntConnect() API connects to a (virtualized) IRQ and the
 * associated interrupt controller is programmed with the allocated vector.
 * The Quark_SE board virtualizes IRQs as follows:
 *
 *   - The first IOAPIC_NUM_RTES IRQs are provided by the IOAPIC
 *   - The remaining IRQs are provided by the LOAPIC.
 *
 * Thus, since the IOAPIC supports 32 IRQs:
 *
 *   - IRQ0 to IRQ31   map to IOAPIC IRQ0 to IRQ31
 *   - IRQ32 to IRQ37  map to LOAPIC LVT entries as follows:
 *
 *       IRQ32 -> LOAPIC_TIMER
 *       IRQ33 -> LOAPIC_THERMAL
 *       IRQ34 -> LOAPIC_PMC
 *       IRQ35 -> LOAPIC_LINT0
 *       IRQ36 -> LOAPIC_LINT1
 *       IRQ37 -> LOAPIC_ERROR
 */
#define LOAPIC_VEC_BASE(x) (x + INT_VEC_IRQ0 + IOAPIC_NUM_RTES)

/* local APIC timer definitions */
#define LOAPIC_TIMER_IRQ	IOAPIC_NUM_RTES
#define LOAPIC_TIMER_INT_PRI	2
#define LOAPIC_TIMER_VEC	LOAPIC_VEC_BASE(0)

#ifndef _ASMLANGUAGE
/*
 * The <pri> parameter is deliberately ignored. For this BSP, the macro just has
 * to make sure that unique vector numbers are generated.
 */
#define SYS_INT_REGISTER(s, irq, pri) \
	NANO_CPU_INT_REGISTER(s, INT_VEC_IRQ0 + (irq), 0)
#endif

/* Start of the 4 MB virtual address space */
#define VIRT_ADDR_START 0xA8000000

/*
 * Start of the pool in virtual memory where space for MMIO memory can be
 * allocated from. We allow a total of 1024 KB total space for static and
 * dynamic memory devices. Static devices *MUST* be allocated first. The
 * remainder of the 1 MB space is assumed to be used for dynamic devices.
 */
#define VIRT_POOL_MMIO_ADDR 0xA8200000
#define VIRT_POOL_MMIO_SIZE MB(1)



/* Core system registers */

struct scss_ccu {
        volatile uint32_t osc0_cfg0;    /**< Hybrid Oscillator Configuration 0 */
        volatile uint32_t osc0_stat1;   /**< Hybrid Oscillator status 1 */
        volatile uint32_t osc0_cfg1;    /**< Hybrid Oscillator configuration 1 */
        volatile uint32_t osc1_stat0;   /**< RTC Oscillator status 0 */
        volatile uint32_t osc1_cfg0;    /**< RTC Oscillator Configuration 0 */
        volatile uint32_t usb_pll_cfg0; /**< USB Phase lock look configuration */
        volatile uint32_t
            ccu_periph_clk_gate_ctl; /**< Peripheral Clock Gate Control */
        volatile uint32_t
            ccu_periph_clk_div_ctl0; /**< Peripheral Clock Divider Control 0 */
        volatile uint32_t
            ccu_gpio_db_clk_ctl; /**< Peripheral Clock Divider Control 1 */
        volatile uint32_t ccu_ext_clock_ctl; /**< External Clock Control Register */
        volatile uint32_t ccu_ss_periph_clk_gate_ctl; /**< Sensor subsustem
                                                     peripheral clock gate
                                                     control */
        volatile uint32_t ccu_lp_clk_ctl; /**< System Low Power Clock Control */
        volatile uint32_t reserved;
        volatile uint32_t ccu_mlayer_ahb_ctl; /**< AHB Control Register */
        volatile uint32_t ccu_sys_clk_ctl;    /**< System Clock Control Register */
        volatile uint32_t osc_lock_0;        /**< Clocks Lock Register */
};

struct scss_peripheral {
        volatile uint32_t usb_phy_cfg0; /**< USB Configuration */
        volatile uint32_t periph_cfg0;  /**< Peripheral Configuration */
        volatile uint32_t reserved[2];
        volatile uint32_t cfg_lock; /**< Configuration Lock */
};

struct int_ss_i2c {
        volatile uint32_t err_mask;
        volatile uint32_t rx_avail_mask;
        volatile uint32_t tx_req_mask;
        volatile uint32_t stop_det_mask;
};

struct int_ss_spi {
        volatile uint32_t err_int_mask;
        volatile uint32_t rx_avail_mask;
        volatile uint32_t tx_req_mask;
};

struct scss_interrupt {
        volatile uint32_t int_ss_adc_err_mask;
        volatile uint32_t int_ss_adc_irq_mask;
        volatile uint32_t int_ss_gpio_intr_mask[2];
        struct int_ss_i2c int_ss_i2c[2];
        struct int_ss_spi int_ss_spi[2];
        volatile uint32_t int_i2c_mst_mask[2];
        volatile uint32_t reserved;
        volatile uint32_t int_spi_mst_mask[2];
        volatile uint32_t int_spi_slv_mask[1];
        volatile uint32_t int_uart_mask[2];
        volatile uint32_t int_i2s_mask;
        volatile uint32_t int_gpio_mask;
        volatile uint32_t int_pwm_timer_mask;
        volatile uint32_t int_usb_mask;
        volatile uint32_t int_rtc_mask;
        volatile uint32_t int_watchdog_mask;
        volatile uint32_t int_dma_channel_mask[8];
        volatile uint32_t int_mailbox_mask;
        volatile uint32_t int_comparators_ss_halt_mask;
        volatile uint32_t int_comparators_host_halt_mask;
        volatile uint32_t int_comparators_ss_mask;
        volatile uint32_t int_comparators_host_mask;
        volatile uint32_t int_host_bus_err_mask;
        volatile uint32_t int_dma_error_mask;
        volatile uint32_t int_sram_controller_mask;
        volatile uint32_t int_flash_controller_mask[2];
        volatile uint32_t int_aon_timer_mask;
        volatile uint32_t int_adc_pwr_mask;
        volatile uint32_t int_adc_calib_mask;
        volatile uint32_t int_aon_gpio_mask;
        volatile uint32_t lock_int_mask_reg;
};

#define SCSS_PERIPHERAL_BASE (0xB0800800)
#define SCSS_PERIPHERAL ((struct scss_peripheral *)SCSS_PERIPHERAL_BASE)

#define SCSS_INT_BASE (0xB0800400)
#define SCSS_INTERRUPT ((struct scss_interrupt *)SCSS_INT_BASE)

/* Base Register */
#define SCSS_REGISTER_BASE		0xB0800000
#define SCSS_CCU ((struct scss_ccu *)SCSS_REGISTER_BASE)

/* Peripheral Clock Gate Control */
#define SCSS_CCU_PERIPH_CLK_GATE_CTL	0x18
#define CCU_PERIPH_CLK_EN 		(1 << 1)
#define CCU_PERIPH_CLK_DIV_CTL0		0x1C
#define INT_UNMASK_IA  		        (~0x00000001)

/* PWM */
#define CCU_PWM_PCLK_EN_SW     		(1 << 12)

#ifdef CONFIG_WATCHDOG
/* Watchdog */
#define WDT_BASE_ADDR               	0xB0000000
#define INT_WDT_IRQ               	0xc
#define INT_WATCHDOG_MASK		0x47C
#define SCSS_PERIPH_CFG0     		0x804
#define SCSS_PERIPH_CFG0_WDT_ENABLE	(1 << 1)
#define CCU_WDT_PCLK_EN_SW     		(1 << 10)
#endif

#ifdef CONFIG_RTC
/* RTC */
#define RTC_BASE_ADDR               	0xB0000400
#define SCSS_INT_RTC_MASK	        0x478
#define SCSS_CCU_SYS_CLK_CTL		0x38
#define CCU_RTC_PCLK_EN_SW 		(1 << 11)

#define INT_RTC_IRQ  		        0xb
#endif

/* Comparator */
#define INT_AIO_CMP_IRQ			(0x16)

/*ARC INIT*/
#define RESET_VECTOR                   0x40034000
#define SCSS_SS_CFG                    0x0600
#define SCSS_SS_STS                    0x0604
#define ARC_HALT_INT_REDIR             (1 << 26)
#define ARC_HALT_REQ_A                 (1 << 25)
#define ARC_RUN_REQ_A                  (1 << 24)
#define ARC_RUN (ARC_HALT_INT_REDIR | ARC_RUN_REQ_A)
#define ARC_HALT (ARC_HALT_INT_REDIR | ARC_HALT_REQ_A)

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
