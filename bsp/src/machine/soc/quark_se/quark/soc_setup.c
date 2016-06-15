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

/* VxMicro symbols */
#include <misc/printk.h>
#include <microkernel.h>
#include <arch/cpu.h>


/* Operating System Abstraction */
#include "os/os.h"
#include "zephyr/common.h"

/* Board/SOC */
#include "machine.h"
#include "board.h"
#include <drivers/uart.h>
#include "machine/soc/quark_se/quark/soc_setup.h"
#include "infra/pm.h"
#include "infra/device.h"
#include "machine/soc/quark_se/usb_setup.h"
#include "pinmux.h"
#include "drivers/ioapic.h"
#include "drivers/loapic.h"
#include "drivers/system_timer.h"
#include "quark_se_mapping.h"
#include "machine/soc/quark_se/quark/uart_tcmd_client.h"

#include "infra/log.h"

extern void _InitHardware(void);

struct device *uart_dev;

/* Reboot information */
#define RESET_REASON_SIZE 8
char *rst_reasons[RESET_REASON_SIZE] = { "COLD RESET", "DEBUG", "SW",
        "QRK_WDT", "ARC_WDLT", "INTERUPT_QRK", "INTERUPT_SS", "UNKNOWN" };

void _mbxIntStub(void);
SYS_INT_REGISTER(_mbxIntStub, SOC_MBOX_INTERRUPT, 0);
#if !defined(CONFIG_WDT_NMI) && defined(CONFIG_INTEL_QRK_WDT)
void _wdtIntStub(void);
SYS_INT_REGISTER(_wdtIntStub, SOC_WDT_INTERRUPT, 0);
#endif
#ifdef CONFIG_INTEL_QRK_RTC
void _rtcIntStub(void);
SYS_INT_REGISTER(_rtcIntStub, SOC_RTC_INTERRUPT, 0);
#endif
#ifdef CONFIG_INTEL_QRK_AON_PT
void _aonptIntStub(void);
SYS_INT_REGISTER(_aonptIntStub, SOC_AONPT_INTERRUPT, 0);
#endif
#ifdef CONFIG_INTEL_QRK_PWM
void _pwmIntStub(void);
SYS_INT_REGISTER(_pwmIntStub, SOC_PWM_INTERRUPT, 0);
#endif
#ifdef CONFIG_SOC_GPIO
void _gpioIntStub(void);
SYS_INT_REGISTER(_gpioIntStub, SOC_GPIO_INTERRUPT, 0);
#ifdef CONFIG_SOC_GPIO_AON
void _gpioAonIntStub(void);
SYS_INT_REGISTER(_gpioAonIntStub, SOC_GPIO_AON_INTERRUPT, 0);
#endif
#endif
#ifdef CONFIG_INTEL_QRK_I2C
void _i2c0IntStub(void);
SYS_INT_REGISTER(_i2c0IntStub, SOC_I2C0_INTERRUPT, 0);
void _i2c1IntStub(void);
SYS_INT_REGISTER(_i2c1IntStub, SOC_I2C1_INTERRUPT, 0);
#endif
#ifdef CONFIG_INTEL_QRK_SPI
void _spiM0IntStub(void);
SYS_INT_REGISTER(_spiM0IntStub, SOC_SPIM0_INTERRUPT, 0);
void _spiM1IntStub(void);
SYS_INT_REGISTER(_spiM1IntStub, SOC_SPIM1_INTERRUPT, 0);
void _spiSIntStub(void);
SYS_INT_REGISTER(_spiSIntStub, SOC_SPIS0_INTERRUPT, 0);
#endif
#ifdef CONFIG_SOC_COMPARATOR
void _compIntStub(void);
SYS_INT_REGISTER(_compIntStub, SOC_CMP_INTERRUPT, 0);
#endif
#ifdef CONFIG_USB
void _usbIntStub(void);
SYS_INT_REGISTER(_usbIntStub, SOC_USB_INTERRUPT, 0);
#endif
#ifdef CONFIG_IPC_UART
void _uart0IntStub(void);
SYS_INT_REGISTER(_uart0IntStub, SOC_UART0_INTERRUPT, 0);
#endif
#ifdef CONFIG_UART_NS16550
void _uart1IntStub(void);
SYS_INT_REGISTER(_uart1IntStub, SOC_UART1_INTERRUPT, 0);
#endif

#ifdef CONFIG_QUARK_SE_PANIC_DEFAULT
void _divTrapStub(void);
void _nmiTrapStub(void);
void _overflowTrapStub(void);
void _boundTrapStub(void);
void _invOptTrapStub(void);
void _devNotAvailTrapStub(void);
void _dblFaultTrapStub(void);
void _tssTrapStub(void);
void _segTrapStub(void);
void _stackTrapStub(void);
void _gpfTrapStub(void);
void _pageTrapStub(void);
void _floatTrapStub(void);
void _alignTrapStub(void);

NANO_CPU_INT_REGISTER(_divTrapStub, 0, 0);
NANO_CPU_INT_REGISTER(_nmiTrapStub, 2, 0);
NANO_CPU_INT_REGISTER(_overflowTrapStub, 4, 0);
NANO_CPU_INT_REGISTER(_boundTrapStub, 5, 0);
NANO_CPU_INT_REGISTER(_invOptTrapStub, 6, 0);
NANO_CPU_INT_REGISTER(_devNotAvailTrapStub, 7, 0);
NANO_CPU_INT_REGISTER(_dblFaultTrapStub, 8, 0);
NANO_CPU_INT_REGISTER(_tssTrapStub, 10, 0);
NANO_CPU_INT_REGISTER(_segTrapStub, 11, 0);
NANO_CPU_INT_REGISTER(_stackTrapStub, 12, 0);
NANO_CPU_INT_REGISTER(_gpfTrapStub, 13, 0);
NANO_CPU_INT_REGISTER(_pageTrapStub, 14, 0);
NANO_CPU_INT_REGISTER(_floatTrapStub, 16, 0);
NANO_CPU_INT_REGISTER(_alignTrapStub, 17, 0);

#ifdef CONFIG_PANIC_ON_BUS_ERROR
void _pmuIntStub(void);
SYS_INT_REGISTER(_pmuIntStub, SOC_SYSTEM_PMU_INTERRUPT, 0);
#endif /* CONFIG_PANIC_ON_BUS_ERROR */
#endif

void mbx_out_channel()
{
	if (MBX_DAT1(4) == 0) {
		//pr_info(LOG_MODULE_MAIN, "A%s", MBX_DAT0(4));
	} else {
		/* The logs use the default UART */
		 UART_POLL_OUT(CONFIG_UART_CONSOLE_INDEX, MBX_DAT0(4));
	}
}

static void (*ipc_message_handler) () = NULL;

void mbxIsr(int param)
{
	do {
		int i;
		for (i = 4; i < 6; i++) {
			int sts = MBX_STS(i);
			if (MBX_STS(i) & 0x2) {
				if (i == 4) {
					mbx_out_channel();
					MBX_STS(i) = sts;
				} else if ((i == 5) && ipc_message_handler) {
					ipc_message_handler();
				} else
					MBX_STS(i) = sts;
			} else
				MBX_STS(i) = sts;
		}
	} while (MBX_CHALL_STS & 0x0500);
}

void soc_setup(void (*handler) ())
{
	pinmux_config();

	/* Temporary: we need to reference the IPC message handler from mbxIsr */
	ipc_message_handler = handler;

	/* TODO: Get the ARC Flash start address from predefined HW block */

	unsigned int *src = (unsigned int *)(ARC_FLASH_START_ADDR);
	uint32_t arc_state;
	int i;


	/* Copy arc vector to start of RAM. */
	shared_data->arc_start = *src;

	printk("\n\n\n\nBOOT\n\n\nQuark_SE  ID %d Rev %d\r\n",
	       SCSS_REG_VAL(SCSS_ID), SCSS_REG_VAL(SCSS_REV));

	/* Check Sensor Subsystem (ARC) status */
	arc_state = SCSS_REG_VAL(SCSS_SS_STS);
	if (arc_state == 0) {
		printk("ARC running on boot, halt !\r\n");
		SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_HALT_REQ_A;
		disable_scheduling();
		interrupt_lock();
		while (1)
			mbxIsr(0);
	}
	printk("ARC Core state: %p\r\n", arc_state);

	/* Print Boot Reason */
	printk("REBOOT REASON: %s\r\n",
	       rst_reasons[MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, SCSS_GPS0)
	                   >> RESET_REASON_OFFSET]);
	/* Clear Boot Reason */
	MMIO_REG_VAL_FROM_BASE (SCSS_REGISTER_BASE,SCSS_RSTS) = 0x1f;

	/* Initialize OS abstraction */
	framework_init_common();
	printk("os abstraction init done\n");
	SCSS_REG_VAL(SCSS_INT_MAILBOX_MASK) = 0xffffffff;
	/* Enable interrupt for channels 4 & 5 */
	SOC_MBX_INT_UNMASK(4);
	SOC_MBX_INT_UNMASK(5);

	/* Enable Always on timer */
	SCSS_REG_VAL(SCSS_AONC_CFG) = AONC_CNT_EN;

#ifdef CONFIG_QUARK_SE_DISABLE_INTERNAL_VREG
	/* disable platform voltage regulator */
	SCSS_REG_VAL(SCSS_PLAT3P3_VR) &= ~(PLAT_VR_ENABLE | PLAT_VR_HIZ);
	SCSS_REG_VAL(SCSS_PLAT1P8_VR) &= ~(PLAT_VR_ENABLE | PLAT_VR_HIZ);
#endif
	/* Clear Mailboxes */
	for (i = 0; i < 8; i++)
		MBX_STS(i) = 3;

	for (i = 0; i < 32; i++) {
		_SysIntVecProgram(INT_VEC_IRQ0 + i, i);
	}
#ifdef CONFIG_PANIC_ON_BUS_ERROR
	/* Clear bus error interrupt before activating */
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, QRK_SCSS_P_STS) = 8;
	/* Enable PMU interrupt */
	SET_INTERRUPT_HANDLER(SOC_SYSTEM_PMU_INTERRUPT, _pmuIntStub);
	MMIO_REG_VAL_FROM_BASE(SCSS_REGISTER_BASE, INT_SYSTEM_PMU_MASK) &= ~(1);
#endif

	interrupt_enable(SOC_MBOX_INTERRUPT);
	interrupt_enable(SOC_CMP_INTERRUPT);

	/* Init logs */
	log_init();

#ifdef CONFIG_WAKELOCK
	/* init shared wakelock status variables */
	shared_data->any_qrk_wakelock_taken = 0;
	shared_data->any_arc_wakelock_taken = 0;

	/* Init wakelocks */
	pm_wakelock_init_mgr();
#endif

	/* Init devices */
	init_all_devices();

#ifdef CONFIG_USB
	/* Init SoC specific USB */
	platform_usb_init();
#endif

	task_sleep(10);
}

void start_arc(unsigned int reset_vector)
{
	if (reset_vector != 0) {
		shared_data->arc_start = reset_vector;
	}

	shared_data->arc_ready = 0;


	/* Put ARC out of reset. */
	pr_debug(LOG_MODULE_MAIN, "Put ARC out of reset");

	task_sleep(10);
	/*
	 * In order to debug the arc, you can comment this line,
	 * in order to be able to set breakpoints and start ARC with
	 * breakpoints enabled.
	 */
	SCSS_REG_VAL(SCSS_SS_CFG) |= ARC_RUN_REQ_A;
	task_sleep(10);

	pr_debug(LOG_MODULE_MAIN, "ARC Core state: %p",
			SCSS_REG_VAL(SCSS_SS_STS));

	while(!shared_data->arc_ready) ;
	pr_debug(LOG_MODULE_MAIN, "ARC ready");
}


void soc_resume(void)
{
	/* LOAPIC initialisation*/
	_loapic_init();
	/* IOAPIC initialisation*/
	_ioapic_init();
	/* TODO: restore other saved value */
	_sys_clock_driver_init(NULL);

	pinmux_config();

	int i = 0;
	for (i = 0; i < 32; i++) {
		_SysIntVecProgram(INT_VEC_IRQ0 + i, i);
	}

	interrupt_enable(SOC_CMP_INTERRUPT);
	interrupt_enable(SOC_MBOX_INTERRUPT);
	uart_dev->driver->resume(uart_dev);
}

void soc_suspend(void)
{
	uart_dev = get_device(UART1_PM_ID);
	//TODO save QRK_SE registers
}
#ifdef CONFIG_WAKELOCK
bool pm_wakelock_are_other_cpu_wakelocks_taken(void){
	return shared_data->any_arc_wakelock_taken;
}

void pm_wakelock_set_any_wakelock_taken_on_cpu(bool wl_status){
	shared_data->any_qrk_wakelock_taken = wl_status;
}
#endif
