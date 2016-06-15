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

#define _ASMLANGUAGE


#ifndef CONFIG_DYNAMIC_INT_STUBS

#include <arch/x86/asm.h>

	/* exports (internal APIs) */
	GTEXT(_loApicTimerIntStub)

	/* externs (internal APIs) */

	GTEXT(_IntEnt)
	GTEXT(_IntExit)
	GTEXT(_LoApicIntEoi)
	GTEXT(_mbxIntStub)
#if !defined(CONFIG_WDT_NMI) && defined(CONFIG_INTEL_QRK_WDT)
	GTEXT(_wdtIntStub)
#endif
	GTEXT(_rtcIntStub)
	GTEXT(_pwmIntStub)
	GTEXT(_gpioIntStub)
	GTEXT(_gpioAonIntStub)
	GTEXT(_i2c0IntStub)
	GTEXT(_i2c1IntStub)
	GTEXT(_spiM0IntStub)
	GTEXT(_spiM1IntStub)
	GTEXT(_spiSIntStub)
	GTEXT(_compIntStub)
#ifdef CONFIG_USB
	GTEXT(_usbIntStub)
#endif
	GTEXT(_uart0IntStub)
	GTEXT(_uart1IntStub)
#ifdef CONFIG_INTEL_QRK_AON_PT
	GTEXT(_aonptIntStub)
#endif

SECTION_FUNC(TEXT, _mbxIntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    mbxIsr	        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */
#if !defined(CONFIG_WDT_NMI) && defined(CONFIG_INTEL_QRK_WDT)
SECTION_FUNC(TEXT, _wdtIntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    wdt_isr	        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit       /* Inform kernel interrupt is done */
#endif
SECTION_FUNC(TEXT, _rtcIntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    rtc_isr	        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _pwmIntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    pwm_isr	        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _gpioIntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    gpio_isr        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _gpioAonIntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    gpio_aon_isr    /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _i2c0IntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    isr_dev_0	        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _i2c1IntStub)
	call    _IntEnt			/* Inform kernel interrupt has begun */
	pushl   $0			    /* Push dummy parameter */
	call    isr_dev_1	        /* Call actual interrupt handler */
	call    _ioapic_eoi	/* Inform ioapic interrupt is done */
	addl    $4, %esp		/* Clean-up stack from push above */
	jmp     _IntExit		/* Inform kernel interrupt is done */


SECTION_FUNC(TEXT, _spiM0IntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    soc_spi_mst0_ISR /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _spiM1IntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    soc_spi_mst1_ISR /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _spiSIntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    soc_spi_slv_ISR /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */

#ifdef CONFIG_USB
SECTION_FUNC(TEXT, _usbIntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    usb_ISR         /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */
#endif /* CONFIG_USB */

SECTION_FUNC(TEXT, _compIntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    comp_isr        /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _uart0IntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    uart_ipc_isr    /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */

SECTION_FUNC(TEXT, _uart1IntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    uart_rx_isr    /* Call actual interrupt handler */
	call    _ioapic_eoi  /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */

#ifdef CONFIG_INTEL_QRK_AON_PT
SECTION_FUNC(TEXT, _aonptIntStub)
	call    _IntEnt         /* Inform kernel interrupt has begun */
	pushl   $0              /* Push dummy parameter */
	call    aonpt_ISR       /* Call actual interrupt handler */
	call    _ioapic_eoi     /* Inform ioapic interrupt is done */
	addl    $4, %esp        /* Clean-up stack from push above */
	jmp     _IntExit        /* Inform kernel interrupt is done */
#endif //CONFIG_INTEL_QRK_AON_PT
#endif /* !CONFIG_DYNAMIC_INT_STUBS */
