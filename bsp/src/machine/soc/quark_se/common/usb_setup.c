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

/* SoC-specific USB initialazation */

#include "infra/log.h"
#include "drivers/soc_gpio.h"
#include "machine.h"


void platform_usb_init(void)
{
	pr_info(LOG_MODULE_USB,"USB INIT");

	/* platform specific init of USB core */
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_0) = PIN_MUX_SLEW_4mA_driver;
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_1) = PIN_MUX_SLEW_4mA_driver;
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_2) = PIN_MUX_SLEW_4mA_driver;
	MMIO_REG_VAL(QRK_PMUX_SLEW_RATE_3) = PIN_MUX_SLEW_4mA_driver;

	/* Init the USB driver */
	SCSS_REG_VAL(SCSS_INT_USB_MASK_OFFSET) = QRK_INT_USB_UNMASK_QRK;
}
