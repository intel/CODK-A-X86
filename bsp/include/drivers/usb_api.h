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

/* Includes everything to initialize and use the DWC USB driver customized for
 * MMU-less platform */

#ifndef __USB_API_H__
#define __USB_API_H__

#include <stdint.h>
#include "usb.h"

/**
 * Interrupt routine.
 */
void usb_ISR(void);

/**
 * USB Function driver initialization function.
 *
 * This function is called at startup and each time the USB port is plugged in
 * the @ref usb_driver_init function.
 *
 * @param priv User defined private data.
 *
 * */
typedef void (*function_driver_init_t)(void *priv, usb_string_descriptor_t * alt_strings);

/**
 * Usb function driver registration.
 *
 * Before initialazing the USB driver, you must register a function driver
 * initialization function.
 *
 * @param f_init The function driver initialization function.
 * @param priv The argument of the function driver initialization function.
 * @return  0 in cas of success, < 0 in case of failure.
 */
int usb_register_function_driver(function_driver_init_t f_init, void *priv, usb_string_descriptor_t * alt_strings);


/**
 * Usb driver initialization.
 *
 * @param base_addr  The base address of the USB core.
 * @return  0 in cas of success, < 0 in case of failure.
 */
int usb_driver_init(uint32_t base_addr);


#endif /* __USB_API_H__ */
