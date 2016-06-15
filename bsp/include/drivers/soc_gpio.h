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

#ifndef SOC_GPIO_H_
#define SOC_GPIO_H_

#include "drivers/gpio.h"
#include "drivers/data_type.h"
#include "infra/device.h"

/**
 * @defgroup soc_gpio Quark_SE SOC GPIO
 * Quark_SE SOC General Purpose Input/Output drivers API.
 * @ingroup common_drivers
 * @{
 */

/**
 * SOC GPIO driver.
 */
extern struct driver soc_gpio_driver;

// FIXME: remove when IRQs are called with private parameter
void gpio_aon_isr();
void gpio_isr();

/**
 * Configure specified GPIO bit in specified GPIO port.
 *
 * Configuration parameters must be valid or an error is returned - see return values below.
 *
 * @param dev              GPIO device to use
 * @param bit              bit in port to configure
 * @param config           pointer to configuration structure
 *
 * @return
 *          - DRV_RC_OK                             on success
 *          - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED      if port id is not supported by this controller
 *          - DRV_RC_INVALID_CONFIG                 if any configuration parameters are not valid
 *          - DRV_RC_CONTROLLER_IN_USE              if port/bit is in use
 *          - DRV_RC_CONTROLLER_NOT_ACCESSIBLE      if port/bit is not accessible from this core
 *          - DRV_RC_FAIL                           otherwise
 */
DRIVER_API_RC soc_gpio_set_config(struct device *dev, uint8_t bit, gpio_cfg_data_t *config);

/**
 * Configure specified GPIO port (in QRK case the selection is ignored - only one port).
 *
 * @param dev              GPIO device to use
 * @param config           pointer to configuration structure
 *
 * @return
 *          - RC_OK                           on success
 *          - RC_DEVICE_TYPE_NOT_SUPPORTED    if port id is not supported by this controller
 *          - RC_INVALID_CONFIG               if any configuration parameters are not valid
 *          - RC_FAIL                         otherwise
 */
DRIVER_API_RC soc_gpio_set_port_config(struct device *dev, gpio_port_cfg_data_t *config);

/**
 * Deconfigure specified GPIO bit in specified GPIO port
 *
 * Configuration parameters must be valid or an error is returned - see return values below.
 *
 * @param dev             : GPIO device to use
 * @param bit             : bit in port to deconfigure
 *
 * @return
 *          - DRV_RC_OK                            on success
 *          - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED     if port id is not supported by this controller
 *          - DRV_RC_INVALID_CONFIG                if any configuration parameters are not valid
 *          - DRV_RC_CONTROLLER_IN_USE,            if port/bit is in use
 *          - DRV_RC_CONTROLLER_NOT_ACCESSIBLE     if port/bit is not accessible from this core
 *          - DRV_RC_FAIL                          otherwise
 */
DRIVER_API_RC soc_gpio_deconfig(struct device *dev, uint8_t bit);

/**
 * Deconfigure specified specified GPIO port
 *
 * @param dev             GPIO device to use
 *
 * @return
 *          - RC_OK on success
 *          - RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_port_deconfig(struct device *dev);

/**
 * Get callback argument pointer for a specific pin
 *
 * @param dev              GPIO device to use
 * @param pin              pin in port to configure
 *
 * @return  pointer if success else NULL
 */
void* soc_gpio_get_callback_arg(struct device *dev, uint8_t pin);

/**
 * Enable the specified GPIO port
 *
 * Upon success, the specified GPIO port is no longer clock gated in hardware, it is now
 * capable of reading and writing GPIO bits and of generating interrupts.
 *
 * @param  dev              GPIO device to use
 *
 * @return
 *          - DRV_RC_OK                            on success
 *          - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED     if port id is not supported by this controller
 *          - DRV_RC_INVALID_CONFIG                if any configuration parameters are not valid
 *          - DRV_RC_CONTROLLER_IN_USE,            if port/bit is in use
 *          - DRV_RC_CONTROLLER_NOT_ACCESSIBLE     if port/bit is not accessible from this core
 *          - DRV_RC_FAIL                          otherwise
 */
DRIVER_API_RC soc_gpio_enable(struct device *dev);

/**
 * Disable the specified GPIO port
 *
 * Upon success, the specified GPIO port is clock gated in hardware, it is now
 * incapable of reading, writing GPIO bits and of generating interrupts.
 *
 * @param  dev              GPIO device to use
 *
 * @return
 *          - DRV_RC_OK                            on success
 *          - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED     if port id is not supported by this controller
 *          - DRV_RC_INVALID_CONFIG                if any configuration parameters are not valid
 *          - DRV_RC_CONTROLLER_IN_USE,            if port/bit is in use
 *          - DRV_RC_CONTROLLER_NOT_ACCESSIBLE     if port/bit is not accessible from this core
 *          - DRV_RC_FAIL                          otherwise
 */
DRIVER_API_RC soc_gpio_disable(struct device *dev);

/**
 * Set output value on the gpio bit
 *
 * @param dev              GPIO device to use
 * @param bit              bit in port to configure
 * @param value            value to write to bit
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_write(struct device *dev, uint8_t bit, bool value);

/**
 * Read a GPIO bit
 *
 * @param dev              GPIO device to use
 * @param bit              bit in port to configure
 * @param value            address to place read value
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_read(struct device *dev, uint8_t bit, bool *value);

/**
 * Write to a value to a given port
 *
 * @param dev              GPIO device to use
 * @param value            value to write to port
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_write_port(struct device *dev, uint32_t value);

/**
 * Read a given port
 *
 * @param dev              GPIO device to use
 * @param value            location to store result
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_read_port(struct device *dev, uint32_t *value);

/**
 *  Mask the interrupt of a GPIO bit
 *
 *  @param dev              GPIO device to use
 *  @param bit              bit in port to configure
 *
 *  @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_mask_interrupt(struct device *dev, uint8_t bit);

/**
 *  Unmask the interrupt of a GPIO bit
 *
 *  @param dev              GPIO device to use
 *  @param bit              bit in port to configure
 *
 *  @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC soc_gpio_unmask_interrupt(struct device *dev, uint8_t bit);

/** @} */

#endif  /* SOC_GPIO_H_ */
