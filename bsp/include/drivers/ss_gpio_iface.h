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

#ifndef SS_GPIO_IFACE_H_
#define SS_GPIO_IFACE_H_

#include "drivers/gpio.h"
#include "infra/device.h"

/**
 * @defgroup gpio_arc_driver Sensors Subsystem GPIO
 * General Purpose Input/Output ARC driver API.
 * @ingroup arc_driver
 * @{
 */

// FIXME: remove when IRQs are called with private parameter
void ss_gpio_8b0_ISR();
void ss_gpio_8b1_ISR();

/**
 * SS GPIO driver.
 */
extern struct driver ss_gpio_driver;

/**
 * Configures specified GPIO bit in specified GPIO port.
 *
 * Configuration parameters must be valid or an error is returned - see return values below.
 *
 * @param   dev              GPIO device to use
 * @param   bit              bit in port to configure
 * @param   config           pointer to configuration structure
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED    - if port id is not supported by this controller
 *          - DRV_RC_INVALID_CONFIG               - if any configuration parameters are not valid
 *          - DRV_RC_CONTROLLER_IN_USE,             if port/bit is in use
 *          - DRV_RC_CONTROLLER_NOT_ACCESSIBLE      if port/bit is not accessible from this core
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_set_config(struct device *dev, uint8_t bit, gpio_cfg_data_t *config);

/**
 * Gets callback argument pointer for a specific pin.
 *
 * @param   dev              GPIO device to use
 * @param   pin              pin in port to configure
 *
 * @return  ptr if success else NULL
 */
void* ss_gpio_get_callback_arg(struct device *dev, uint8_t pin);

/**
 * Transmits a block of data to the specified SPI slave.
 *
 * @param   dev              GPIO device to use
 * @param   bit              bit in port to configure
 * @param   value            value to write to bit
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_write(struct device *dev, uint8_t bit, bool value);

/**
 * Reads a GPIO bit.
 *
 * @param   dev              GPIO device to use
 * @param   bit              bit in port to configure
 * @param   value            address to place read value
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_read(struct device *dev, uint8_t bit, bool *value);

/**
 * Writes to a value to a given port.
 *
 * @param   dev              GPIO device to use
 * @param   value            value to write to port
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_write_port(struct device *dev, uint32_t value);

/**
 * Reads a given port.
 *
 * @param   dev              GPIO device to use
 * @param   *value           location to store result
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_read_port(struct device *dev, uint32_t *value);

/**
 * Masks the interrupt of a GPIO bit.
 *
 * @param   dev              GPIO device to use
 * @param   bit              bit in port to configure
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_mask_interrupt(struct device *dev, uint8_t bit);

/**
 * Unmasks the interrupt of a GPIO bit.
 *
 * @param   dev              GPIO device to use
 * @param   bit              bit in port to configure
 *
 * @return
 *          - DRV_RC_OK on success
 *          - DRV_RC_FAIL otherwise
 */
DRIVER_API_RC ss_gpio_unmask_interrupt(struct device *dev, uint8_t bit);
#endif  /* SS_GPIO_IFACE_H_ */

/** @} */
