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

#include "drivers/soc_gpio.h"

#include "machine.h"
#include "infra/device.h"
#include "infra/log.h"

#define GPIO_CLKENA_POS         (31)
#define GPIO_LS_SYNC_POS        (0)

static void soc_gpio_ISR_proc(struct device *dev);

#ifdef CONFIG_SOC_GPIO_32
// FIXME: remove when IRQs are called with private parameter
static struct device *soc_gpio_32_irq_dev;

/*! \brief  Interrupt handler for GPIO port 0 (32 bits)
 */
DECLARE_INTERRUPT_HANDLER void gpio_isr()
{
    soc_gpio_ISR_proc(soc_gpio_32_irq_dev);
}
#endif

#ifdef CONFIG_SOC_GPIO_AON
// FIXME: remove when IRQs are called with private parameter
static struct device *soc_gpio_aon_irq_dev;

/*! \brief  Interrupt handler for GPIO port 1 (aon, 6 bits)
 */
DECLARE_INTERRUPT_HANDLER void gpio_aon_isr()
{
    soc_gpio_ISR_proc(soc_gpio_aon_irq_dev);
}
#endif

static int soc_gpio_init(struct device *dev)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;


    // FIXME: remove when IRQs are called with private parameter
    switch (dev->id) {
#ifdef CONFIG_SOC_GPIO_32
        case SOC_GPIO_32_ID:
            soc_gpio_32_irq_dev = dev;
            break;
#endif
#ifdef CONFIG_SOC_GPIO_AON
        case SOC_GPIO_AON_ID:
            soc_gpio_aon_irq_dev = dev;
            break;
#endif
        default:
            pr_error(LOG_MODULE_DRV, "invalid soc gpio id %d", dev->id);
            return -1;
    }

    /* enable peripheral clock */
    SET_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_CLKENA_POS);
    SET_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_LS_SYNC), (uint32_t)GPIO_LS_SYNC_POS);
    /* Clear any existing interrupts */
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_PORTA_EOI) = ~(0);
    /* enable interrupt for this GPIO block */
    SET_INTERRUPT_HANDLER(gpio_dev->vector, gpio_dev->gpio_isr);
    /* Enable GPIO  Interrupt into SSS  */
    SOC_UNMASK_INTERRUPTS(gpio_dev->gpio_int_mask);

    return 0;
}

static int soc_gpio_suspend(struct device *dev, PM_POWERSTATE state)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;

    /* Save Config */
    gpio_dev->pm_context.gpio_type =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_SWPORTA_DDR);
    gpio_dev->pm_context.int_type =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_INTTYPE_LEVEL);
    gpio_dev->pm_context.int_polarity =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_INTPOLARITY);
    gpio_dev->pm_context.int_debounce =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_DEBOUNCE);
    gpio_dev->pm_context.int_ls_sync =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_LS_SYNC);
    gpio_dev->pm_context.int_bothedge =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_INT_BOTHEDGE);
    gpio_dev->pm_context.output_values =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_SWPORTA_DR);
    gpio_dev->pm_context.mask_interrupt =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_INTMASK);
    gpio_dev->pm_context.mask_inten =
        MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base,
                SOC_GPIO_INTEN);

    pr_debug(LOG_MODULE_DRV, "gpio suspend device %d (%d)", dev->id,
            state);

    return DRV_RC_OK;
}

static int soc_gpio_resume(struct device *dev)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;

    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) =
        gpio_dev->pm_context.int_type;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTPOLARITY) =
        gpio_dev->pm_context.int_polarity;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_DEBOUNCE) =
        gpio_dev->pm_context.int_debounce;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_LS_SYNC) =
        gpio_dev->pm_context.int_ls_sync;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INT_BOTHEDGE) =
        gpio_dev->pm_context.int_bothedge;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTMASK) =
        gpio_dev->pm_context.mask_interrupt;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTEN) =
        gpio_dev->pm_context.mask_inten;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_SWPORTA_DR) =
        gpio_dev->pm_context.output_values;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_SWPORTA_DDR) =
        gpio_dev->pm_context.gpio_type;

    /* enable interrupt for this GPIO block */
    SET_INTERRUPT_HANDLER(gpio_dev->vector, gpio_dev->gpio_isr);
    /* Enable GPIO  Interrupt into SSS  */
    SOC_UNMASK_INTERRUPTS(gpio_dev->gpio_int_mask);

    pr_debug(LOG_MODULE_DRV, "gpio resume device %d", dev->id);
    return DRV_RC_OK;
}

struct driver soc_gpio_driver = {
    .init = soc_gpio_init,
    .suspend = soc_gpio_suspend,
    .resume = soc_gpio_resume
};

void* soc_gpio_get_callback_arg(struct device *dev, uint8_t pin)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;

    if (pin >= gpio_dev->no_bits) {
        return NULL;
    }

    return gpio_dev->gpio_cb_arg[pin];
}

DRIVER_API_RC soc_gpio_set_config(struct device *dev, uint8_t bit, gpio_cfg_data_t *config)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    uint32_t saved;

    // Check pin index
    if (bit >= gpio_dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }

    // Validate config data
    if ((config->gpio_type != GPIO_INPUT) && (config->gpio_type != GPIO_OUTPUT) && (config->gpio_type != GPIO_INTERRUPT)) {
        return DRV_RC_INVALID_CONFIG;
    }

    // Check listen configuration
    if(config->gpio_type == GPIO_INTERRUPT) {
        if (gpio_dev->gpio_cb[bit] != NULL) {
            // pin is already in use
            return DRV_RC_CONTROLLER_IN_USE;
        }
        if ((config->int_type != LEVEL) &&
                (config->int_type != EDGE) &&
                (config->int_type != DOUBLE_EDGE)) {
            // invalid config for listen
            return DRV_RC_INVALID_CONFIG;
        }

    }

    /* Disable Interrupts from this bit */
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTEN) &= ~(1 << bit);

    // Set interrupt handler to NULL
    gpio_dev->gpio_cb[bit] = NULL;
    gpio_dev->gpio_cb_arg[bit] = NULL;

    switch(config->gpio_type)
    {
        case GPIO_INPUT:
            /* configure as input */
            CLEAR_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_SWPORTA_DDR), (uint32_t)bit);
            break;
        case GPIO_OUTPUT:
            /* configure as output */
            SET_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_SWPORTA_DDR), (uint32_t)bit);
            break;
        case GPIO_INTERRUPT:
            saved = interrupt_lock();
            // Configure interrupt handler
            gpio_dev->gpio_cb[bit] = config->gpio_cb;
            gpio_dev->gpio_cb_arg[bit] = config->gpio_cb_arg;
            MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_SWPORTA_DDR) &= ~(1 << bit);

            /* Set Level, Edge or Double Edge */
            if(config->int_type == LEVEL) {
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) &= ~(1 << bit);
            } else if(config->int_type == EDGE) {
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) |= (1 << bit);
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INT_BOTHEDGE) &= ~(1 << bit);
            } else { // DOUBLE_EDGE
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) |= (1 << bit);
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INT_BOTHEDGE) |= (1 << bit);
            }

            /* Set Polarity - Active Low / High */
            if(ACTIVE_LOW == config->int_polarity) {
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTPOLARITY) &= ~(1 << bit);
            } else {
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTPOLARITY) |= (1 << bit);
            }

            /* Set Debounce - On / Off */
            if(config->int_debounce == DEBOUNCE_OFF) {
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_DEBOUNCE) &= ~(1 << bit);
            } else {
                MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_DEBOUNCE) |= (1 << bit);
            }

            /* Enable as Interrupt */
            MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTEN) |= (1 << bit);
            /* Unmask Interrupt */
            MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTMASK) &= ~(1 << bit);

            interrupt_unlock(saved);
            break;
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_set_port_config(struct device *dev, gpio_port_cfg_data_t *config)
{
    unsigned int i;
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;

    // Disable gpio interrupts
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTEN) = 0;

    for(i=0; i<gpio_dev->no_bits; i++) {
        gpio_dev->gpio_cb[i] = config->gpio_cb[i];
        gpio_dev->gpio_cb_arg[i] = config->gpio_cb_arg[i];
    }

    // Set gpio direction
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_SWPORTA_DDR) = config->gpio_type;
    // Set gpio interrupt settings
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTTYPE_LEVEL) = config->int_type;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTPOLARITY) = config->int_polarity;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_DEBOUNCE) = config->int_debounce;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_LS_SYNC) = config->int_ls_sync;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INT_BOTHEDGE) = config->int_bothedge;

    // Clear interrupt flag
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_PORTA_EOI) = (1<<gpio_dev->no_bits)-1;
    // Enable gpio interrupt
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTMASK) = ~((~(config->gpio_type)) & (config->is_interrupt));
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTEN) = ((~(config->gpio_type)) & (config->is_interrupt));

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_deconfig(struct device *dev, uint8_t bit)
{
    gpio_cfg_data_t config;
    // Default configuration (input pin without interrupt)
    config.gpio_type = GPIO_INPUT;
    config.int_type = EDGE;
    config.int_polarity = ACTIVE_LOW;
    config.int_debounce = DEBOUNCE_OFF;
    config.int_ls_sync = LS_SYNC_OFF;
    config.gpio_cb = NULL;
    config.gpio_cb_arg = NULL;

    return soc_gpio_set_config(dev, bit, &config);
}

DRIVER_API_RC soc_gpio_port_deconfig(struct device *dev)
{
    unsigned int i;
    gpio_port_cfg_data_t config;
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    // Default configuration (input pin without interrupt)
    config.gpio_type = 0;
    config.is_interrupt = 0;
    config.int_type =  0;
    config.int_bothedge =  0;
    config.int_polarity = 0;
    config.int_debounce = 0;
    config.int_ls_sync =  0;

    // TODO: use memset
    for(i=0; i<gpio_dev->no_bits; i++) {
        config.gpio_cb[i] = NULL;
        config.gpio_cb_arg[i] = NULL;
    }
    return soc_gpio_set_port_config(dev, &config);
}

DRIVER_API_RC soc_gpio_write(struct device *dev, uint8_t bit, bool value)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;

    // Check pin index
    if (bit >= gpio_dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    /* read/modify/write bit */
    if (value) {
        SET_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_SWPORTA_DR), (uint32_t)bit);
    } else {
        CLEAR_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_SWPORTA_DR), (uint32_t)bit);
    }

    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_write_port(struct device *dev, uint32_t value)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_SWPORTA_DR) = value;
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_read(struct device *dev, uint8_t bit, bool *value)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    // Check pin index
    if (bit >= gpio_dev->no_bits) {
        return DRV_RC_CONTROLLER_NOT_ACCESSIBLE;
    }
    if (MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_SWPORTA_DDR) & (1 << bit)) {
        return DRV_RC_INVALID_OPERATION;          /* not configured as input */
    }

    *value = !!(MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_EXT_PORTA) & (1 << bit));
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_read_port(struct device *dev, uint32_t *value)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    *value = MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_EXT_PORTA);
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_mask_interrupt(struct device *dev, uint8_t bit)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    SET_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_INTMASK), (uint32_t)bit);
    return DRV_RC_OK;
}

DRIVER_API_RC soc_gpio_unmask_interrupt(struct device *dev, uint8_t bit)
{
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;
    CLEAR_MMIO_BIT((volatile uint32_t *)(gpio_dev->reg_base+SOC_GPIO_INTMASK), (uint32_t)bit);
    return DRV_RC_OK;
}

static void soc_gpio_ISR_proc(struct device *dev)
{
    unsigned int i;
    gpio_info_pt gpio_dev = (gpio_info_pt)dev->priv;

    // Save interrupt status
    uint32_t status = MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_INTSTATUS);
    // Clear interrupt flag (write 1 to clear)
    MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_PORTA_EOI) = status;

    for (i=0; i<gpio_dev->no_bits; i++) {
        if ((status & (1 << i))) {
            if (gpio_dev->gpio_cb[i]) {
                (gpio_dev->gpio_cb[i])(
                        !!(MMIO_REG_VAL_FROM_BASE(gpio_dev->reg_base, SOC_GPIO_EXT_PORTA) & (1 << i)),
                        gpio_dev->gpio_cb_arg[i]);
            } else {
                pr_info(LOG_MODULE_DRV, "spurious isr: %d on %d", i, dev->id);
                soc_gpio_mask_interrupt(dev, i);
            }
        }
    }
}
