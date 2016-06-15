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

#include "infra/wakelock_ids.h"
#include "machine/soc/quark_se/soc_config.h"
#include "drivers/serial_bus_access.h"
#include "drivers/eiaextensions.h"
#include "machine/soc/quark_se/arc/soc_register.h"
#include "drivers/io_config.h"
#include "drivers/soc_gpio.h"
#include "drivers/ss_gpio_iface.h"
#include "drivers/soc_comparator.h"
#include "drivers/usb_pm.h"
#include "drivers/nordic_pm.h"
#include "drivers/clk_system.h"

#define PLATFORM_INIT(devices, buses) \
	init_devices(devices, (unsigned int)(sizeof(devices) / sizeof(*devices)), \
		     buses, (unsigned int)(sizeof(buses) / sizeof(*buses)))

DECLARE_SUSPEND_BLOCKERS(
#if defined(CONFIG_USB_PM_SUSPEND_BLOCKERS)
	{
		.cb = usb_pm_is_suspend_allowed,
		.dev_id = USB_PM_ID
	},
#endif
#ifdef CONFIG_NORDIC_SUSPEND_BLOCKER_PM
	{
		.cb = nordic_pm_can_sleep,
		.dev_id = NORDIC_PM_ID
	},
#endif
	);
#ifdef CONFIG_SS_SPI
// sba ss_spi1 devices (to statically init device tree)
static struct sba_device *qrk_se_sba_ss_spi_1_devices[] = {
#ifdef CONFIG_BMI160
#ifdef CONFIG_BMI160_SPI
	&(struct sba_device){
		.dev.id = SPI_BMI160_ID,
		.dev.driver = &spi_bmi160_driver,
		.addr.cs = BMI160_PRIMARY_BUS_ADDR,
	},
#endif
#endif
};
#endif
// sba ss_i2c0 devices (to statically init device tree)
#ifdef CONFIG_SS_I2C
static struct sba_device *qrk_se_sba_ss_i2c_0_devices[] = {
#ifdef CONFIG_BMI160
#ifndef CONFIG_BMI160_SPI
	&(struct sba_device){
		.dev.id = I2C_BMI160_ID,
		.dev.driver = &i2c_bmi160_driver,
		.addr.slave_addr = BMI160_PRIMARY_BUS_ADDR,
	},
#endif
#endif
};
#endif

// Configuration of sba master devices (bus)
#ifdef CONFIG_INTEL_QRK_SPI
static struct sba_master_cfg_data qrk_se_sba_soc_spi_0_cfg = {
	.bus_id				= SBA_SPI_MASTER_0,
	.config.spi_config		= {
		.speed			= 250,                  /*!< SPI bus speed in KHz */
		.txfr_mode		= SPI_TX_RX,            /*!< Transfer mode */
		.data_frame_size	= SPI_8_BIT,            /*!< Data Frame Size ( 4 - 16 bits ) */
		.slave_enable		= SPI_SE_3,             /*!< Slave Enable, Flash Memory is on CS3 */
		.bus_mode		= SPI_BUSMODE_0,        /*!< SPI bus mode is 0 by default */
		.spi_mode_type		= SPI_MASTER,           /*!< SPI 0 is in master mode */
		.loopback_enable	= 0                     /*!< Loopback disabled by default */
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= PERIPH_CLK_GATE_CTRL,
		.bits_mask		= SPI0_CLK_GATE_MASK,
	},
};
#endif
#ifdef CONFIG_INTEL_QRK_I2C
static struct sba_master_cfg_data qrk_se_sba_soc_i2c_1_cfg = {
	.bus_id				= SBA_I2C_MASTER_1,
	.config.i2c_config		= {
		.speed			= I2C_SLOW,
		.addressing_mode	= I2C_7_Bit,
		.mode_type		= I2C_MASTER
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= PERIPH_CLK_GATE_CTRL,
		.bits_mask		= I2C1_CLK_GATE_MASK,
	},
};
#endif
#ifdef CONFIG_SS_SPI
static struct sba_master_cfg_data qrk_se_sba_ss_spi_0_cfg = {
	.bus_id				= SBA_SS_SPI_MASTER_0,
	.config.spi_config		= {
		.speed			= 20,                   /*!< SPI bus speed in KHz */
		.txfr_mode		= SPI_TX_RX,            /*!< Transfer mode */
		.data_frame_size	= SPI_8_BIT,            /*!< Data Frame Size ( 4 - 16 bits ) */
		.slave_enable		= SPI_SE_1,             /*!< Slave Enable, Flash Memory is on CS3  */
		.bus_mode		= SPI_BUSMODE_0,        /*!< SPI bus mode is 0 by default */
		.spi_mode_type		= SPI_MASTER,           /*!< SPI 0 is in master mode */
		.loopback_enable	= 0                     /*!< Loopback disabled by default */
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= SS_PERIPH_CLK_GATE_CTL,
		.bits_mask		= SS_SPI0_CLK_GATE_MASK,
	},
};

static struct sba_master_cfg_data qrk_se_sba_ss_spi_1_cfg = {
	.bus_id				= SBA_SS_SPI_MASTER_1,
	.config.spi_config		= {
		.speed			= 250,                          /*!< SPI bus speed in KHz */
		.txfr_mode		= SPI_TX_RX,                    /*!< Transfer mode */
		.data_frame_size	= SPI_8_BIT,                    /*!< Data Frame Size ( 4 - 16 bits ) */
		.slave_enable		= BMI160_PRIMARY_BUS_ADDR,      /*!< Slave Enable, Flash Memory is on CS3  */
		.bus_mode		= SPI_BUSMODE_0,                /*!< SPI bus mode is 0 by default */
		.spi_mode_type		= SPI_MASTER,                   /*!< SPI 0 is in master mode */
		.loopback_enable	= 0                             /*!< Loopback disabled by default */
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= SS_PERIPH_CLK_GATE_CTL,
		.bits_mask		= SS_SPI1_CLK_GATE_MASK,
	},
};
#endif
#ifdef CONFIG_SS_I2C
static struct sba_master_cfg_data qrk_se_sba_ss_i2c_0_cfg = {
	.bus_id				= SBA_SS_I2C_MASTER_0,
	.config.i2c_config		= {
		.speed			= I2C_FAST,
		.addressing_mode	= I2C_7_Bit,
		.mode_type		= I2C_MASTER,
		.slave_adr		= BMI160_I2C_ADDR1
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= SS_PERIPH_CLK_GATE_CTL,
		.bits_mask		= SS_I2C0_CLK_GATE_MASK,
	},
};
#endif
// Array of qrk_se bus devices (sba buses etc ...)
static struct bus qrk_se_platform_bus_devices[] = {
#ifdef CONFIG_INTEL_QRK_SPI
	{
		.id = SBA_SOC_SPI_0_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_soc_spi_0_cfg,
		LINK_DEVICES_TO_BUS_NULL,
		LINK_BUSES_TO_BUS_NULL
	},
#endif
#ifdef CONFIG_INTEL_QRK_I2C
	{
		.id = SBA_SOC_I2C1,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_soc_i2c_1_cfg,
		LINK_DEVICES_TO_BUS_NULL,
		LINK_BUSES_TO_BUS_NULL
	},
#endif
#ifdef CONFIG_SS_SPI
	{
		.id = SBA_SS_SPI_0_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_ss_spi_0_cfg,
		LINK_DEVICES_TO_BUS_NULL,
		LINK_BUSES_TO_BUS_NULL
	},
	{
		.id = SBA_SS_SPI_1_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_ss_spi_1_cfg,
		LINK_DEVICES_TO_BUS(qrk_se_sba_ss_spi_1_devices),
		LINK_BUSES_TO_BUS_NULL
	},
#endif
#ifdef CONFIG_SS_I2C
	{
		.id = SBA_SS_I2C_0_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_ss_i2c_0_cfg,
		LINK_DEVICES_TO_BUS(qrk_se_sba_ss_i2c_0_devices),
		LINK_BUSES_TO_BUS_NULL
	}
#endif
};

// Array of qrk_se devices (on die memory, spi slave etc ...)
static struct device *qrk_se_platform_devices[] = {
#ifdef CONFIG_CLK_SYSTEM
	&(struct device){
		.id = SS_CLK_GATE,
		.driver = &clk_system_driver,
		.priv = &(struct clk_gate_info_s)
		{
			.clk_gate_register = SS_PERIPH_CLK_GATE_CTL,
			.bits_mask = SS_CLK_GATE_INIT_VALUE,
		}
	},
	&(struct device){
		.id = MLAYER_CLK_GATE,
		.driver = &clk_system_driver,
		.priv = &(struct clk_gate_info_s)
		{
			.clk_gate_register = MLAYER_AHB_CTL,
			.bits_mask = MLAYER_CLK_GATE_INIT_VALUE,
		}
	},
#endif
#ifdef CONFIG_SS_ADC
	&(struct device){
		.id = SS_ADC_ID,
		.driver = &ss_adc_driver,
		.priv = &(struct adc_info_t){
			.reg_base = AR_IO_ADC0_SET,
			.creg_slv = AR_IO_CREG_SLV0_OBSR,
			.creg_mst = AR_IO_CREG_MST0_CTRL,
			.rx_vector = IO_ADC0_INT_IRQ,
			.err_vector = IO_ADC0_INT_ERR,
			.fifo_tld = IO_ADC0_FS / 2,
			.adc_irq_mask = SCSS_REGISTER_BASE +
					INT_SS_ADC_IRQ_MASK,
			.adc_err_mask = SCSS_REGISTER_BASE +
					INT_SS_ADC_ERR_MASK,
			.wakelock.id = ADC_WAKELOCK,
			.clk_gate_info = &(struct clk_gate_info_s){
				.clk_gate_register = SS_PERIPH_CLK_GATE_CTL,
				.bits_mask = SS_ADC_CLK_GATE_MASK,
			},
		},
	},
#endif
#ifdef CONFIG_SOC_GPIO_AON
	&(struct device){
		.id = SOC_GPIO_AON_ID,
		.driver = &soc_gpio_driver,
		.priv = &(gpio_info_t){
			.reg_base = SOC_GPIO_AON_BASE_ADDR,
			.no_bits = SOC_GPIO_AON_BITS,
			.gpio_int_mask = INT_AON_GPIO_MASK,
			.vector = SOC_GPIO_AON_INTERRUPT,
			.gpio_isr = gpio_aon_isr,
			.gpio_cb = (gpio_callback_fn[SOC_GPIO_AON_BITS]) { NULL },
			.gpio_cb_arg = (void *[SOC_GPIO_AON_BITS]) { NULL }
		}
	},
#endif
#ifdef CONFIG_SOC_GPIO_32
	&(struct device){
		.id = SOC_GPIO_32_ID,
		.driver = &soc_gpio_driver,
		.priv = &(gpio_info_t) {
			.reg_base = SOC_GPIO_BASE_ADDR,
			.no_bits = SOC_GPIO_32_BITS,
			.gpio_int_mask = INT_GPIO_MASK,
			.vector = SOC_GPIO_INTERRUPT,
			.gpio_isr = gpio_isr,
			.gpio_cb = (gpio_callback_fn[SOC_GPIO_32_BITS]) { NULL },
			.gpio_cb_arg = (void *[SOC_GPIO_32_BITS]) { NULL }
		}
	},
#endif
#ifdef CONFIG_SS_GPIO
	&(struct device){
		.id = SS_GPIO_8B0_ID,
		.driver = &ss_gpio_driver,
		.priv = &(gpio_info_t) {
			.reg_base = AR_IO_GPIO_8B0_SWPORTA_DR,
			.no_bits = SS_GPIO_8B0_BITS,
			.gpio_int_mask = INT_SS_GPIO_0_INTR_MASK,
			.vector = IO_GPIO_8B0_INT_INTR_FLAG,
			.gpio_isr = ss_gpio_8b0_ISR,
			.gpio_cb = (gpio_callback_fn[SS_GPIO_8B0_BITS]) { NULL },
			.gpio_cb_arg = (void *[SS_GPIO_8B0_BITS]) { NULL }
		}
	},
	&(struct device){
		.id = SS_GPIO_8B1_ID,
		.driver = &ss_gpio_driver,
		.priv = &(gpio_info_t) {
			.reg_base = AR_IO_GPIO_8B1_SWPORTA_DR,
			.no_bits = SS_GPIO_8B1_BITS,
			.gpio_int_mask = INT_SS_GPIO_1_INTR_MASK,
			.vector = IO_GPIO_8B1_INT_INTR_FLAG,
			.gpio_isr = ss_gpio_8b1_ISR,
			.gpio_cb = (gpio_callback_fn[SS_GPIO_8B1_BITS]) { NULL },
			.gpio_cb_arg = (void *[SS_GPIO_8B1_BITS]) { NULL }
		}
	},
#endif
#ifdef CONFIG_SOC_COMPARATOR
	&(struct device){
		.id = COMPARATOR_ID,
		.driver = &soc_comparator_driver,
		.priv = (struct cmp_cb[CMP_COUNT]) {}
	},
#endif
#if defined(CONFIG_USB_PM) && defined(CONFIG_BOARD_QUARK_SE_APP)
	&(struct device){
		.id = USB_PM_ID,
		.driver = &usb_pm_driver,
		.priv = &(struct usb_pm_info) {
			.evt_dev_id = SOC_GPIO_AON_ID,
			.interrupt_source = USB_AON_IRQ_SOURCE,
			.source_pin = 0, // USB Vbus is connected on gpio aon 0 for APP
#ifdef CONFIG_SOC_GPIO_32
			.vusb_enable_port = SOC_GPIO_32_ID,
			.vusb_enable_pin = 28,
#endif
		}
	},
#endif
#ifdef CONFIG_NORDIC_SUSPEND_BLOCKER_PM
	&(struct device){
		.id = NORDIC_PM_ID,
		.driver = &nordic_pm_driver,
		.priv = &(struct nordic_pm_info) {
			.gpio_dev_id = BLE_QRK_SE_INT_PORT,
			.wakeup_pin = BLE_QRK_SE_INT_PIN // gpio aon 5 is BLE_QRK_SE_INT on Curie
		}
	},
#endif
};

int init_all_devices()
{
	// Init plateform devices and buses
	return PLATFORM_INIT(qrk_se_platform_devices, qrk_se_platform_bus_devices);
}
