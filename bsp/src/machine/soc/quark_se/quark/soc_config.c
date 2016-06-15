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

#include "drivers/spi_flash.h"
#include "drivers/serial_bus_access.h"
#include "drivers/intel_qrk_rtc.h"
#include "drivers/soc_flash.h"
#include "drivers/usb_pm.h"
#include "storage.h"
#include "board.h"
#include "drivers/soc_gpio.h"
#include "machine/soc/quark_se/soc_config.h"
#include "infra/wakelock_ids.h"
#include "machine/soc/quark_se/arc/soc_register.h"
#include "machine/soc/quark_se/quark/uart_tcmd_client.h"
#include "drivers/soc_comparator.h"
#include "quark_se_mapping.h"
#include "drivers/intel_qrk_aonpt.h"
#include "drivers/clk_system.h"

#define PLATFORM_INIT(devices, buses) \
	init_devices(devices, (unsigned int)(sizeof(devices) / sizeof(*devices)), \
		     buses, (unsigned int)(sizeof(buses) / sizeof(*buses)))

#ifdef CONFIG_BOARD_ARDUINO101
#define SPI_FLASH_CS SPI_SE_1
#else
#define SPI_FLASH_CS SPI_SE_3
#endif

DECLARE_SUSPEND_BLOCKERS(
#if defined(CONFIG_USB_PM_SUSPEND_BLOCKERS)
	{
		.cb = usb_pm_is_suspend_allowed,
		.dev_id = USB_PM_ID
	},
#endif
	);

#ifdef CONFIG_INTEL_QRK_SPI
// sba spi0 devices (to statically init device tree)
static struct sba_device *qrk_se_sba_spi_0_devices[] = {
#ifdef CONFIG_SPI_FLASH_INTEL_QRK
	&(struct sba_device){
		.dev.id = SPI_FLASH_0_ID,
		// .dev.driver = &mx25u12835f_driver,
		.dev.driver = &spi_flash_driver,
		.addr.cs = SPI_FLASH_CS
	}
#endif
};
static struct sba_device *qrk_se_sba_spi_1_devices[] = {};
#endif

#ifdef CONFIG_INTEL_QRK_SPI
// Configuration of sba master devices (bus)
static struct sba_master_cfg_data qrk_se_sba_spi_0_cfg = {
	.bus_id				= SBA_SPI_MASTER_0,
	.config.spi_config		= {
		.speed			= 250,                  /*!< SPI bus speed in KHz   */
		.txfr_mode		= SPI_TX_RX,            /*!< Transfer mode */
		.data_frame_size	= SPI_8_BIT,            /*!< Data Frame Size ( 4 - 16 bits ) */
		.slave_enable		= SPI_FLASH_CS,         /*!< Slave Enable, Flash Memory is on CS3 or CS1 */
		.bus_mode		= SPI_BUSMODE_0,        /*!< SPI bus mode is 0 by default */
		.spi_mode_type		= SPI_MASTER,           /*!< SPI 0 is in master mode */
		.loopback_enable	= 0                     /*!< Loopback disabled by default */
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= PERIPH_CLK_GATE_CTRL,
		.bits_mask		= SPI0_CLK_GATE_MASK,
	},
};

static struct sba_master_cfg_data qrk_se_sba_spi_1_cfg = {
	.bus_id				= SBA_SPI_MASTER_1,
	.config.spi_config		= {
		.speed			= 250,                  /*!< SPI bus speed in KHz   */
		.txfr_mode		= SPI_TX_RX,            /*!< Transfer mode */
		.data_frame_size	= SPI_8_BIT,            /*!< Data Frame Size ( 4 - 16 bits ) */
		.slave_enable		= SPI_SE_3,             /*!< Slave Enable, NFC device */
		.bus_mode		= SPI_BUSMODE_0,        /*!< SPI bus mode is 0 by default */
		.spi_mode_type		= SPI_MASTER,           /*!< SPI 0 is in master mode */
		.loopback_enable	= 0                     /*!< Loopback disabled by default */
	}
};
#endif

// i2c
#ifdef CONFIG_INTEL_QRK_I2C
// sba i2c1 devices (to statically init device tree)
static struct sba_device *qrk_se_sba_i2c_0_devices[] = {
};
static struct sba_device *qrk_se_sba_i2c_1_devices[] = {
#ifdef CONFIG_DRV2605
	&(struct sba_device){
		.dev.id = DRV2605_ID,
		.dev.driver = &drv2605_driver,
		.dev.priv = &(struct haptic_info) {},
		.addr.slave_addr = 0x5A,
	},
#endif
#ifdef CONFIG_NFC_STN54E
	&(struct sba_device){
		.dev.id = NFC_STN54E_ID,
		.dev.driver = &nfc_stn54e_driver,
		.dev.priv = &(struct nfc_stn54e_info){
			.wakelock.id = NFC_WAKELOCK,
			.gpio_port_id = SOC_GPIO_32_ID,
			.stn_reset_pin = CONFIG_STN54E_RST_PIN,
			.stn_irq_pin = CONFIG_STN54E_IRQ_OUT_PIN,
#ifdef CONFIG_STN54E_HAS_PWR_EN
			.stn_pwr_en_pin = CONFIG_STN54E_PWR_EN_PIN,
#endif
#ifdef CONFIG_STN54E_HAS_BOOSTER
			.booster_reset_pin = CONFIG_STN54E_BOOSTER_RST_PIN,
#endif
		},
		.addr.slave_addr = 0x08,
	},
#endif
};
#endif

#ifdef CONFIG_INTEL_QRK_I2C
static struct sba_master_cfg_data qrk_se_sba_i2c_0_cfg = {
	.bus_id				= SBA_I2C_MASTER_0,
	.config.i2c_config		= {
		.speed			= I2C_SLOW,
		.addressing_mode	= I2C_7_Bit,
		.mode_type		= I2C_MASTER
	},
	.clk_gate_info			= &(struct clk_gate_info_s) {
		.clk_gate_register	= PERIPH_CLK_GATE_CTRL,
		.bits_mask		= I2C0_CLK_GATE_MASK,
	},
};
static struct sba_master_cfg_data qrk_se_sba_i2c_1_cfg = {
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
// Array of qrk_se bus devices (sba buses etc ...)
static struct bus qrk_se_platform_bus_devices[] = {
#ifdef CONFIG_INTEL_QRK_SPI
	{
		.id = SBA_SPI0_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_spi_0_cfg,
		LINK_DEVICES_TO_BUS(qrk_se_sba_spi_0_devices),
		LINK_BUSES_TO_BUS_NULL
	},
	{
		.id = SBA_SPI1_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_spi_1_cfg,
		LINK_DEVICES_TO_BUS(qrk_se_sba_spi_1_devices),
		LINK_BUSES_TO_BUS_NULL
	},
#endif
#ifdef CONFIG_INTEL_QRK_I2C
	{
		.id = SBA_I2C0_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_i2c_0_cfg,
		LINK_DEVICES_TO_BUS(qrk_se_sba_i2c_0_devices),
		LINK_BUSES_TO_BUS_NULL
	},
	{
		.id = SBA_I2C1_ID,
		.driver = &serial_bus_access_driver,
		.priv = &qrk_se_sba_i2c_1_cfg,
		LINK_DEVICES_TO_BUS(qrk_se_sba_i2c_1_devices),
		LINK_BUSES_TO_BUS_NULL
	}
#endif
};

// Array of qrk_se devices (on die memory, spi slave etc ...)
static struct device *qrk_se_platform_devices[] = {
	&(struct device){
		.id = QRK_CLK_GATE,
		.driver = &clk_system_driver,
		.priv = &(struct clk_gate_info_s)
		{
			.clk_gate_register = PERIPH_CLK_GATE_CTRL,
			.bits_mask = QRK_CLK_GATE_INIT_VALUE,
		}
	},
#ifdef CONFIG_INTEL_QRK_RTC
	&(struct device){
		.id = RTC_ID,
		.driver = &rtc_driver,
		.priv = &(struct rtc_pm_data){
			.clk_gate_info = &(struct clk_gate_info_s){
				.clk_gate_register = PERIPH_CLK_GATE_CTRL,
				.bits_mask = RTC_CLK_GATE_MASK,
			},
		},
	},
#endif
#ifdef CONFIG_INTEL_QRK_WDT
	&(struct device){
		.id = WDT_ID,
		.driver = &watchdog_driver,
		.priv = &(struct wdt_pm_data){
			.clk_gate_info = &(struct clk_gate_info_s){
				.clk_gate_register = PERIPH_CLK_GATE_CTRL,
				.bits_mask = WDT_CLK_GATE_MASK,
			},
		},
	},
#endif
#ifdef CONFIG_INTEL_QRK_AON_PT
	&(struct device){
		.id = AON_PT_ID,
		.driver = &aonpt_driver
	},
#endif
#ifdef CONFIG_UART_PM_NS16550
#ifdef CONFIG_IPC_UART_NS16550
	&(struct device){
		.id = IPC_UART_ID,
		.driver = &ipc_uart_ns16550_driver,
		.priv = &(struct ipc_uart_info){
			.uart_num = 0,
			.irq_vector = COM1_INT_LVL,
			.irq_mask = INT_UART_0_MASK,
			.rx_wl.id = IPC_UARTRX_WAKELOCK,
			.tx_wl.id = IPC_UARTTX_WAKELOCK
		},
	},
#endif
#ifdef CONFIG_IPC_UART
	/* FIXME: CONFIG_IPC_UART_BAUDRATE is "IPC_UART" specific
	 *        hence we must check if CONFIG_IPC_UART is set before
	 *        allocating this device*/
	&(struct device){
		.id = UART0_PM_ID,
		.driver = &ns16550_pm_driver,
		.priv = &(struct ns16550_pm_device){
			.uart_num = 0,
			.vector = SOC_UART0_INTERRUPT,
			.uart_int_mask = INT_UART_0_MASK,
			.init_info = &(struct uart_init_info){
				.sys_clk_freq = UART_XTAL_FREQ,
				.baud_rate = CONFIG_IPC_UART_BAUDRATE,
				.options = UART_OPTION_AFCE,
				.int_pri = COM1_INT_PRI,
			},
		},
	},
#endif
	&(struct device){
		.id = UART1_PM_ID,
		.driver = &ns16550_pm_driver,
		.priv = &(struct ns16550_pm_device){
			.uart_num = 1,
#ifdef CONFIG_TCMD
			.uart_rx_callback = uart_console_input,
#endif
			.vector = SOC_UART1_INTERRUPT,
			.uart_int_mask = INT_UART_1_MASK,
			.init_info = &(struct uart_init_info){
				.sys_clk_freq = UART_XTAL_FREQ,
				.baud_rate = COM2_BAUD_RATE,
				.options = 0,
				.int_pri = COM2_INT_PRI,
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
#ifdef CONFIG_SOC_COMPARATOR
	&(struct device){
		.id = COMPARATOR_ID,
		.driver = &soc_comparator_driver,
		.priv = (struct cmp_cb[CMP_COUNT]) {}
	},
#endif
#ifdef CONFIG_SOC_FLASH
	&(struct device){
		.id = SOC_FLASH_ID,
		.driver = &soc_flash_driver
	},
#endif
#if defined(CONFIG_USB_PM) && defined(CONFIG_QUARK_SE_CURIE)
	&(struct device){
		.id = USB_PM_ID,
		.driver = &usb_pm_driver,
		.priv = &(struct usb_pm_info) {
			.evt_dev_id = COMPARATOR_ID,
			.interrupt_source = USB_COMPARATOR_IRQ_SOURCE,
			.source_pin = 7, // USB Vbus is connected on comparator 7 for CTB
#ifdef CONFIG_SOC_GPIO_32
			.vusb_enable_port = SOC_GPIO_32_ID,
			.vusb_enable_pin = 28,
#endif
		}
	},
#endif
#ifdef CONFIG_SOC_LED
	&(struct device){
		.id = SOC_LED_ID,
		.driver = &soc_led_driver,
	},
#endif
#ifdef CONFIG_DISPLAY_HD44780
	&(struct device){
		.id = HD44780_ID,
		.driver = &hd44780_driver,
	},
#endif
#ifdef CONFIG_PWM
	&(struct device){
		.id = PWM_ID,
		.driver = &pwm_driver,
		.priv = &(struct pwm_pm_info) {
			.timer_info = (struct timer_info_s[QRK_PWM_NPWM]) {},
			.running_pwm = 0,
			.clk_gate_info = &(struct clk_gate_info_s) {
				.clk_gate_register = PERIPH_CLK_GATE_CTRL,
				.bits_mask = PWM_CLK_GATE_MASK,
			},
		},
	},
#endif
};

int init_all_devices(void)
{
	// Init plateform devices and buses
	return PLATFORM_INIT(qrk_se_platform_devices, qrk_se_platform_bus_devices);
}

// Array of qrk_se flash memory partitioning
flash_partition_t storage_configuration[] =
{
	{
		.partition_id = APPLICATION_DATA_PARTITION_ID,
		.flash_id = APPLICATION_DATA_FLASH_ID,
		.start_block = APPLICATION_DATA_START_BLOCK,
		.end_block = APPLICATION_DATA_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_NON_PERSISTENT
	},
	{
		.partition_id = DEBUGPANIC_PARTITION_ID,
		.flash_id = DEBUGPANIC_FLASH_ID,
		.start_block = DEBUGPANIC_START_BLOCK,
		.end_block = DEBUGPANIC_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_NON_PERSISTENT
	},
	{
		.partition_id = FACTORY_RESET_NON_PERSISTANT_PARTITION_ID,
		.flash_id = FACTORY_RESET_NON_PERSISTANT_FLASH_ID,
		.start_block = FACTORY_RESET_NON_PERSISTANT_START_BLOCK,
		.end_block = FACTORY_RESET_NON_PERSISTANT_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_NON_PERSISTENT
	},
	{
		.partition_id = FACTORY_RESET_PERSISTANT_PARTITION_ID,
		.flash_id = FACTORY_RESET_PERSISTANT_FLASH_ID,
		.start_block = FACTORY_RESET_PERSISTANT_START_BLOCK,
		.end_block = FACTORY_RESET_PERSISTANT_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_PERSISTENT
	},
	{
		.partition_id = FACTORY_SETTINGS_PARTITION_ID,
		.flash_id = FACTORY_SETTINGS_FLASH_ID,
		.start_block = FACTORY_SETTINGS_START_BLOCK,
		.end_block = FACTORY_SETTINGS_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_PERSISTENT
	},
	{
		.partition_id = SPI_APPLICATION_DATA_PARTITION_ID,
		.flash_id = SPI_APPLICATION_DATA_FLASH_ID,
		.start_block = SPI_APPLICATION_DATA_START_BLOCK,
		.end_block = SPI_APPLICATION_DATA_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_NON_PERSISTENT
	},
	{
		.partition_id = SPI_DEBUGPANIC_PARTITION_ID,
		.flash_id = SPI_DEBUGPANIC_FLASH_ID,
		.start_block = SPI_DEBUGPANIC_START_BLOCK,
		.end_block = SPI_DEBUGPANIC_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_NON_PERSISTENT
	},
	{
		.partition_id = SPI_FACTORY_RESET_NON_PERSISTANT_PARTITION_ID,
		.flash_id = SPI_FACTORY_RESET_NON_PERSISTANT_FLASH_ID,
		.start_block = SPI_FACTORY_RESET_NON_PERSISTANT_START_BLOCK,
		.end_block = SPI_FACTORY_RESET_NON_PERSISTANT_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_NON_PERSISTENT
	},
	{
		.partition_id = SPI_FACTORY_RESET_PERSISTANT_PARTITION_ID,
		.flash_id = SPI_FACTORY_RESET_PERSISTANT_FLASH_ID,
		.start_block = SPI_FACTORY_RESET_PERSISTANT_START_BLOCK,
		.end_block = SPI_FACTORY_RESET_PERSISTANT_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_PERSISTENT
	},
	{
		.partition_id = SPI_FACTORY_SETTINGS_PARTITION_ID,
		.flash_id = SPI_FACTORY_SETTINGS_FLASH_ID,
		.start_block = SPI_FACTORY_SETTINGS_START_BLOCK,
		.end_block = SPI_FACTORY_SETTINGS_END_BLOCK,
		.factory_reset_state = FACTORY_RESET_PERSISTENT
	}
};

// Array of qrk_se flash memory devices
const flash_device_t flash_devices[] =
{
	{
		.flash_id = EMBEDDED_FLASH_ID,
		.nb_blocks = EMBEDDED_FLASH_NB_BLOCKS,
		.block_size = EMBEDDED_FLASH_BLOCK_SIZE,
		.flash_location = EMBEDDED_FLASH
	},
	{
		.flash_id = SERIAL_FLASH_ID,
		.nb_blocks = SERIAL_FLASH_NB_BLOCKS,
		.block_size = SERIAL_FLASH_BLOCK_SIZE,
		.flash_location = SERIAL_FLASH
	}
};
