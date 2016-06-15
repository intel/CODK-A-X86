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
/* aae_pinmux.c - pin out mapping for the AAE board */


#include <pinmux.h>

struct pin_config mux_config[CONFIG_PINMUX_NUM_PINS] = {
	/* pin, selected mode           ball, mode A, mode B, mode C */
	/* Analog Inputs */
	{ 0, PINMUX_FUNC_B }, /* F02, gpio_0, ain_0, spi_s_cs */
	{ 1, PINMUX_FUNC_B }, /* G04, gpio_1, ain_1, spi_s_miso */
	{ 2, PINMUX_FUNC_B }, /* H05, gpio_2, ain_2, spi_s_sck */
	{ 3, PINMUX_FUNC_B }, /* J06, gpio_3, ain_3, spi_s_mosi */
	{ 4, PINMUX_FUNC_B }, /* K06, gpio_4, ain_4, NA */
	{ 5, PINMUX_FUNC_B }, /* L06, gpio_5, ain_5, NA */
	{ 6, PINMUX_FUNC_A }, /* H04, gpio_6, ain_6, NA */
	{ 7, PINMUX_FUNC_B }, /* G03, gpio_7, ain_7, NA */
	/* Sensor subsystem GPIOs */
	{ 8, PINMUX_FUNC_C },  /* L05, gpio_ss_0, ain_8, uart1_cts */
	{ 9, PINMUX_FUNC_C },  /* M05, gpio_ss_1, ain_9, uart1_rts */
	{ 10, PINMUX_FUNC_A }, /* K05, gpio_ss_2, ain_10 */
	{ 11, PINMUX_FUNC_A }, /* G01, gpio_ss_3, ain_11 */
	{ 12, PINMUX_FUNC_A }, /* J04, gpio_ss_4, ain_12 */
	{ 13, PINMUX_FUNC_A }, /* G02, gpio_ss_5, ain_13 */
	{ 14, PINMUX_FUNC_A }, /* F01, gpio_ss_6, ain_14 */
	{ 15, PINMUX_FUNC_A }, /* J05, gpio_ss_7, ain_15 */
	/* Debug UART */
	{ 16, PINMUX_FUNC_C }, /* L04, gpio_ss_8, ain_16, uart1_txd */
	{ 17, PINMUX_FUNC_C }, /* M04, gpio_ss_9, ain_17, uart1_rxd */
	/* BT UART */
	{ 18, PINMUX_FUNC_A }, /* K04, uart0_rx, ain_18, NA */
	{ 19, PINMUX_FUNC_A }, /* B02, uart0_tx, gpio_31, NA */
	{ 20, PINMUX_FUNC_A }, /* C01, i2c0_scl, NA, NA */
	{ 21, PINMUX_FUNC_A }, /* C02, i2c0_sda, NA, NA */
	{ 22, PINMUX_FUNC_A }, /* D01, i2c1_scl, NA, NA */
	{ 23, PINMUX_FUNC_A }, /* D02, i2c1_sda, NA, NA */
	/* Digital sensors on sensor subsystem i2c0 */
	{ 24, PINMUX_FUNC_A }, /* E01, i2c0_ss_sda, NA, NA */
	{ 25, PINMUX_FUNC_A }, /* E02, i2c0_ss_scl, NA, NA */
	/* Digital sensors on sensor subsystem i2c1 */
	{ 26, PINMUX_FUNC_A }, /* B03, i2c1_ss_sda, NA, NA */
	{ 27, PINMUX_FUNC_A }, /* A03, i2c1_ss_scl, NA, NA */
	/* Digital sensors on sensor system on SPI0 */
	{ 28, PINMUX_FUNC_A }, /* C03, spi0_ss_miso, NA, NA */
	{ 29, PINMUX_FUNC_A }, /* E03, spi0_ss_mosi, NA, NA */
	{ 30, PINMUX_FUNC_A }, /* D03, spi0_ss_sck, NA, NA */
	{ 31, PINMUX_FUNC_A }, /* D04, spi0_ss_cs0, NA, NA */
	{ 32, PINMUX_FUNC_A }, /* C04, spi0_ss_cs1, NA, NA */
	{ 33, PINMUX_FUNC_A }, /* B04, spi0_ss_cs2, gpio_29, NA */
	{ 34, PINMUX_FUNC_A }, /* A04, spi0_ss_cs3, gpio_30, NA */
	/* Digital sensors on sensor system on SPI1 */
	{ 35, PINMUX_FUNC_A }, /* B05, spi1_ss_miso, NA, NA */
	{ 36, PINMUX_FUNC_A }, /* C05, spi1_ss_mosi, NA, NA */
	{ 37, PINMUX_FUNC_A }, /* D05, spi1_ss_sck, NA, NA */
	{ 38, PINMUX_FUNC_A }, /* E05, spi1_ss_cs0, NA, NA */
	{ 39, PINMUX_FUNC_A }, /* E04, spi1_ss_cs1, NA, NA */
	/* BT UART */
	{ 40, PINMUX_FUNC_B }, /* A06, spi1_ss_cs2, uart0_cts, NA */
	{ 41, PINMUX_FUNC_B }, /* B06, spi1_ss_cs3, uart0_rts, NA */
	/* Host GPIOs */
	{ 42, PINMUX_FUNC_A }, /* C06, gpio_8, spi1_m_sck, NA */
	{ 43, PINMUX_FUNC_A }, /* D06, gpio_9, spi1_m_miso, NA */
	{ 44, PINMUX_FUNC_A }, /* E06, gpio_10, spi1_m_mosi, NA */
	{ 45, PINMUX_FUNC_A }, /* D07, gpio_11, spi1_m_cs0, NA */
	{ 46, PINMUX_FUNC_A }, /* C07, gpio_12, spi1_m_cs1, NA */
	{ 47, PINMUX_FUNC_A }, /* B07, gpio_13, spi1_m_cs2, NA */
	{ 48, PINMUX_FUNC_A }, /* A07, gpio_14, spi1_m_cs3, NA */
	/* Audio in/out */
	{ 49, PINMUX_FUNC_A }, /* B08, gpio_15, i2s_rxd, NA */
	{ 50, PINMUX_FUNC_A }, /* A08, gpio_16, i2s_rscki, NA */
	{ 51, PINMUX_FUNC_A }, /* B09, gpio_17, i2s_rws, NA */
	{ 52, PINMUX_FUNC_A }, /* A09, gpio_18, i2s_tsck, NA */
	{ 53, PINMUX_FUNC_A }, /* C09, gpio_19, i2s_twsi, NA */
	{ 54, PINMUX_FUNC_A }, /* D09, gpio_20, i2s_txd, NA */
	/* Display, flash, BT SPI */
	{ 55, PINMUX_FUNC_B }, /* D08, gpio_21, spi0_m_sck, NA */
	{ 56, PINMUX_FUNC_B }, /* E07, gpio_22, spi0_m_miso, NA */
	{ 57, PINMUX_FUNC_B }, /* E09, gpio_23, spi0_m_mosi, NA */
	{ 58, PINMUX_FUNC_B }, /* E08, gpio_24, spi0_m_cs0, NA */
	{ 59, PINMUX_FUNC_A }, /* A10, gpio_25, spi0_m_cs1, NA */
	{ 60, PINMUX_FUNC_A }, /* B10, gpio_26, spi0_m_cs2, NA */
	{ 61, PINMUX_FUNC_A }, /* C10, gpio_27, spi0_m_cs3, NA */
	{ 62, PINMUX_FUNC_A }, /* D10, gpio_28, NA, NA */
	/* LEDs / pin configurations for PWM */
	{ 63, PINMUX_FUNC_B }, /* E10, gpio_ss_10, pwm_0, NA */
	{ 64, PINMUX_FUNC_B }, /* D11, gpio_ss_11, pwm_1, NA */
	{ 65, PINMUX_FUNC_B }, /* C11, gpio_ss_12, pwm_2, NA */
	{ 66, PINMUX_FUNC_B }, /* B11, gpio_ss_13, pwm_3, NA */
	/* Host configuration for GPIO-1 */
	{ 67, PINMUX_FUNC_A }, /* D12, gpio_ss_14, clkout_32khz, NA */
	{ 68, PINMUX_FUNC_A }, /* C12, gpio_ss_15, clkout_16mhz, NA */
};
