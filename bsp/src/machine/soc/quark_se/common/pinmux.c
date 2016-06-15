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

#include "pinmux.h"
#include "scss_registers.h"

static uint8_t tab_pin_mux[][2]={
	/* ADC channels muxing */
#if defined CONFIG_BOARD_ARDUINO101 \
	|| defined CONFIG_BOARD_QUARK_SE_CTB
	{0, QRK_PMUX_SEL_MODEB},	/* AIN[0]           */
	{1, QRK_PMUX_SEL_MODEB},	/* AIN[1]           */
	{2, QRK_PMUX_SEL_MODEB},	/* AIN[2]           */
#elif defined CONFIG_BOARD_QUARK_SE_APP \
	|| defined CONFIG_BOARD_QUARK_SE_CRB \
	|| defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	{0, QRK_PMUX_SEL_MODEA},	/* GPIO[0]/AIN[0]   */
	{1, QRK_PMUX_SEL_MODEA},	/* GPIO[1]/AIN[1]   */
	{2, QRK_PMUX_SEL_MODEA},	/* GPIO[2]/AIN[2]   */
#endif
#if defined CONFIG_BOARD_QUARK_SE_CRB \
	|| defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	{3, QRK_PMUX_SEL_MODEA},	/* GPIO[3]/AIN[3]   */
#else
	{3, QRK_PMUX_SEL_MODEB},	/* GPIO[3]/AIN[3]   */
#endif
	{4, QRK_PMUX_SEL_MODEB},	/* AIN[4]           */
	{5, QRK_PMUX_SEL_MODEB},	/* AIN[5]           */
#if defined CONFIG_BOARD_QUARK_SE_APP
	{6, QRK_PMUX_SEL_MODEB},	/* AIN[6]           */
#elif defined CONFIG_QUARK_SE_CURIE
	/* Nordic SWD debug pins */
	{6, QRK_PMUX_SEL_MODEA},	/* GPIO[6]/AIN[6]        BLE_SWDIO  */
#endif
	/* usb_pm pin muxing */
#if defined CONFIG_QUARK_SE_CURIE
	{7, QRK_PMUX_SEL_MODEB},	/* AIN[7]           */
#endif
	/*Setup UART 1 PIN muxing */
	{8, QRK_PMUX_SEL_MODEC},	/* UART1_CTS_B      */
	{9, QRK_PMUX_SEL_MODEC},	/* UART1_RTS_B      */

#if defined CONFIG_BOARD_QUARK_SE_CRB \
	|| defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	{10, QRK_PMUX_SEL_MODEB},	/* GPIO_SS[2]/AIN[10] */
#else
	{10, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[2]/AIN[10] */
#endif

#if defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	{11, QRK_PMUX_SEL_MODEB},	/* AIN[11] */
	{12, QRK_PMUX_SEL_MODEB},	/* AIN[12] */
	{13, QRK_PMUX_SEL_MODEB},	/* AIN[13] */
#else
	{11, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[3] */
	{12, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[4] */
	{13, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[5] */
#endif
	{14, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[6]/AIN[14] */
	{15, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[7]/AIN[15] */

	{16, QRK_PMUX_SEL_MODEC},	/* GPIO_SS[8]/AIN[16]/UART1_TXD */
	{17, QRK_PMUX_SEL_MODEC},	/* GPIO_SS[9]/AIN[17]/UART1_RXD */
#if defined CONFIG_IPC_UART
	/* Setup UART0 for BLE communication, HW flow control required  */
	{18, QRK_PMUX_SEL_MODEA},	/* UART0_RXD        */
	{19, QRK_PMUX_SEL_MODEA},	/* UART0_TXD        */
#else
	/* Following pins are shared by UART0, AIN and SPI1_SS modules  */
	{18, QRK_PMUX_SEL_MODEB},	/* AIN[18]          */
	{19, QRK_PMUX_SEL_MODEB},	/* AIN[19]          */
#endif

	/* SSS I2C0 Pin Muxing */
	{24, QRK_PMUX_SEL_MODEA},	/* I2C0_SS_SCL      */
	{25, QRK_PMUX_SEL_MODEA},	/* I2C0_SS_SDA      */
	/* SSS I2C1 Pin Muxing */
	{26, QRK_PMUX_SEL_MODEA},	/* I2C1_SS_SCL      */
	{27, QRK_PMUX_SEL_MODEA},	/* I2C1_SS_SDA      */

	/* SSS SPI0 Pin Muxing */
	{28, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_MISO     */
	{29, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_MOSI     */
	{30, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_SCK      */
	{31, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_CS_B[0]  */
	{32, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_CS_B[1]  */
	{33, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_CS_B[2]  */
	{34, QRK_PMUX_SEL_MODEA},	/* SPI0_SS_CS_B[3]  */

	/* SSS SPI1 Pin Muxing */
	{35, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_MISO     */
	{36, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_MOSI     */
	{37, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_SCK      */
	{38, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_CS_B[0]  */
	{39, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_CS_B[1]  */

#if defined CONFIG_IPC_UART
	{5, QRK_PMUX_SEL_MODEA},	/* GPIO[5] is QRK_SE_BLE_INT */
	{40, QRK_PMUX_SEL_MODEB},	/* UART0_CTS_B      */
	{41, QRK_PMUX_SEL_MODEB},	/* UART0_RTS_B      */
#else
	{40, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_CS_B[2]  */
	{41, QRK_PMUX_SEL_MODEA},	/* SPI1_SS_CS_B[3]  */
#endif

#if defined CONFIG_BOARD_QUARK_SE_CRB \
	|| defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	{42, QRK_PMUX_SEL_MODEB},	/* GPIO[8]/SPI1_M_SCK       */
	{43, QRK_PMUX_SEL_MODEB},	/* GPIO[9]/SPI1_M_MISO      */
	{44, QRK_PMUX_SEL_MODEB},	/* GPIO[10]/SPI1_M_MOSI     */
	/* Do not remux PAD 45 as it is already initialized in Bootloader. */
	{46, QRK_PMUX_SEL_MODEB},	/* GPIO[12]/SPI1_M_CS_B[1]  */
	{47, QRK_PMUX_SEL_MODEB},	/* GPIO[13]/SPI1_M_CS_B[2]  */
	{48, QRK_PMUX_SEL_MODEA},	/* GPIO[14]/SPI1_M_CS_B[3]  */
#endif
	/* Pads 49 to 54 are muxed in MODEA by default. */
	{49, QRK_PMUX_SEL_MODEA},	/* GPIO[15]/I2S_RXD  */
	{50, QRK_PMUX_SEL_MODEA},	/* GPIO[16]/I2S_RSCK */
	{51, QRK_PMUX_SEL_MODEA},	/* GPIO[17]/I2S_RWS  */
	{52, QRK_PMUX_SEL_MODEA},	/* GPIO[18]/I2S_TSCK */
	{53, QRK_PMUX_SEL_MODEA},	/* GPIO[19]/I2S_TWS  */
	{54, QRK_PMUX_SEL_MODEA},	/* GPIO[20]/I2S_TXD  */

#if defined CONFIG_BOARD_QUARK_SE_APP \
	|| defined CONFIG_BOARD_ARDUINO101 \
	|| defined CONFIG_BOARD_QUARK_SE_CRB
	{55, QRK_PMUX_SEL_MODEB},	/* SPI0_M_SCK       */
	{56, QRK_PMUX_SEL_MODEB},	/* SPI0_M_MISO      */
	{57, QRK_PMUX_SEL_MODEB},	/* SPI0_M_MOSI      */
	/* SPI0_M CS is controlled as a gpio */
	{58, QRK_PMUX_SEL_MODEA},	/* GPIO[24]/SPI0_M_CS_0 */
	{59, QRK_PMUX_SEL_MODEA},	/* GPIO[25]/SPI0_M_CS_1 */
	{60, QRK_PMUX_SEL_MODEA},	/* GPIO[26]/SPI0_M_CS_2 */
#elif defined CONFIG_BOARD_QUARK_SE_CTB \
	|| defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	/* No SPIM0 by default for CTB & Morpheus */
	{55, QRK_PMUX_SEL_MODEA},	/* GPIO[21]         */
	{56, QRK_PMUX_SEL_MODEA},	/* GPIO[22]         */
	{57, QRK_PMUX_SEL_MODEA},	/* GPIO[23]         */
	{58, QRK_PMUX_SEL_MODEA},	/* GPIO[24]         */
	{59, QRK_PMUX_SEL_MODEA},	/* GPIO[25]         */
	{60, QRK_PMUX_SEL_MODEA},	/* GPIO[26]         */
#endif

#if defined CONFIG_QUARK_SE_CURIE
	{61, QRK_PMUX_SEL_MODEA},	/* GPIO[27]/SPI0_M_CS[3] BLE_SW_CLK */
#endif
#if defined CONFIG_BOARD_QUARK_SE_CRB \
	|| defined CONFIG_BOARD_QUARK_SE_MORPHEUS
	{63, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[10]/PWM[0] */
	{64, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[11]/PWM[1] */
	{65, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[12]/PWM[2] */
	{66, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[13]/PWM[3] */
	{67, QRK_PMUX_SEL_MODEA},	/* GPIO_SS[14]/PLT_CLK[0] */
#else
	/* Set QRK drivers pinmux */
	//LED
	{63, QRK_PMUX_SEL_MODEB},	/* GPIO_SS[10]/PWM[0] */
	{64, QRK_PMUX_SEL_MODEB},	/* GPIO_SS[11]/PWM[1] */
	{65, QRK_PMUX_SEL_MODEB},	/* GPIO_SS[12]/PWM[2] */
	{66, QRK_PMUX_SEL_MODEB},	/* GPIO_SS[13]/PWM[3] */
	/* Warning: sss gpio1 bits 2 to 5 are used by the PWM module */
	{67, QRK_PMUX_SEL_MODEA},	/* sss gpio1 bit 6  */
	{68, QRK_PMUX_SEL_MODEA},	/* sss gpio1 bit 7  */
#endif

};

void pinmux_config(void)
{
	uint32_t i = 0;
	for(i = 0; i < (sizeof(tab_pin_mux) / sizeof(tab_pin_mux[0])); i++)
	{
		SET_PIN_MODE(tab_pin_mux[i][0], tab_pin_mux[i][1]);
	}
}
