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

#ifndef _WAKELOCK_IDS_H_
#define _WAKELOCK_IDS_H_

#define SBA_WAKELOCK_BASE   0x10 /*!< sba wakelock from 0x10 to 0x1F */
#define SOC_FLASH_WAKELOCK  0x20 /*!< ondie soc flash wakelock */
#define SPI_FLASH_WAKELOCK  0x30 /*!< spi flash wakelock */
#define ADC_WAKELOCK        0x40 /*!< adc wakelock */
#define HAPTIC_WAKELOCK     0x50 /*!< haptic wakelock */
#define LED_WAKELOCK        0x60 /*!< led wakelock */
#define RTC_WAKELOCK        0x70 /*!< rtc wakelock */
#define PSH_CORE_WAKELOCK   0x80 /*!< psh core wakelock */
#define IPC_UARTRX_WAKELOCK 0x90 /*!< ipc uart wakelock for RX data */
#define IPC_UARTTX_WAKELOCK 0xa0 /*!< ipc uart wakelock for TX data */
#define OPENCORE_WAKELOCK   0xb0 /*!< open core wakelock for pool sensor data */
#define NFC_WAKELOCK        0xc0 /*!< nfc devices wakelock base */

#endif /* _WAKELOCK_IDS_H_ */
