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

#include "uart.h"
#include "bootlogic.h"
#include "machine.h"
#include "drivers/soc_flash.h"
#include "drivers/data_type.h"
#include "machine/soc/quark_se/quark_se_mapping.h"

#define BOOT_PAGE_START 0
#define BOOT_PAGE_NR 30

void main(void)
{
    unsigned int retlen;
    uint32_t data_new_bl = ARC_START_PAGE;
    int count =0;

    soc_init();
    uart_init(1, COM2_BASE_ADRS, 115200);
    uart_puts("UART app updater\r\n");
    uart_puts("Copying image on Bootloader Partition\r\n");

    soc_flash_block_erase(BOOT_PAGE_START,BOOT_PAGE_NR );

    /* Perform actual copy using soc_flash driver */
    for (count = BOOT_PAGE_START; count < BOOT_PAGE_NR; count++) {
        soc_flash_write((BOOT_PAGE_START + count) * EMBEDDED_FLASH_BLOCK_SIZE,
			EMBEDDED_FLASH_BLOCK_SIZE/4,
                        &retlen,
			(uint32_t*) ((data_new_bl + count) * EMBEDDED_FLASH_BLOCK_SIZE + BASE_FLASH_ADDR));
        uart_puts(".");
    }
    uart_puts("Copy Done. \r\n");
    uart_puts("Rebooting... \r\n");
    reboot(TARGET_FLASHING);
    /* You should never get here... */
    while (1);
}
