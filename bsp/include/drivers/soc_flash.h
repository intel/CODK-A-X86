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

#ifndef SOC_FLASH_H_
#define SOC_FLASH_H_

#include "drivers/data_type.h"

/**
 * @defgroup soc_flash Quark_SE SOC Flash
 * Quark_SE SOC On-die flash driver API.
 * @ingroup common_drivers
 * @{
 */

/**
 * spi flash memory driver.
 */
extern struct driver soc_flash_driver;

/**
 *  Read memory data on a page
 *
 *  @param  address         : Address (in bytes) to read (need to be 4 bytes aligned)
 *  @param  len             : Number of dword (32bits) to read
 *  @param  retlen          : Number of dword (32bits) successfully red
 *  @param  data            : Buffer to store red data
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC soc_flash_read(uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data);

/**
 *  Write buffer into flash memory
 *
 *  @param  address         : Address (in bytes) to read (need to be 4 bytes aligned)
 *  @param  len             : Number of dword (32bits) to write
 *  @param  retlen          : Number of dword (32bits) successfully written
 *  @param  data            : data to write
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC soc_flash_write(uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data);

/**
 *  Erase blocks of flash or ROM memory
 *
 *  @param  start_block     : First block to erase
 *  @param  block_count     : Number of blocks to erase
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC soc_flash_block_erase(unsigned int start_block, unsigned int block_count);

/** @} */

#endif  /* SOC_FLASH_H_ */
