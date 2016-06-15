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

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

#include "drivers/data_type.h"
#include "infra/device.h"

/**
 * @defgroup flash_spi_driver External SPI Flash
 * External SPI Flash driver API.
 * @ingroup common_drivers
 * @{
 */

/**
 * spi flash memory driver.
 */
extern struct driver spi_flash_driver;

/**
 * SPI Flash IOCTL commands
 */
#define STORAGE_SIZE			(0x1)
#define STORAGE_PAGE_SIZE		(0x2)
#define STORAGE_SECTOR_SIZE		(0x3)
#define STORAGE_BLOCK_SIZE		(0x4)
#define STORAGE_LARGE_BLOCK_SIZE	(0x5)

/**
 *  Read dwords data on SPI flash
 *
 *  @param  dev             : spi flash device to use
 *  @param  address         : Address (in bytes) to read (need to be 4 bytes aligned)
 *  @param  len             : Number of dword (32bits) to read
 *  @param  retlen          : Number of dword (32bits) successfully red
 *  @param  data            : Buffer to store red data
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_read(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data);

/**
 *  Read dwords data on SPI flash
 *
 *  @param  dev             : spi flash device to use
 *  @param  address         : Address (in bytes) to read
 *  @param  len             : Number of bytes to read
 *  @param  retlen          : Number of bytes successfully red
 *  @param  data            : Buffer to store red data
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_read_byte(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint8_t *data);

/**
 *  Write buffer into flash memory (1024 bytes maximum).
 *
 *  @param  dev             : spi flash device to use
 *  @param  address         : Address (in bytes) to write on (need to be 4 bytes aligned)
 *  @param  len             : Number of dword (32 bits) to write
 *  @param  retlen          : Number of dword (32 bits) successfully written
 *  @param  data            : data to write
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_write(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data);

/**
 *  Write buffer into flash memory (1024 bytes maximum).
 *
 *  @param  dev             : spi flash device to use
 *  @param  address         : Address (in bytes) to write on
 *  @param  len             : Number of bytes to write
 *  @param  retlen          : Number of bytes successfully written
 *  @param  data            : data to write
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_write_byte(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint8_t *data);

/**
 *  Erase sectors (4kB) of spi flash memory
 *
 *  @param  dev             : spi flash device to use
 *  @param  start_sector    : First sector to erase
 *  @param  sector_count    : Number of sectors to erase
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_sector_erase(struct device *dev, unsigned int start_sector, unsigned int sector_count);

/**
 *  Erase blocks (32kB) of spi flash memory
 *
 *  @param  dev             : spi flash device to use
 *  @param  start_block     : First block to erase
 *  @param  block_count     : Number of blocks to erase
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_block_erase(struct device *dev, unsigned int start_block, unsigned int block_count);

/**
 *  Erase large blocks (64kB) of spi flash memory
 *
 *  @param  dev             : spi flash device to use
 *  @param  start_block     : First block to erase
 *  @param  block_count     : Number of blocks to erase
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_large_block_erase(struct device *dev, unsigned int start_block, unsigned int block_count);

/**
  *  Get the current status of the SPI Flash disk
  *
  *  @param  dev             : spi flash device to use
  *  @param  status          : Return value of the internal state of the flash
  *
  *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
  */
DRIVER_API_RC spi_flash_get_status(struct device *dev, uint8_t *status);

/**
 *  Erase all SPI flash memory
 *
 *  @param  dev             : spi flash device to use
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_large_chip_erase(struct device *dev);

/**
 *  Get SPI flash memory RDID
 *
 *  @param  dev      : spi flash device to use
 *  @param  rdid     : pointer to the output value for rdid
 *
 *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
 */
DRIVER_API_RC spi_flash_get_rdid(struct device *dev, uint32_t *rdid);

/**
  *  Get SPI flash memory details via IOCTL
  *
  *  @param  dev      : spi flash device to use
  *  @param  result   : pointer to the output value ioctl call
  *  @param  ioctl    : requested IOCTL operation
  *
  *  @return  DRV_RC_OK on success else DRIVER_API_RC error code
  */
DRIVER_API_RC spi_flash_ioctl(struct device *dev, uint32_t *result,
			      uint8_t ioctl);

/** @} */

#endif  /* SPI_FLASH_H_ */
