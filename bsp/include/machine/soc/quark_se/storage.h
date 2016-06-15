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

#ifndef __STORAGE_H__
#define __STORAGE_H__

typedef enum flash_location
{
	EMBEDDED_FLASH,     /* on-die flashs */
	SERIAL_FLASH       /* serial nor */
} flash_location_t;

typedef struct flash_partition {
	uint16_t partition_id;  /* logical id of the partition in the full system*/
	uint16_t flash_id;      /* id of the flash */
	uint16_t start_block;   /* within the flash */
	uint16_t end_block;     /* must fit within the same flash*/
	uint8_t factory_reset_state; /* partition data status during factory erase */
} flash_partition_t;

typedef struct flash_device {
	uint16_t flash_id;      /* id of the flash */
	uint16_t nb_blocks;
	uint16_t block_size;    /* size of the erase block we should consider */
	flash_location_t flash_location;  /* on-die flash, external flash */
} flash_device_t;

#endif /* __STORAGE_H__ */

