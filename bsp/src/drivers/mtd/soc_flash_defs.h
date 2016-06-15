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

#ifndef SOC_FLASH_DEFS_H
#define SOC_FLASH_DEFS_H

// Definition of the flash base addresses
#define FLASH0_BASE_ADDR                 (0x40000000)    /* Flash 0 base address */
#define FLASH1_BASE_ADDR                 (0x40030000)    /* Flash 1 base address */
#define ROM_BASE_ADDR                    (0xFFFFE000)    /* ROM base address */

// Definition of the flash register base
#define FLASH0_REG_BASE_ADDR                 (0xB0100000)    /* Flash 0 reg base address */
#define FLASH1_REG_BASE_ADDR                 (0xB0200000)    /* Flash 1 reg base address */
#define ROM_REG_BASE_ADDR                    (0xB0100000)    /* ROM reg base address */

// Flash controller register offsets
#define TMG_CTRL        0x00
#define ROM_WR_CTRL     0x04
#define ROM_WR_DATA     0x08
#define FLASH_WR_CTRL   0x0C
#define FLASH_WR_DATA   0x10
#define FLASH_STTS      0x14
#define CTRL            0x18
#define FPR0_RD_CFG     0x1C
#define FPR1_RD_CFG     0x20
#define FPR2_RD_CFG     0x24
#define FPR3_RD_CFG     0x28
#define MPR_WR_CFG      0x2C
#define MPR_VSTS        0x30

// CTRL register
#define CTRL_FL_WR_DIS       0x00000010 /*!< When set, disable any flash write or erase of both the main flash and ROM */
#define CTRL_MASS_ERASE_INFO 0x00000040 /*!< Mass erase info bit, set high to trigger the OTP flash mass erase operation. */
#define CTRL_MASS_ERASE      0x00000080 /*!< Mass erase request bit, set high to trigger a flash mass erase operation. */
#define CTRL_ROM_RD_DIS_U    0x00000008 /*!< Rom read disable for upper 4k region of ROM */
#define CTRL_ROM_RD_DIS_L    0x00000004 /*!< Rom read disable for lower 4k region of ROM */
#define CTRL_PRE_EN_BIT               0 /*!< Prefetch buffer enable bit. */
#define CTRL_PRE_FLUSH_BIT            1 /*!< Prefetch buffer flush bit to be toggled to clear data in buffer. */

/* FLASH_WR_CTRL register */
#define FLASH_WR_CTRL_WR_REQ              0x00000001 /*!< Write request bit, set high to trigger a flash write. */
#define FLASH_WR_CTRL_ER_REQ              0x00000002 /*!< Erase request bit, set high to trigger a flash page erase. */
#define FLASH_WR_CTRL_WR_ADDR_BIT_OFFSET           2 /*!< Bit offset for the write address field of the write address register. */

/* ROM_WR_CTRL register */
#define ROM_WR_CTRL_WR_REQ                0x00000001 /*!< Write request bit, set high to trigger a flash write. */
#define ROM_WR_CTRL_ER_REQ                0x00000002 /*!< Erase request bit, set high to trigger a flash page erase. */
#define ROM_WR_CTRL_WR_ADDR_BIT_OFFSET             2 /*!< Bit offset for the write address field of the write address register. */

/* FLASH_STTS register */
#define FLASH_STTS_ER_DONE  0x00000001 /*!< When set, indicates that the page erase is completed. */
#define FLASH_STTS_WR_DONE  0x00000002 /*!< When set, indicates that the write operation is completed */
#define FLASH_STTS_ROM_PROG 0x00000004 /*!< When set, indicates that the ROM has been programmed and any further attempt to write ROM is blocked. */

#endif
