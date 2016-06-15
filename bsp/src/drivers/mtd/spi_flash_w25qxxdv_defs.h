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

#ifndef HW_FLASH_DEFS_H
#define HW_FLASH_DEFS_H

/* Status Registers
 *    S7     S6     S5     S4     S3     S2     S1     S0
 *  +-------------------------------------------------------+
 *  | SRP0 | SEC  |  TB  | BP2  | BP1  | BP0  | WEL  | BUSY |
 *  +-------------------------------------------------------+
 *
 * BUSY - Erase/Write In Progress - 1 device is executing a command, 0 ready for command
 * WEL  - Write Enable Latch - 1 write enable is received, 0 completeion of a Write Disable, Page Program, Erase, Write Status Register
 *
 *    S15    S14    S13    S12    S11    S10    S9     S8
 *  +-------------------------------------------------------+
 *  | SUS  | CMP  | LB3  | LB2  | LB1  | xxx  | QE   | SRP1 |
 *  +-------------------------------------------------------+
 *
 *    S23        S22    S21    S20    S19    S18    S17    S16
 *  +----------------------------------------------------------+
 *  | HOLD/RST | DRV1 | DRV0 | xxx  | xxx  | WPS  | xxx  | xxx |
 *  +----------------------------------------------------------+
 */

#define FLASH_RDID_VALUE      (0x001540ef)
// relevant status register bits
#define FLASH_WIP_BIT         (0x01)      // ***YYY Write-In-Progress bit
#define FLASH_WEL_BIT         (0x02)      // ***YYY Write-Enable-Latch bit
#define FLASH_SRWD_BIT        (0x80)      // ***YYY Status-Register, Write-Protect bit
#define FLASH_TB_BIT          (0x8)       // Top Bottom (T/B) bit
#define FLASH_SR_BP_OFFSET    (2)         // Status-Register, Block-Protection bits offset.
                                             // BP0 is at bit 2 in the status register
// relevant configuration register bits

// relevant security register bits
#define FLASH_SECR_WPSEL_BIT  (0x80)       // Security-Register, Write-Protection type bit
#define FLASH_SECR_EFAIL_BIT  (0x40)       // Security-Register, Erase Fail bit
#define FLASH_SECR_PFAIL_BIT  (0x20)       // Security-Register, Program Fail bit

#define FLASH_SIZE            (0x1000000) // memory size in bytes (16777216)
#define FLASH_PAGE_SIZE       (0x100)     // page size in units of bytes (256)
#define FLASH_SECTOR_SIZE     (0x1000)    // sector size in units of bytes (4096)
#define FLASH_BLOCK32K_SIZE   (0x8000)    // block size in units of bytes (32768)
#define FLASH_BLOCK_SIZE      (0x10000)   // block size in units of bytes (65536)

// nominal operation timings (see p.67 w25q16dv of datasheet)
#define FLASH_SECTOR_ERASE_MS       (60) //4KB
#define FLASH_BLOCK_ERASE_MS        (150) // 32KB
#define FLASH_LARGE_BLOCK_ERASE_MS  (180) // 64KB?
#define FLASH_CHIP_ERASE_MS         (3) //
#define FLASH_MAX_ERASE_MS          (2000) // Maximum timeout for an erase operation

//   "*"      means command may be used in future code
//   "***"    means command is currently used in the code.
//   "***YYY" means command is used and is the same for both MXIC and Winbond flash.
//   "***XXX" means command is used but different for MXIC and Winbond flash

//ID comands
#define FLASH_CMD_RDID        0x9F        // ***YYY RDID (Read Identification)
#define FLASH_CMD_RES         0xAB        // RES (Read Electronic ID)
#define FLASH_CMD_REMS        0x90        // REMS (Read Electronic & Device ID)
#define FLASH_CMD_QPIID       0xAF        // QPIID (QPI ID Read)
#define FLASH_CMD_UNID        0x4B        // UNID (Read Unique ID)

//Register comands
#define FLASH_CMD_WRSR        0x01        // ***YYY WRSR (Write Status Register)
#define FLASH_CMD_RDSR        0x05        // ***YYY RDSR (Read Status Register-1)
#define FLASH_CMD_RDSR2       0x35        // ***XXX RDSR2 (Read Status Register-2)
#define FLASH_CMD_WRSCUR      0x2F        // * WRSCUR (Write Security Register)
#define FLASH_CMD_RDSCUR      0x48        // ***XXX RDSCUR (Read Security Register)


//READ comands
#define FLASH_CMD_READ        0x03        // ***YYY READ (1 x I/O)
#define FLASH_CMD_2READ       0xBB        // 2READ (2 x I/O)
#define FLASH_CMD_4READ       0xEB        // 4READ (4 x I/O)
#define FLASH_CMD_FASTREAD    0x0B        // FAST READ (Fast read data).
#define FLASH_CMD_DREAD       0x3B        // DREAD (1In/2 Out fast read)
#define FLASH_CMD_QREAD       0x6B        // QREAD (1In/4 Out fast read)
#define FLASH_CMD_RDSFDP      0x5A        // RDSFDP (Read SFDP)

//Program comands
#define FLASH_CMD_WREN        0x06        // ***YYY WREN (Write Enable)
#define FLASH_CMD_WRDI        0x04        // WRDI (Write Disable)
#define FLASH_CMD_PP          0x02        // ***YYY PP (page program)
#define FLASH_CMD_4PP         0x32        // 4PP (Quad page program)
#define FLASH_CMD_WRENVSR     0x50        // WRENVSR  (WREV WrteEnable Volatile Status Register

//Erase comands
#define FLASH_CMD_SE          0x20        // ***YYY SE (Sector Erase)
#define FLASH_CMD_BE32K       0x52        // BE32K (Block Erase 32kb)
#define FLASH_CMD_BE          0xD8        // ***YYY BE (Block Erase)
#define FLASH_CMD_CE          0x60        // CE (Chip Erase) hex code: 60 or C7

//Mode setting comands
#define FLASH_CMD_DP          0xB9        // ***YYY DP (Deep Power Down)
#define FLASH_CMD_RDP         0xAB        // ***YYY RDP (Release from Deep Power Down)
//#define FLASH_CMD_ENSO        0xB1        // ENSO (Enter Secured OTP)
//#define FLASH_CMD_EXSO        0xC1        // EXSO (Exit Secured OTP)
//#define FLASH_CMD_EQIO        0x35        // EQIO (Enable Quad I/O)
//#define FLASH_CMD_WPSEL       0x68        // WPSEL (Enable block protect mode)

//Reset comands
#define FLASH_CMD_RSTEN       0x66        // RSTEN (Reset Enable)
#define FLASH_CMD_RST         0x99        // RST (Reset Memory)
#define FLASH_CMD_RSTQIO      0xF5        // RSTQIO (Reset Quad I/O)

//Security comands
#define FLASH_CMD_ERSR        0x44        // ERSR (Erase Security Registers)
#define FLASH_CMD_PRSR        0x42        // PRSR (Program Security Registers)

//Suspend/Resume comands
#define FLASH_CMD_PGM_ERS_S   0x75        // PGM/ERS Suspend (Suspends Program/Erase) old: 0xB0
#define FLASH_CMD_PGM_ERS_R   0x7A        // PGM/ERS Erase (Resumes Program/Erase) old: 0x30


#define FLASH_CMD_NOP         0x00        // NOP (No Operation)

#endif
