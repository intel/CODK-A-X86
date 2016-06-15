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

#include "drivers/soc_flash.h"

#include "machine.h"
#include "soc_flash_defs.h"
#ifdef CONFIG_DEVICE_FRAMEWORK
#include "infra/device.h"
#endif

#ifdef CONFIG_WAKELOCK
#include "infra/wakelock_ids.h"

#define SOC_FLASH_WAKELOCK_TIMEOUT 1000
#endif

typedef enum {
    FLASH_0 = 0,
    FLASH_1,
    TOTAL_FLASH} FLASH_ID;

/*! Flash/ROM memory management structure */
typedef struct flash_info_struct
{
    uint32_t           addr_base;       /*!< base address of memory */
    uint32_t           reg_base;        /*!< base address of device register set */
    uint32_t           clk_freq;        /*!< Device clock frequency */
    bool               use_low_voltage; /*!< Low voltage setting */
    unsigned int       block_count;     /*!< Number of blocks in the memory */
    unsigned int       block_size;      /*!< Block size in bytes */
    unsigned int       mem_size;        /*!< Memory size in byte (It must be equal to block_count*block_size) */
    uint8_t            is_init;         /*!< Init state of memory */
} flash_info_t, *flash_info_pt;

static flash_info_t flash_devs[] = {
        { .is_init = 0,
          .use_low_voltage = 0,
          .reg_base = FLASH0_REG_BASE_ADDR,
          .addr_base = FLASH0_BASE_ADDR,
          .block_count = 96,
          .block_size = 0x800,
          .mem_size = 0x30000},
        { .is_init = 0,
          .use_low_voltage = 0,
          .reg_base = FLASH1_REG_BASE_ADDR,
          .addr_base = FLASH1_BASE_ADDR,
          .block_count = 96,
          .block_size = 0x800,
          .mem_size = 0x30000}};

#ifdef CONFIG_WAKELOCK
static struct pm_wakelock soc_flash_wakelock;
#endif

static DRIVER_API_RC soc_flash_validate_protection(flash_info_pt dev)
{
    if(MMIO_REG_VAL_FROM_BASE(dev->reg_base, CTRL) & CTRL_FL_WR_DIS) {
        return DRV_RC_WRITE_PROTECTED;
    }
    return DRV_RC_OK;
}

static int soc_flash_validate_block(unsigned int start_block, unsigned int block_count, unsigned int* block_offset, bool check_protection)
{
    unsigned int i, idx;
    uint32_t size = 0;
    DRIVER_API_RC ret;

    uint32_t phy_start, phy_last;
    uint32_t pc_addr = (uint32_t)__builtin_return_address(0);

    // Iterate flash memories to find the first we need to write/read on
    for(i=0; i<TOTAL_FLASH; i++) {
        if(size+(flash_devs[i].block_count) > start_block) {
            // Compute offset from flash device addr_base to the area to read/write
            *block_offset = start_block-size;
            break;
        }
        size += flash_devs[i].block_count;
    }
    // Address not found in memory range
    if(i==TOTAL_FLASH) {
        return -DRV_RC_OUT_OF_MEM;
    }
    idx = i;
    phy_start = flash_devs[i].addr_base+*block_offset*flash_devs[i].block_size;

    // Check if there is enough space in flash devices to read/write data
    for(; i<TOTAL_FLASH; i++) {
        if (idx != i)
            phy_start = flash_devs[i].addr_base;
        phy_last = size-start_block;
        size += flash_devs[i].block_count;
        // If check protection is not needed, just check memory size
        if(!check_protection) {
            if(size >= start_block+block_count) {
                break;
            }
        } else {
            // Check write protection for current flash device
            if((ret = soc_flash_validate_protection(&flash_devs[i])) != DRV_RC_OK) {
                return -DRV_RC_WRITE_PROTECTED;
            }
            // Check PC frame location
            if(size >= start_block+block_count) {
                phy_last = flash_devs[i].addr_base+(block_count-phy_last)*flash_devs[i].block_size;
                if((pc_addr >= phy_start) && (pc_addr < phy_last)) {
                    return -DRV_RC_ERASE_PC;
                }
                break;
            } else {
                phy_last = flash_devs[i].addr_base+flash_devs[i].mem_size;
                if((pc_addr >= phy_start) && (pc_addr < phy_last)) {
                    return -DRV_RC_ERASE_PC;
                }
            }
        }
    }
    // Address not found in memory range
    if(i==TOTAL_FLASH) {
        return -DRV_RC_OUT_OF_MEM;
    }

    return idx;
}

static int soc_flash_validate_addr(uint32_t address, unsigned int len, uint32_t* offset, bool check_protection)
{
    unsigned int i, idx;
    uint32_t size = 0;
    DRIVER_API_RC ret;

    uint32_t phy_start, phy_last;
    uint32_t pc_addr = (uint32_t)__builtin_return_address(0);

    // Iterate flash memory to find the first we need to write/read on
    for(i=0; i<TOTAL_FLASH; i++) {
        if(size+(flash_devs[i].mem_size) > address) {
            // Compute offset from flash device addr_base to the area to read/write
            *offset = (address-size) & ~(3);
            break;
        }
        size += flash_devs[i].mem_size;
    }
    // Address not found in memory range
    if(i==TOTAL_FLASH) {
        return -DRV_RC_OUT_OF_MEM;
    }
    idx = i;
    phy_start = flash_devs[i].addr_base+*offset;

    // len is for dword buffer. We need to convert it to 8bits
    len = (len << 2);

    // Check if there is enough space in flash devices to read/write data
    for(; i<TOTAL_FLASH; i++) {
        if (idx != i)
            phy_start = flash_devs[i].addr_base;
        phy_last = size-address;
        size += flash_devs[i].mem_size;
        // If check protection is not needed, just check memory size
        if(!check_protection) {
            if(size >= address+len) {
                break;
            }
        } else {
            // Check write protection for current flash device
            if((ret = soc_flash_validate_protection(&flash_devs[i])) != DRV_RC_OK) {
                return -DRV_RC_WRITE_PROTECTED;
            }
            // Check PC frame location
            if(size >= address+len) {
                phy_last = flash_devs[i].addr_base+len-phy_last;
                if((pc_addr >= phy_start) && (pc_addr < phy_last)) {
                    return -DRV_RC_ERASE_PC;
                }
                break;
            } else {
                phy_last = flash_devs[i].addr_base+flash_devs[i].mem_size;
                if((pc_addr >= phy_start) && (pc_addr < phy_last)) {
                    return -DRV_RC_ERASE_PC;
                }
            }
        }
    }
    // Address not found in memory range
    if(i==TOTAL_FLASH) {
        return -DRV_RC_OUT_OF_MEM;
    }

    return idx;
}

DRIVER_API_RC soc_flash_read(uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data)
{
    unsigned int i;
    uint32_t offset = 0;

    int index;
    DRIVER_API_RC ret = DRV_RC_OK;

#ifdef CONFIG_WAKELOCK
    pm_wakelock_acquire(&soc_flash_wakelock, SOC_FLASH_WAKELOCK_TIMEOUT);
#endif
    *retlen = 0;

    if(len == 0) {
        ret = DRV_RC_INVALID_OPERATION;
        goto exit;
    }

    if((index = soc_flash_validate_addr(address, len, &offset, false)) < 0) {
        ret = -index;
        goto exit;
    }

    for(i=0; i<len; i++) {
        if(offset >= flash_devs[index].mem_size) {
            index++;
            offset = 0;
        }
        data[i] = MMIO_REG_VAL_FROM_BASE(flash_devs[index].addr_base, offset);
        offset+=4;
    }

    *retlen = len;

exit:
#ifdef CONFIG_WAKELOCK
    pm_wakelock_release(&soc_flash_wakelock);
#endif
    return ret;
}

DRIVER_API_RC soc_flash_write(uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data)
{
    unsigned int i;
    uint32_t offset, read_offset = 0;
    int index, read_index;
    DRIVER_API_RC ret = DRV_RC_OK;

#ifdef CONFIG_WAKELOCK
    pm_wakelock_acquire(&soc_flash_wakelock, SOC_FLASH_WAKELOCK_TIMEOUT);
#endif

    *retlen = 0;

    if(len == 0) {
        ret = DRV_RC_INVALID_OPERATION;
        goto exit;
    }

    if((index = soc_flash_validate_addr(address, len, &offset, true)) < 0) {
        ret = -index;
        goto exit;
    }

    // Init write verification index and offset
    read_index = index;
    read_offset = offset;

    // Start write operation
    MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_WR_DATA) = data[0];
    MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_WR_CTRL) = ((offset)<<FLASH_WR_CTRL_WR_ADDR_BIT_OFFSET | FLASH_WR_CTRL_WR_REQ);
    // Compute next address
    offset += 4;
    // Check if we reach the end of memory
    if(offset >= flash_devs[index].mem_size) {
        index++;
        offset = 0;
    }
    // Address not found in memory range
    if(index == TOTAL_FLASH) {
        ret = DRV_RC_OUT_OF_MEM;
        goto exit;
    }
    // Wait until write operation is complete
    while( (MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_STTS) & FLASH_STTS_WR_DONE) == 0);

    for(i=1; i<len; i++) {
        // Start next write operation
        MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_WR_DATA) = data[i];
        MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_WR_CTRL) = ((offset)<<FLASH_WR_CTRL_WR_ADDR_BIT_OFFSET | FLASH_WR_CTRL_WR_REQ);

        // Because write operation takes several cpu cycles to complete,
        // we verify previous written dword after the operation is started

        // Flush the pre-fetch buffer
        SET_MMIO_BIT((volatile uint32_t*)(flash_devs[index].reg_base+CTRL), CTRL_PRE_FLUSH_BIT);
        CLEAR_MMIO_BIT((volatile uint32_t*)(flash_devs[index].reg_base+CTRL), CTRL_PRE_FLUSH_BIT);

        // Validate previous write operation
        if(__builtin_expect((MMIO_REG_VAL_FROM_BASE(flash_devs[read_index].addr_base, read_offset) != data[i-1]), 0)) {
            // Write error (bad block detected ?)
            // Wait until write operation is complete
            while( (MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_STTS) & FLASH_STTS_WR_DONE) == 0);
            *retlen = i-1;
            ret = DRV_RC_CHECK_FAIL;
            goto exit;
        }
        // Compute write address for next operation
        offset += 4;
        read_offset += 4;

        // Check if we reach the end of memory
        if(__builtin_expect((read_offset >= flash_devs[read_index].mem_size), 0)) {
            read_index++;
            read_offset = 0;
        }
        // Check if we reach the end of memory
        if(__builtin_expect((offset >= flash_devs[index].mem_size), 0)) {
            index++;
            offset = 0;
        }

        // Wait for current write operation to complete
        while( (MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_STTS) & FLASH_STTS_WR_DONE) == 0);
    }
    // Flush the pre-fetch buffer
    SET_MMIO_BIT((volatile uint32_t*)(flash_devs[index].reg_base+CTRL), CTRL_PRE_FLUSH_BIT);
    CLEAR_MMIO_BIT((volatile uint32_t*)(flash_devs[index].reg_base+CTRL), CTRL_PRE_FLUSH_BIT);

    // Validate previous write operation
    if(__builtin_expect((MMIO_REG_VAL_FROM_BASE(flash_devs[read_index].addr_base, read_offset) != data[len-1]), 0)) {
        // Write error (bad block detected ?)
        *retlen = len-1;
        ret = DRV_RC_CHECK_FAIL;
        goto exit;
    }

    *retlen = len;

exit:
#ifdef CONFIG_WAKELOCK
    pm_wakelock_release(&soc_flash_wakelock);
#endif
    return ret;
}

DRIVER_API_RC soc_flash_block_erase(unsigned int start_block, unsigned int block_count)
{
    unsigned int i, blk_offset = 0;
    int index;
    DRIVER_API_RC ret = DRV_RC_OK;

#ifdef CONFIG_WAKELOCK
    pm_wakelock_acquire(&soc_flash_wakelock, SOC_FLASH_WAKELOCK_TIMEOUT);
#endif

    if(block_count == 0) {
        ret = DRV_RC_INVALID_OPERATION;
        goto exit;
    }

    if((index = soc_flash_validate_block(start_block, block_count, &blk_offset, true)) < 0) {
        ret = -index;
        goto exit;
    }

    for(i=0; i<block_count; i++) {
        if(blk_offset >= flash_devs[index].block_count) {
            index++;
            blk_offset = 0;
        }
        MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_WR_CTRL) =
            (((blk_offset*(flash_devs[index].block_size))<<FLASH_WR_CTRL_WR_ADDR_BIT_OFFSET) | FLASH_WR_CTRL_ER_REQ);
        /* Wait until erase operation is complete.*/
        while( (MMIO_REG_VAL_FROM_BASE(flash_devs[index].reg_base, FLASH_STTS) & FLASH_STTS_ER_DONE) == 0);

        blk_offset++;
    }

exit:
#ifdef CONFIG_WAKELOCK
    pm_wakelock_release(&soc_flash_wakelock);
#endif
    return ret;
}

#ifdef CONFIG_DEVICE_FRAMEWORK
static int pm_soc_flash_init(struct device* dev){
#ifdef CONFIG_WAKELOCK
    pm_wakelock_init(&soc_flash_wakelock, SOC_FLASH_WAKELOCK);
#endif
    return 0;
}

struct driver soc_flash_driver = {
    .init = pm_soc_flash_init,
    .suspend = NULL,
    .resume = NULL
};
#endif
