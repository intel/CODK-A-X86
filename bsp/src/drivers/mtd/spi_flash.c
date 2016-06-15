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

#include "drivers/spi_flash.h"

#include <string.h>    // For memcpy

#include "os/os.h"     // For balloc
#include "infra/log.h" // For logger

#include "drivers/serial_bus_access.h"

#ifdef CONFIG_SPI_FLASH_W25Q16DV
#include "spi_flash_w25qxxdv_defs.h"
#elif defined(CONFIG_SPI_FLASH_MX25U12835F)
#include "spi_flash_defs.h"
#else
#error Please define the device for which you build the driver
#endif

#include "infra/wakelock_ids.h"

#define SPI_FLASH_WAKELOCK_TIMEOUT 1000

#define SBA_TIMEOUT    1000
#define DEVICE_MUTEX_DELAY OS_NO_WAIT

/*! Flash memory management structure */
typedef struct spi_flash_info {
    uint32_t           sector_count;                 /*!< Number of sectors in the memory */
    uint32_t           sector_size;                  /*!< Sector size in bytes */
    uint32_t           block_count;                  /*!< Number of blocks in the memory */
    uint32_t           block_size;                   /*!< Block size in bytes */
    uint32_t           large_block_count;            /*!< Number of large blocks in the memory */
    uint32_t           large_block_size;             /*!< Large block size in bytes */
    uint32_t           mem_size;                     /*!< Memory size in byte (It must be equal to block_count*block_size) */
    uint8_t            is_init;                      /*!< Init state of memory */

    struct sba_request req;                          /*!< sba request object used to drive the spi flash */
    T_TIMER            spi_timer;                    /*!< Timer to wait for erase/program operations to complete */
    T_SEMAPHORE        spi_timer_sem;                /*!< Semaphore to wait for spi_timer event */
    T_SEMAPHORE        spi_sync_sem;                 /*!< Semaphore to wait for and spi transfer to complete */
    uint8_t            tx_buffer[FLASH_PAGE_SIZE+4]; /*!< Buffer used to store tx data during write operation */
    T_MUTEX            device_mtx;                   /*!< Device in use mutex */
    struct pm_wakelock wakelock;                     /*!< wakelock */
} spi_flash_info_t, *spi_flash_info_pt;

/*! Erase operations list */
typedef enum {
        ER_SECTOR,      /*!< Erase sector */
        ER_BLOCK,       /*!< Erase block */
        ER_LARGE_BLOCK, /*!< Erase lage block */
        ER_CHIP         /*!< Erase all flash chip */
} ER_TYPE;

// Internal device driver functions
static DRIVER_API_RC spi_flash_get_rdscur(struct device *dev, uint8_t *rdscur);
static DRIVER_API_RC spi_flash_write_enable(struct device *dev);
static DRIVER_API_RC spi_flash_sleep(struct device *dev, bool on);
static DRIVER_API_RC spi_flash_erase(struct device *dev, ER_TYPE er_type, unsigned int start, unsigned int count);
static DRIVER_API_RC spi_sync(struct device *dev, struct sba_request *req);

// User API functions
DRIVER_API_RC spi_flash_get_status(struct device *dev, uint8_t *status);
DRIVER_API_RC spi_flash_read_byte(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint8_t *data);
DRIVER_API_RC spi_flash_write_byte(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint8_t *data);
DRIVER_API_RC spi_flash_sector_erase(struct device *dev, unsigned int start_sector, unsigned int sector_count);
DRIVER_API_RC spi_flash_block_erase(struct device *dev, unsigned int start_block, unsigned int block_count);
DRIVER_API_RC spi_flash_large_block_erase(struct device *dev, unsigned int start_block, unsigned int block_count);
DRIVER_API_RC spi_flash_chip_erase(struct device *dev);
DRIVER_API_RC spi_flash_ioctl(struct device *dev, uint32_t *result,
			      uint8_t ioctl);

DRIVER_API_RC spi_flash_get_rdid(struct device *dev, uint32_t *rdid);

// Device driver callback functions
static void spi_timer_sync_callback(void* priv);
static void spi_completion_callback(struct sba_request *req);

static int spi_flash_init(struct device *device)
{
    uint32_t rdid;
    DRIVER_API_RC ret = DRV_RC_FAIL;
    struct spi_flash_info *flash_dev;
    struct sba_device     *dev = (struct sba_device*)device;

    // Alloc device priv data
    if ((flash_dev = (struct spi_flash_info*)balloc(sizeof(struct spi_flash_info), NULL)) == NULL) {
        // TODO: log
        pr_error(LOG_MODULE_DRV, "failed to alloc priv data for device %d", device->id);
        return -1;
    }

    pm_wakelock_init(&flash_dev->wakelock, SPI_FLASH_WAKELOCK);

    // Compute block and sector counts
    flash_dev->sector_size =      FLASH_SECTOR_SIZE,
    flash_dev->block_size =       FLASH_BLOCK32K_SIZE,
    flash_dev->large_block_size = FLASH_BLOCK_SIZE,
    flash_dev->mem_size = FLASH_SIZE,
    flash_dev->sector_count = flash_dev->mem_size/flash_dev->sector_size;
    flash_dev->block_count = flash_dev->mem_size/flash_dev->block_size;
    flash_dev->large_block_count = flash_dev->mem_size/flash_dev->large_block_size;

    // Create mutex for device multiple access protection
    if ((flash_dev->device_mtx = mutex_create(NULL)) == NULL) {
        goto exit_dev_mutex;
    }
    // Create a taken semaphore for non blocking waits
    if ((flash_dev->spi_timer_sem = semaphore_create(0, NULL)) == NULL) {
        goto exit_sem;
    }
    // Create a taken semaphore for spi sync transfers
    if ((flash_dev->spi_sync_sem = semaphore_create(0, NULL)) == NULL) {
        goto exit_spi_sem;
    }
    // Create timer for non blocking waits (synchronous operating mode)
    if ((flash_dev->spi_timer = timer_create(spi_timer_sync_callback, flash_dev, FLASH_BLOCK_ERASE_MS, false, false, NULL)) == NULL) {
        goto exit_timer;
    }
    // Init sba_request struct
    flash_dev->req.request_type = SBA_TRANSFER;
    flash_dev->req.addr.cs = dev->addr.cs;

    // Link driver priv data to device
    device->priv = flash_dev;

    // Init ok
    flash_dev->is_init = 1;
    // Check flash RDID (probe device)
    if (((ret = spi_flash_get_rdid(device, &rdid)) != DRV_RC_OK) || (rdid != FLASH_RDID_VALUE)) {
        pr_error(LOG_MODULE_DRV, "rdid check failed (0x%x)", rdid);
        flash_dev->is_init = 0;
        goto exit_rdid;
    }
    pr_debug(LOG_MODULE_DRV, "init ok %d, master is %d", device->id, device->parent->id);
    return DRV_RC_OK;

exit_rdid:
    timer_delete(flash_dev->spi_timer, NULL);
exit_timer:
    semaphore_delete(flash_dev->spi_sync_sem, NULL);
exit_spi_sem:
    semaphore_delete(flash_dev->spi_timer_sem, NULL);
exit_sem:
    mutex_delete(flash_dev->device_mtx, NULL);
exit_dev_mutex:
    bfree(flash_dev);
    return -!!ret; // -1 if error else 0
}

static int spi_flash_suspend(struct device *dev, PM_POWERSTATE state)
{
    pr_debug(LOG_MODULE_DRV, "spi flash suspend device %d (%d)", dev->id, state);
    return 0;
}

static int spi_flash_resume(struct device *dev)
{
    pr_debug(LOG_MODULE_DRV, "spi flash resume device %d", dev->id);
    return 0;
}

struct driver spi_flash_driver = {.init = spi_flash_init,
                                  .suspend = spi_flash_suspend,
                                  .resume = spi_flash_resume};

// ***************************************
// * spi flash driver internal functions *
// ***************************************

static void spi_timer_sync_callback(void* priv)
{
    // Unlock spi timer mutex to notify that the wait is complete
    semaphore_give(((struct spi_flash_info*)priv)->spi_timer_sem, NULL);
}

static void spi_completion_callback(struct sba_request *req)
{
    // Give spi timer semaphore to notify that the spi transfer is complete
    semaphore_give((T_SEMAPHORE)(req->priv_data), NULL);
}

static DRIVER_API_RC spi_sync(struct device *dev, struct sba_request *req)
{
    DRIVER_API_RC ret;
    OS_ERR_TYPE ret_os;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;

    req->priv_data = flash_dev->spi_sync_sem;
    req->callback = spi_completion_callback;

    if((ret = sba_exec_dev_request((struct sba_device*)dev, req)) == DRV_RC_OK) {
        // Wait for transfer to complete (timeout = 100ms)
        if ((ret_os = semaphore_take(flash_dev->spi_sync_sem, SBA_TIMEOUT)) != E_OS_OK) {
            if (ret_os == E_OS_ERR_BUSY) {
                ret = DRV_RC_TIMEOUT;
            } else {
                ret = DRV_RC_FAIL;
            }
        } else {
            ret = req->status;
        }
    }

    return ret;
}

static DRIVER_API_RC spi_flash_sleep(struct device *dev, bool on)
{
    uint8_t command;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;
    flash_dev->req.tx_len         = 1;
    flash_dev->req.tx_buff        = &command;
    flash_dev->req.rx_len         = 0;
    flash_dev->req.rx_buff        = NULL;

    if (on == true) {
        //wakeup to standby mode
        command = FLASH_CMD_RDP;
        return spi_sync(dev, &flash_dev->req);
    }
#ifdef LOW_POWER_MODE
    //enter deep power down mode
    command = FLASH_CMD_DP;
    return spi_sync(dev, &flash_dev->req);
#else
    return DRV_RC_OK;
#endif
}

DRIVER_API_RC spi_flash_get_rdid(struct device *dev, uint32_t *rdid)
{
    uint8_t command;
    DRIVER_API_RC ret;
    OS_ERR_TYPE ret_os;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;

    if (!flash_dev->is_init) {
        return DRV_RC_INVALID_OPERATION;
    }
    // Take spi device mutex
    if ((ret_os = mutex_lock(flash_dev->device_mtx, DEVICE_MUTEX_DELAY)) != E_OS_OK) {
        if (ret_os == E_OS_ERR_BUSY) {
            return DRV_RC_CONTROLLER_IN_USE;
        }
        return DRV_RC_FAIL;
    }
    pm_wakelock_acquire(&flash_dev->wakelock, SPI_FLASH_WAKELOCK_TIMEOUT);
    // wake up the flash
    if ((ret = spi_flash_sleep(dev, true)) != DRV_RC_OK) {
        goto exit_mutex;
    }

    flash_dev->req.tx_len         = 1;
    flash_dev->req.tx_buff        = &command;
    flash_dev->req.rx_len         = 3;
    flash_dev->req.rx_buff        = (uint8_t*)rdid;

    command = FLASH_CMD_RDID;
    *rdid = 0;
    ret = spi_sync(dev, &flash_dev->req);

    // put the flash to sleep
    spi_flash_sleep(dev, false);
exit_mutex:
    // Give device mutex
    pm_wakelock_release(&flash_dev->wakelock);
    mutex_unlock(flash_dev->device_mtx, NULL);
    return ret;
}

DRIVER_API_RC spi_flash_get_status(struct device *dev, uint8_t *status)
{
    uint8_t command;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;

    flash_dev->req.tx_len         = 1;
    flash_dev->req.tx_buff        = &command;
    flash_dev->req.rx_len         = 1;
    flash_dev->req.rx_buff        = status;

    command = FLASH_CMD_RDSR;
    *status = 0;
    return spi_sync(dev, &flash_dev->req);
}

static DRIVER_API_RC spi_flash_get_rdscur(struct device *dev, uint8_t *rdscur)
{
    uint8_t command;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;

    flash_dev->req.tx_len  = 1;
    flash_dev->req.tx_buff = &command;
    flash_dev->req.rx_len  = 1;
    flash_dev->req.rx_buff = rdscur;

    command = FLASH_CMD_RDSCUR;
    *rdscur = 0;
    return spi_sync(dev, &flash_dev->req);
}

static DRIVER_API_RC spi_flash_write_enable(struct device *dev)
{
    uint8_t command = 0, status = 0;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;
    DRIVER_API_RC ret;

    flash_dev->req.tx_len  = 1;
    flash_dev->req.tx_buff = &command;
    flash_dev->req.rx_len  = 0;
    flash_dev->req.rx_buff = NULL;

    command = FLASH_CMD_WREN;

    // Send write enable command
    if ((ret = spi_sync(dev, &flash_dev->req)) != DRV_RC_OK) {
        return ret;
    }

    flash_dev->req.rx_len  = 1;
    flash_dev->req.rx_buff = &status;

    command = FLASH_CMD_RDSR;

    // Read status register to check if write is enabled
    if ((ret = spi_sync(dev, &flash_dev->req)) != DRV_RC_OK) {
        return ret;
    }

    return (status & FLASH_WEL_BIT) ? DRV_RC_OK : DRV_RC_FAIL;
}

DRIVER_API_RC spi_flash_read_byte(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint8_t *data)
{
    DRIVER_API_RC ret = DRV_RC_OK;
    OS_ERR_TYPE ret_os;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;
    uint8_t command[4];

    *retlen = 0;

    // Check input parameters
    if ((!flash_dev->is_init) || (len == 0)) {
        return DRV_RC_INVALID_OPERATION;
    }
    if((len+address) > flash_dev->mem_size) {
        return DRV_RC_OUT_OF_MEM;
    }

    // Take spi device mutex
    if ((ret_os = mutex_lock(flash_dev->device_mtx, DEVICE_MUTEX_DELAY)) != E_OS_OK) {
        if (ret_os == E_OS_ERR_BUSY) {
            return DRV_RC_CONTROLLER_IN_USE;
        }
        return DRV_RC_FAIL;
    }
    pm_wakelock_acquire(&flash_dev->wakelock, SPI_FLASH_WAKELOCK_TIMEOUT);

    // wake up the flash
    if ((ret = spi_flash_sleep(dev, true)) != DRV_RC_OK) {
        goto exit_mutex;
    }

    command[0] = FLASH_CMD_READ;
    command[1] = (uint8_t)(address >> 16);
    command[2] = (uint8_t)(address >> 8);
    command[3] = (uint8_t)address;

    flash_dev->req.tx_len  = 4;
    flash_dev->req.tx_buff = command;
    flash_dev->req.rx_len  = len;
    flash_dev->req.rx_buff = data;

    // TODO: use DMA if transfer is too long
    if ((ret = spi_sync(dev, &flash_dev->req)) == DRV_RC_OK) {
        // TODO: sba does not provide a retlen data (could be done through request.rx_len)
        //       so we set retlen to len if success else 0
        *retlen = len;
    }
    // put the flash to sleep
    spi_flash_sleep(dev, false);
exit_mutex:
    // Give device mutex
    pm_wakelock_release(&flash_dev->wakelock);
    mutex_unlock(flash_dev->device_mtx, NULL);
    return ret;
}

DRIVER_API_RC spi_flash_read(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data)
{
    DRIVER_API_RC ret = spi_flash_read_byte(dev, address, len<<2, retlen, (uint8_t*)data);
    *retlen = (*retlen >> 2);
    return ret;
}

DRIVER_API_RC spi_flash_write_byte(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint8_t *data)
{
    DRIVER_API_RC ret = DRV_RC_OK;
    OS_ERR_TYPE ret_os;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;
    uint8_t status;

    *retlen = 0;

    // Check input parameters
    if ((!flash_dev->is_init) || (len == 0)) {
        return DRV_RC_INVALID_OPERATION;
    }
    if((len+address) > flash_dev->mem_size) {
        return DRV_RC_OUT_OF_MEM;
    }

    // Take spi device mutex
    if ((ret_os = mutex_lock(flash_dev->device_mtx, DEVICE_MUTEX_DELAY)) != E_OS_OK) {
        if (ret_os == E_OS_ERR_BUSY) {
            return DRV_RC_CONTROLLER_IN_USE;
        }
        return DRV_RC_FAIL;
    }
    pm_wakelock_acquire(&flash_dev->wakelock, SPI_FLASH_WAKELOCK_TIMEOUT);

    // wake up the flash
    if ((ret = spi_flash_sleep(dev, true)) != DRV_RC_OK) {
        goto exit_mutex;
    }

    *retlen = len;

    // We can only program a page with PP command so we use several write operations
    unsigned int count; // Byte count to write for next program operation
    count = FLASH_PAGE_SIZE - (address & (FLASH_PAGE_SIZE-1));
    if (count > len) {
        count = len;
    }

    flash_dev->tx_buffer[0] = FLASH_CMD_PP;

    // Loop on pages to program
    for (; len; address+=count, data+=count, count = ((len-=count) > FLASH_PAGE_SIZE ? FLASH_PAGE_SIZE : len)) {
        // Enable write operation
        if ((ret = spi_flash_write_enable(dev)) != DRV_RC_OK) {
            *retlen -= len;
            goto exit_wakeup;
        }

        // Set data for write operation
        memcpy((uint8_t*)(flash_dev->tx_buffer)+4, data, count);
        flash_dev->req.tx_len  = count+4;
        flash_dev->req.tx_buff = flash_dev->tx_buffer;
        flash_dev->req.rx_len  = 0;
        flash_dev->req.rx_buff = NULL;

        flash_dev->tx_buffer[1] = (uint8_t)(address >> 16);
        flash_dev->tx_buffer[2] = (uint8_t)(address >> 8);
        flash_dev->tx_buffer[3] = (uint8_t)address;

        if ((ret = spi_sync(dev, &flash_dev->req)) != DRV_RC_OK) {
            *retlen -= len;
            goto exit_wakeup;
        }

        // loop until the write operation is complete
        do {
            if ((ret = spi_flash_get_status(dev, &status)) != DRV_RC_OK) {
                // Error detected
                *retlen -= len;
                goto exit_wakeup;
            }
        } while (status & FLASH_WIP_BIT);

        // Check for success
        if ((ret = spi_flash_get_rdscur(dev, &status)) != DRV_RC_OK) {
                // Error detected
                *retlen -= len;
                goto exit_wakeup;
        }
        if (status & FLASH_SECR_PFAIL_BIT) {
            // Write failed
            ret = DRV_RC_CHECK_FAIL;
            *retlen -= len;
            goto exit_wakeup;
        }
    }

exit_wakeup:
    spi_flash_sleep(dev, false);
exit_mutex:
    // Give device mutex
    pm_wakelock_release(&flash_dev->wakelock);
    mutex_unlock(flash_dev->device_mtx, NULL);
    return ret;
}

DRIVER_API_RC spi_flash_write(struct device *dev, uint32_t address, unsigned int len, unsigned int *retlen, uint32_t *data)
{
    DRIVER_API_RC ret = spi_flash_write_byte(dev, address, len<<2, retlen, (uint8_t*)data);
    *retlen = (*retlen >> 2);
    return ret;
}

static DRIVER_API_RC spi_flash_erase(struct device *dev, ER_TYPE er_type, unsigned int start, unsigned int count)
{
    DRIVER_API_RC ret = DRV_RC_OK;
    OS_ERR_TYPE ret_os;
    uint8_t command[16];
    unsigned int er_size, timeout, er_count;
    struct spi_flash_info* flash_dev = (struct spi_flash_info*)dev->priv;

    // Check input parameters
    if ((!flash_dev->is_init) || (count == 0)) {
        return DRV_RC_INVALID_OPERATION;
    }

    switch (er_type) {
        case ER_SECTOR:
            command[0] = FLASH_CMD_SE;
            er_size = flash_dev->sector_size;
            er_count = flash_dev->sector_count;
            timeout = FLASH_SECTOR_ERASE_MS;
            break;
        case ER_BLOCK:
            command[0] = FLASH_CMD_BE32K;
            er_size = flash_dev->block_size;
            er_count = flash_dev->block_count;
            timeout = FLASH_BLOCK_ERASE_MS;
            break;
        case ER_LARGE_BLOCK:
            command[0] = FLASH_CMD_BE;
            er_size = flash_dev->large_block_size;
            er_count = flash_dev->large_block_count;
            timeout = FLASH_LARGE_BLOCK_ERASE_MS;
            break;
        case ER_CHIP:
            command[0] = FLASH_CMD_CE;
            er_size = flash_dev->mem_size;
            er_count = 1;
            timeout = FLASH_CHIP_ERASE_MS;
            count = 1;
            start = 0;
            break;
        default:
            return DRV_RC_INVALID_OPERATION;
    }

    if ((count + start) > er_count) {
        return DRV_RC_OUT_OF_MEM;
    }

    // Take spi device mutex
    if ((ret_os = mutex_lock(flash_dev->device_mtx, DEVICE_MUTEX_DELAY)) != E_OS_OK) {
        if (ret_os == E_OS_ERR_BUSY) {
            return DRV_RC_CONTROLLER_IN_USE;
        }
        return DRV_RC_FAIL;
    }
    pm_wakelock_acquire(&flash_dev->wakelock, SPI_FLASH_WAKELOCK_TIMEOUT);

    // wake up the flash
    if ((ret = spi_flash_sleep(dev, true)) != DRV_RC_OK) {
        goto exit_mutex;
    }
    // TODO: Check write protection
    for(count+=start; start<count; start++) {
        uint32_t address = er_size*start;

        // Enable write operation
        if ((ret = spi_flash_write_enable(dev)) != DRV_RC_OK) {
            goto exit_wakeup;
        }

        flash_dev->req.tx_len  = 4;
        flash_dev->req.tx_buff = command;
        flash_dev->req.rx_len  = 0;
        flash_dev->req.rx_buff = NULL;

        command[1] = (uint8_t)(address >> 16);
        command[2] = (uint8_t)(address >> 8);
        command[3] = (uint8_t)address;

        if ((ret = spi_sync(dev, &flash_dev->req)) != DRV_RC_OK) {
            // Error detected
            goto exit_wakeup;
        }

        // Start timer
        timer_start(flash_dev->spi_timer, timeout, &ret_os);

        // Take timer semaphore (wait for FLASH_MAX_ERASE_MS)
        if ((ret_os = semaphore_take(flash_dev->spi_timer_sem, FLASH_MAX_ERASE_MS)) != E_OS_OK) {
            if (ret_os == E_OS_ERR_BUSY) {
                ret = DRV_RC_TIMEOUT;
            } else {
                ret = DRV_RC_FAIL;
            }
            goto exit_wakeup;
        }

        // loop until the erase is complete
        uint8_t status;
        do {
            if ((ret = spi_flash_get_status(dev, &status)) != DRV_RC_OK) {
                // Error detected
                goto exit_wakeup;
            }
        } while (status & FLASH_WIP_BIT);

        // Check for success
        if ((ret = spi_flash_get_rdscur(dev, &status)) != DRV_RC_OK) {
                // Error detected
                goto exit_wakeup;
        }
        if (status & FLASH_SECR_EFAIL_BIT) {
            // Erase failed
            ret = DRV_RC_CHECK_FAIL;
            goto exit_wakeup;
        }
    }
exit_wakeup:
    spi_flash_sleep(dev, false);
exit_mutex:
    // Give device mutex
    pm_wakelock_release(&flash_dev->wakelock);
    mutex_unlock(flash_dev->device_mtx, NULL);
    return ret;
}

DRIVER_API_RC spi_flash_ioctl(struct device *dev, uint32_t *result,
			      uint8_t ioctl)
{
	DRIVER_API_RC ret = DRV_RC_OK;

	switch (ioctl) {
	case STORAGE_SIZE:
		*result = FLASH_SIZE;
		break;
	case STORAGE_PAGE_SIZE:
		*result = FLASH_PAGE_SIZE;
		break;
	case STORAGE_SECTOR_SIZE:
		*result = FLASH_SECTOR_SIZE;
		break;
	case STORAGE_BLOCK_SIZE:
		*result = FLASH_BLOCK32K_SIZE;
		break;
	case STORAGE_LARGE_BLOCK_SIZE:
		*result = FLASH_BLOCK_SIZE;
		break;
	default:
		ret = DRV_RC_INVALID_OPERATION;
	}

	return ret;
}

DRIVER_API_RC spi_flash_sector_erase(struct device *dev, unsigned int start_sector, unsigned int sector_count)
{
    return spi_flash_erase(dev, ER_SECTOR, start_sector, sector_count);
}

DRIVER_API_RC spi_flash_block_erase(struct device *dev, unsigned int start_block, unsigned int block_count)
{
    return spi_flash_erase(dev, ER_BLOCK, start_block, block_count);
}

DRIVER_API_RC spi_flash_large_block_erase(struct device *dev, unsigned int start_block, unsigned int block_count)
{
    return spi_flash_erase(dev, ER_LARGE_BLOCK, start_block, block_count);
}

DRIVER_API_RC spi_flash_chip_erase(struct device *dev)
{
    return spi_flash_erase(dev, ER_CHIP, 0, 1);
}
