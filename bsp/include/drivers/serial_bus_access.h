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

#ifndef INTEL_SBA_H_
#define INTEL_SBA_H_

#include "infra/device.h"
#include "util/list.h"
#include "drivers/common_spi.h"
#include "drivers/common_i2c.h"
#include "drivers/data_type.h"
#include "drivers/clk_system.h"

/**
 * @defgroup sba Quark_SE SOC SBA
 * Quark_SE SOC Serial Bus Access driver API.
 * @ingroup common_drivers
 * @{
 */

/**
* List of all request type
*/
typedef enum {
    SBA_RX = 0,               /*!< Read  */
    SBA_TX,                   /*!< Write */
    SBA_TRANSFER              /*!< Read and write */
} SBA_REQUEST_TYPE;

/**
 * List of all controllers in system ( IA and SS )
 */
typedef enum {
    SBA_SPI_MASTER_0 = 0,     /*!< SPI master controller 0, accessible by both processing entities */
    SBA_SPI_MASTER_1,         /*!< SPI master controller 1, accessible by both processing entities */
    SBA_SPI_SLAVE_0,          /*!< SPI slave controller */
    SBA_I2C_MASTER_0,         /*!< I2C master controller 0, accessible by both processing entities */
    SBA_I2C_MASTER_1,         /*!< I2C master controller 0, accessible by both processing entities */
#ifdef __CPU_ARC__
    SBA_SS_SPI_MASTER_0,      /*!< SPI master controller 0, accessible by ARC cpu only */
    SBA_SS_SPI_MASTER_1,      /*!< SPI master controller 1, accessible by ARC cpu only */
    SBA_SS_I2C_MASTER_0,      /*!< I2C master controller 0, accessible by ARC cpu only */
    SBA_SS_I2C_MASTER_1       /*!< I2C master controller 1, accessible by ARC cpu only */
#endif
} SBA_BUSID;

/**
 *  Request structure.
 */
struct sba_request;
typedef struct sba_request {
    list_t                    list;                 /*!< Pointer of the next request */
    SBA_REQUEST_TYPE          request_type;         /*!< Request type */
    uint32_t                  tx_len;               /*!< Lenght of the write buffer */
    uint8_t                  *tx_buff;              /*!< Write buffer */
    uint32_t                  rx_len;               /*!< Lenght of the read buffer */
    uint8_t                  *rx_buff;              /*!< Read buffer */
    uint8_t                   full_duplex;          /*!< Full duplex transfer request */
    union {
        SPI_SLAVE_ENABLE      cs;                   /*!< Chip select */
        uint32_t              slave_addr;           /*!< Address of the slave */
    } addr;
    SBA_BUSID                 bus_id;               /*!< Controller ID */
    int8_t                    status;               /*!< 0 if ok, -1 if error */
    void                     *priv_data;            /*!< User private data */
    void (*callback)(struct sba_request *);         /*!< Callback */
}sba_request_t;

/**
 *  SPI controller configuration.
 */
typedef struct sba_spi_config {
    uint32_t                  speed;                /*!< SPI bus speed in KHz   */
    SPI_TRANSFER_MODE         txfr_mode;            /*!< Transfer mode          */
    SPI_DATA_FRAME_SIZE       data_frame_size;      /*!< Data Frame Size ( 4 - 16 bits ) */
    SPI_SLAVE_ENABLE          slave_enable;         /*!< Slave Enable ( 0 = none - possibly used for Slaves that are selected by GPIO ) */
    SPI_BUS_MODE              bus_mode;             /*!< See SPI_BUS_MODE above for description */
    SPI_MODE_TYPE             spi_mode_type;        /*!< SPI Master or Slave mode */
    uint8_t                   loopback_enable;      /*!< Loopback enable */
}sba_spi_config_t;

/**
*  I2C controller configuration.
*/
typedef struct sba_i2c_config {
    I2C_SPEED                 speed;                /*!< Slow or Fast */
    I2C_ADDR_MODE             addressing_mode;      /*!< 7 bit / 10 bit addressing */
    I2C_MODE_TYPE             mode_type;            /*!< Master or Slave */
    uint32_t                  slave_adr;            /*!< I2C address if configured as a Slave */
}sba_i2c_config_t;

/**
 *  SBA controller configuration.
 */
union sba_config {
    sba_spi_config_t spi_config;
    sba_i2c_config_t i2c_config;
};

/**
 *  Structure to handle sba bus devices configuration
 */
struct sba_master_cfg_data {
    SBA_BUSID  bus_id;                          /*!< Controller ID */
    union sba_config config;                    /*!< SBA config*/
    /* internal fields */
    list_head_t    request_list;                /*!< List to store requests */
    sba_request_t *current_request;             /*!< current request pointer */
    uint8_t        controller_initialised;      /*!< Controller initialized flag */
    struct pm_wakelock sba_wakelock;            /*!< Power manager wakelock */
    struct clk_gate_info_s* clk_gate_info;      /*!< clock gate data */
};

/**
 *  Structure to handle sba slave devices
 */
struct sba_device {
    struct device        dev;        /*!< Device base structure */
    union {
        SPI_SLAVE_ENABLE cs;         /*!< Chip select */
        uint32_t         slave_addr; /*!< Address of the slave */
    } addr;
};

/** Export sba driver to link it with devices in board_config file (no probe/match system on asp) */
extern struct bus_driver serial_bus_access_driver;

/**
*  Function to configure specified SPI or I2C controller.
*
*  Configuration parameters must be valid or an error is returned - see return values below.
*
*  @param sba_dev            : pointer to bus configuration data
*
*  @return
*           - DRV_RC_OK on success,
*           - DRV_RC_DEVICE_TYPE_NOT_SUPPORTED    - if device type is not supported by this controller
*           - DRV_RC_INVALID_CONFIG               - if any configuration parameters are not valid
*           - DRV_RC_CONTROLLER_IN_USE,             if controller is in use
*           - DRV_RC_CONTROLLER_NOT_ACCESSIBLE      if controller is not accessible from this core
*           - DRV_RC_FAIL otherwise
*/
DRIVER_API_RC sba_init(struct sba_master_cfg_data* sba_dev);

/**
*  Function to add a request in the requests list.
*
*  Configuration parameters must be valid or an error is returned - see return values below.
*
*  @param request      : pointer to the request structure
*
*  @return
*           - DRV_RC_OK on success,
*           - DRV_RC_INVALID_CONFIG        - if any configuration parameters are not valid
*           - DRV_RC_INVALID_OPERATION     - if both Rx and Tx while it is not implemented
*           - DRV_RC_CONTROLLER_IN_USE     - when device is busy
*           - DRV_RC_FAIL                  otherwise
*/
DRIVER_API_RC sba_exec_request(sba_request_t *request);

/**
*  Function to add a request in the requests list for a specific device.
*
*  Configuration parameters must be valid or an error is returned - see return values below.
*
*  @param dev      : sba device used to send the request
*  @param req      : sba_request structure to send
*
*  @return
*           - DRV_RC_OK on success,
*           - DRV_RC_INVALID_CONFIG        - if any configuration parameters are not valid
*           - DRV_RC_INVALID_OPERATION     - if both Rx and Tx while it is not implemented
*           - DRV_RC_CONTROLLER_IN_USE     - when device is busy
*           - DRV_RC_FAIL                  otherwise
*/
DRIVER_API_RC sba_exec_dev_request(struct sba_device *dev, struct sba_request *req);

/**
*  Function to get physical bus id from logical bus id.
*
*  @param   sba_alias_id: logical bus id
*
*  @return  physical bus id
*/
uint32_t get_bus_id_from_sba(uint32_t sba_alias_id);
/** @} */

#endif /* INTEL_SBA_H_ */
