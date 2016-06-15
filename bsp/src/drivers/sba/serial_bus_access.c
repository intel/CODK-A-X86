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

#include "drivers/serial_bus_access.h"
#include "infra/wakelock_ids.h"
#include "infra/log.h" // For logger API
#include "os/os.h"     // For balloc/bfree
#include "drivers/soc_spi.h"

#ifdef CONFIG_SS_I2C
#include "drivers/ss_i2c_iface.h"
#endif
#ifdef CONFIG_SS_SPI
#include "drivers/ss_spi_iface.h"
#endif

#include "machine.h"
#include "drivers/clk_system.h"

#define SBA_WAKELOCK_TIMEOUT 1000

static DRIVER_API_RC execute_request(sba_request_t *request);
static DRIVER_API_RC sba_dev_init(struct bus *dev, union sba_config *config);

/*
 * Lookup table between SBA aliases and actual controller names
 */
static uint8_t controllers_lookup_table[] ={
        [SBA_SPI_MASTER_0]    = SOC_SPI_MASTER_0,
        [SBA_SPI_MASTER_1]    = SOC_SPI_MASTER_1,
        [SBA_SPI_SLAVE_0]     = SOC_SPI_SLAVE_0,
        [SBA_I2C_MASTER_0]    = SOC_I2C_0,
        [SBA_I2C_MASTER_1]    = SOC_I2C_1,
#ifdef CONFIG_SS_SPI
        [SBA_SS_SPI_MASTER_0] = SPI_SENSING_0,
        [SBA_SS_SPI_MASTER_1] = SPI_SENSING_1,
#endif
#ifdef CONFIG_SS_I2C
        [SBA_SS_I2C_MASTER_0] = I2C_SENSING_0,
        [SBA_SS_I2C_MASTER_1] = I2C_SENSING_1
#endif
};

#define NB_BUS sizeof(controllers_lookup_table)

struct bus *sba_bus_device_array[NB_BUS] = {NULL};

uint32_t get_bus_id_from_sba(uint32_t sba_alias_id)
{
        return controllers_lookup_table[sba_alias_id];
}

static int pm_sba_init(struct bus *dev)
{
    DRIVER_API_RC ret;
    struct sba_master_cfg_data *sba_dev ;

    if (!dev) {return DRV_RC_FAIL;}
    sba_dev = (struct sba_master_cfg_data*)dev->priv;
    pr_debug(LOG_MODULE_DRV, "sba: init device %d (%d)", dev->id, sba_dev->bus_id);


    // Device is not initialised
    sba_dev->controller_initialised = 0;

    // Init sba master device
    if ((ret = sba_dev_init(dev, &sba_dev->config)) != DRV_RC_OK) {
        pr_error(LOG_MODULE_DRV, "sba: init device %d (%d) failed", dev->id, sba_dev->bus_id);
        return -1;
    }

    pm_wakelock_init(&sba_dev->sba_wakelock, SBA_WAKELOCK_BASE | sba_dev->bus_id);

    // Device initialized
    sba_dev->controller_initialised = 1;
    sba_bus_device_array[sba_dev->bus_id] = dev;
    return 0;
}

DRIVER_API_RC sba_clock_state(struct sba_master_cfg_data* sba_dev, int enabled){
    DRIVER_API_RC rc = DRV_RC_OK;

    switch(sba_dev->bus_id){
#ifdef CONFIG_INTEL_QRK_SPI
        case (SBA_SPI_MASTER_0):
        case (SBA_SPI_MASTER_1):
        case (SBA_SPI_SLAVE_0):
            if (enabled){
                rc = soc_spi_clock_enable(sba_dev);
            } else {
                rc = soc_spi_clock_disable(sba_dev);
            }
            break;
#endif
#ifdef CONFIG_SS_SPI
        case (SBA_SS_SPI_MASTER_0):
        case (SBA_SS_SPI_MASTER_1):
            if (enabled){
                rc = ss_spi_clock_enable(sba_dev);
            } else {
                rc = ss_spi_clock_disable(sba_dev);
            }
            break;
#endif
#ifdef CONFIG_INTEL_QRK_I2C
        case(SBA_I2C_MASTER_0):
        case(SBA_I2C_MASTER_1):
            if (enabled){
                rc = soc_i2c_clock_enable(sba_dev);
            } else {
                rc = soc_i2c_clock_disable(sba_dev);
            }
            break;
#endif
#ifdef CONFIG_SS_I2C
        case (SBA_SS_I2C_MASTER_0):         //I2C_SS
        case (SBA_SS_I2C_MASTER_1):
            if (enabled){
                rc = ss_i2c_clock_enable(sba_dev);
            } else {
                rc = ss_i2c_clock_disable(sba_dev);
            }
            break;
#endif
        default:
            rc = DRV_RC_INVALID_OPERATION;
            break;
        }
    return rc;
}

static DRIVER_API_RC sba_clock_enable(struct sba_master_cfg_data* sba_dev){
    return sba_clock_state(sba_dev, 1);
}

static DRIVER_API_RC sba_clock_disable(struct sba_master_cfg_data* sba_dev){
  return sba_clock_state(sba_dev, 0);
}

static int sba_suspend(struct bus *dev, PM_POWERSTATE state)
{
    struct sba_master_cfg_data* sba_dev = (struct sba_master_cfg_data*)dev->priv;
    if (!sba_dev) {
        return -1;
    }

    pr_debug(LOG_MODULE_DRV, "sba: suspend device %d (%d) (%d)", dev->id, sba_dev->bus_id, state);
    return 0;
}

static int sba_resume(struct bus *dev)
{
    struct sba_master_cfg_data* sba_dev = (struct sba_master_cfg_data*)dev->priv;
    if (!sba_dev) {
        return -1;
    }

    sba_init(sba_dev);

    pr_debug(LOG_MODULE_DRV, "sba: resume device %d (%d)", dev->id, sba_dev->bus_id);
    return 0;
}

struct bus_driver serial_bus_access_driver = {.init = pm_sba_init,
                                              .suspend = sba_suspend,
                                              .resume = sba_resume};

void sba_err_callback(uint32_t bus_id);

/*! \fn     static void sba_generic_callback(uint32_t bus_id, int8_t status)
*
*  \brief   Function executed when SPI or I2C read, write or transfer is over.
*           This function is called in interrupt context only.
*
*  \param   bus_id      : Bus identifier
*  \param   status      : Status
*
*/
static void sba_generic_callback(uint32_t bus_id, int8_t status)
{
    DRIVER_API_RC rc = DRV_RC_OK;

    // Little hack to get device because we cannot give a priv data to i2c/spi driver (only bus_id)
    struct bus *dev;
    struct sba_master_cfg_data* sba_dev;
    if (((unsigned int)bus_id) >= NB_BUS || !(dev = sba_bus_device_array[bus_id])) {
        return;
    }
    sba_dev = (struct sba_master_cfg_data*)dev->priv;

    if ((sba_dev->current_request) == NULL) {
        panic(E_OS_ERR_UNKNOWN); // Panic because we should never reach this point.
    } else {
        sba_dev->current_request->status = status;
        if (NULL != sba_dev->current_request->callback){
            sba_dev->current_request->callback(sba_dev->current_request);
        }

        if ((sba_dev->current_request = (sba_request_t *)list_get(&sba_dev->request_list)) != NULL) {
            rc = execute_request(sba_dev->current_request);
            if (rc != DRV_RC_OK) {
                sba_err_callback(bus_id);
            }
        } else {
            sba_clock_disable(sba_dev);
            pm_wakelock_release(&sba_dev->sba_wakelock);
        }
    }
}


/*! \fn     void sba_err_callback(uint32_t bus_id)
*
*  \brief   Function executed when an error occurred during a transaction on SPI.
*           This function is called in interrupt context only.
*
*  \param   bus_id      : Bus identifier
*
*/
void sba_err_callback(uint32_t bus_id)
{
    sba_generic_callback(bus_id, -1);
}


/*! \fn     void sba_callback(uint32_t bus_id)
*
*  \brief   Function executed when an error occurred during a transaction on SPI.
*           This function is called in interrupt context only.
*
*  \param   bus_id      : Bus identifier
*
*/
void sba_callback(uint32_t bus_id)
{
    sba_generic_callback(bus_id, 0);
}

static DRIVER_API_RC sba_dev_init(struct bus *dev, union sba_config *config)
{
    struct sba_master_cfg_data* sba_dev = (struct sba_master_cfg_data*)dev->priv;

    if (((unsigned int)(sba_dev->bus_id) < NB_BUS) && (sba_dev->controller_initialised)) {
        // Device already initialized
        return DRV_RC_OK;
    }
    return sba_init(sba_dev);
}

DRIVER_API_RC sba_init(struct sba_master_cfg_data* sba_dev)
{
    DRIVER_API_RC rc = DRV_RC_FAIL; // TODO Initialisation not needed when ifdef will be removed
    union sba_bus_config {
        i2c_cfg_data_t i2c_config_data;
        spi_cfg_data_t spi_config_data;
    } bus_config;


    rc = sba_clock_enable(sba_dev);
    if (rc != DRV_RC_OK) {
        return rc;
    }

    // Set config data for SPI or I2C
    switch (sba_dev->bus_id) {
        case (SBA_SPI_MASTER_0):            //SPI_SOC
        case (SBA_SPI_MASTER_1):
        case (SBA_SPI_SLAVE_0):
#ifdef CONFIG_SS_SPI
        case (SBA_SS_SPI_MASTER_0):         //SPI_SS
        case (SBA_SS_SPI_MASTER_1):
#endif
            bus_config.spi_config_data.speed           = sba_dev->config.spi_config.speed;
            bus_config.spi_config_data.txfr_mode       = sba_dev->config.spi_config.txfr_mode;
            bus_config.spi_config_data.data_frame_size = sba_dev->config.spi_config.data_frame_size;
            bus_config.spi_config_data.slave_enable    = sba_dev->config.spi_config.slave_enable;
            bus_config.spi_config_data.bus_mode        = sba_dev->config.spi_config.bus_mode;
            bus_config.spi_config_data.loopback_enable = sba_dev->config.spi_config.loopback_enable;
            bus_config.spi_config_data.cb_xfer         = sba_callback;
            bus_config.spi_config_data.cb_err          = sba_err_callback;
            bus_config.spi_config_data.cb_slave_rx     = sba_callback;
            bus_config.spi_config_data.cb_xfer_data    = sba_dev->bus_id;
            bus_config.spi_config_data.cb_err_data     = sba_dev->bus_id;

            switch(sba_dev->bus_id){
#ifdef CONFIG_INTEL_QRK_SPI
                case (SBA_SPI_MASTER_0):
                case (SBA_SPI_MASTER_1):
                case (SBA_SPI_SLAVE_0):

                    rc = soc_spi_set_config(get_bus_id_from_sba(sba_dev->bus_id), &bus_config.spi_config_data);
                    break;
#endif
#ifdef CONFIG_SS_SPI
                case (SBA_SS_SPI_MASTER_0):
                case (SBA_SS_SPI_MASTER_1):
                    rc = ss_spi_set_config(get_bus_id_from_sba(sba_dev->bus_id), &bus_config.spi_config_data);
                    break;
#endif
                default:
                    rc = DRV_RC_INVALID_OPERATION;
                    break;
            }
            break;  /* END SPI CFG */

        case (SBA_I2C_MASTER_0):            //I2C_SOC
        case (SBA_I2C_MASTER_1):
#ifdef CONFIG_SS_I2C
        case (SBA_SS_I2C_MASTER_0):         //I2C_SS
        case (SBA_SS_I2C_MASTER_1):
#endif
            bus_config.i2c_config_data.speed           = sba_dev->config.i2c_config.speed;
            bus_config.i2c_config_data.addressing_mode = sba_dev->config.i2c_config.addressing_mode;
            bus_config.i2c_config_data.mode_type       = sba_dev->config.i2c_config.mode_type;
            bus_config.i2c_config_data.slave_adr       = sba_dev->config.i2c_config.slave_adr;
            bus_config.i2c_config_data.cb_rx           = sba_callback;
            bus_config.i2c_config_data.cb_tx           = sba_callback;
            bus_config.i2c_config_data.cb_err          = sba_err_callback;
            bus_config.i2c_config_data.cb_tx_data      = sba_dev->bus_id;
            bus_config.i2c_config_data.cb_rx_data      = sba_dev->bus_id;
            bus_config.i2c_config_data.cb_err_data     = sba_dev->bus_id;

            switch(sba_dev->bus_id){
#ifdef CONFIG_INTEL_QRK_I2C
                case(SBA_I2C_MASTER_0):
                case(SBA_I2C_MASTER_1):
                    rc = soc_i2c_set_config(get_bus_id_from_sba(sba_dev->bus_id), &bus_config.i2c_config_data);
                    break;
#endif
#ifdef CONFIG_SS_I2C
                case (SBA_SS_I2C_MASTER_0):         //I2C_SS
                case (SBA_SS_I2C_MASTER_1):
                    rc = ss_i2c_set_config(get_bus_id_from_sba(sba_dev->bus_id), &bus_config.i2c_config_data);
                    break;
#endif
                default:
                    rc = DRV_RC_INVALID_OPERATION;
                    break;
            }
            break;  /* END I2C CFG */
        default:
            rc = DRV_RC_INVALID_CONFIG;
            break;
    } /* switch case end */
    sba_clock_disable(sba_dev);
    return rc;
}

DRIVER_API_RC sba_exec_dev_request(struct sba_device *dev, struct sba_request *req)
{
    struct bus *p_dev = dev->dev.parent;
    req->bus_id = ((struct sba_master_cfg_data*)p_dev->priv)->bus_id;
    return sba_exec_request(req);
}

/*! \fn     DRIVER_API_RC sba_exec_request (sba_request_t * request)
*
*  \brief   Function to add a request in the requests list.
*           Configuration parameters must be valid or an error is returned - see return values below.
*
*  \param   request      : pointer to the request structure
*
*  \return  DRV_RC_OK on success,
*           DRV_RC_INVALID_CONFIG        - if any configuration parameters are not valid
*           DRV_RC_INVALID_OPERATION     - if both Rx and Tx while it is not implemented
*           DRV_RC_CONTROLLER_IN_USE     - when device is busy
*           DRV_RC_FAIL                  otherwise
*/
DRIVER_API_RC sba_exec_request(sba_request_t *request)
{
    DRIVER_API_RC rc = DRV_RC_OK;
    // Here we have to do a little hack to get the bus device.
    // We keep in sba driver an array of all bus devices indexed by bus_id
    // instead of getting in using sba_device->parent
    // (because everyone can use sba_exec_request, not just bus devices)
    if (request->bus_id >= NB_BUS || !sba_bus_device_array[request->bus_id]) {
        return DRV_RC_FAIL;
    }
    struct bus *dev = sba_bus_device_array[request->bus_id];
    struct sba_master_cfg_data* sba_dev = (struct sba_master_cfg_data*)dev->priv;
    // Form this point everything is fine

    if (sba_dev->controller_initialised != 1) {
        return DRV_RC_FAIL;
    }

    uint32_t saved = interrupt_lock();

    pm_wakelock_acquire(&sba_dev->sba_wakelock, SBA_WAKELOCK_TIMEOUT);
    sba_clock_enable(sba_dev);
    if (sba_dev->current_request == NULL) {
        sba_dev->current_request = request;
        interrupt_unlock(saved);
        rc = execute_request(request);
    } else {
        list_add(&sba_dev->request_list, (list_t *)request);
        interrupt_unlock(saved);
    }
    return rc;
}


/*! \fn     DRIVER_API_RC execute_request(sba_request_t * request)
*
*  \brief   Function to execute a request from the requests list.
*           Configuration parameters must be valid or an error is returned - see return values below.
*
*  \param   request      : pointer to the request structure
*
*  \return  DRV_RC_OK on success,
*           DRV_RC_INVALID_OPERATION     - if any configuration parameters are not valid
*           DRV_RC_CONTROLLER_IN_USE     - when device is busy
*           DRV_RC_FAIL                  otherwise
*/
static DRIVER_API_RC execute_request(sba_request_t *request)
{
    DRIVER_API_RC rc = DRV_RC_FAIL; // TODO Initialisation not needed when ifdef will be removed

    switch(request->request_type) {
        case SBA_RX:
            switch(request->bus_id){
#ifdef CONFIG_INTEL_QRK_SPI
                case (SBA_SPI_MASTER_0):            //SPI_SOC
                case (SBA_SPI_MASTER_1):
                case (SBA_SPI_SLAVE_0):
                    rc = soc_spi_transfer(  get_bus_id_from_sba(request->bus_id), NULL, 0, request->rx_buff, request->rx_len, request->full_duplex, request->addr.cs);
                    break;
#endif
#ifdef CONFIG_INTEL_QRK_I2C
                case (SBA_I2C_MASTER_0):            //I2C_SOC
                case (SBA_I2C_MASTER_1):
                    rc = soc_i2c_read(get_bus_id_from_sba(request->bus_id), request->rx_buff, request->rx_len, request->addr.slave_addr);
                    break;
#endif
#ifdef CONFIG_SS_SPI
                case (SBA_SS_SPI_MASTER_0):         //SPI_SS
                case (SBA_SS_SPI_MASTER_1):
                    rc = ss_spi_transfer(get_bus_id_from_sba(request->bus_id), NULL, 0, request->rx_buff, request->rx_len, request->addr.cs);
                    break;
#endif
#ifdef CONFIG_SS_I2C
                case (SBA_SS_I2C_MASTER_0):         //I2C_SS
                case (SBA_SS_I2C_MASTER_1):
                    rc = ss_i2c_read(get_bus_id_from_sba(request->bus_id), request->rx_buff, request->rx_len, request->addr.slave_addr);
                    break;
#endif
                default:
                    rc = DRV_RC_INVALID_CONFIG;
                    break;
            }
            break;
        case SBA_TX:
            switch(request->bus_id){
#ifdef CONFIG_INTEL_QRK_SPI
                case (SBA_SPI_MASTER_0):            //SPI_SOC
                case (SBA_SPI_MASTER_1):
                case (SBA_SPI_SLAVE_0):
                    rc = soc_spi_transfer(  get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, NULL, 0, request->full_duplex, request->addr.cs);
                    break;
#endif
#ifdef CONFIG_INTEL_QRK_I2C
                case (SBA_I2C_MASTER_0):            //I2C_SOC
                case (SBA_I2C_MASTER_1):
                    rc = soc_i2c_write(get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, request->addr.slave_addr);
                    break;
#endif
#ifdef CONFIG_SS_SPI
                case (SBA_SS_SPI_MASTER_0):         //SPI_SS
                case (SBA_SS_SPI_MASTER_1):
                    rc = ss_spi_transfer(get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, NULL, 0, request->addr.cs);
                    break;
#endif
#ifdef CONFIG_SS_I2C
                case (SBA_SS_I2C_MASTER_0):         //I2C_SS
                case (SBA_SS_I2C_MASTER_1):
                    rc = ss_i2c_write(get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, request->addr.slave_addr);
                    break;
#endif
                default:
                    rc = DRV_RC_INVALID_CONFIG;
                    break;
            }
            break;
        case SBA_TRANSFER:
            switch(request->bus_id){
#ifdef CONFIG_INTEL_QRK_SPI
                case (SBA_SPI_MASTER_0):            //SPI_SOC
                case (SBA_SPI_MASTER_1):
                case (SBA_SPI_SLAVE_0):
                    rc = soc_spi_transfer(  get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, request->rx_buff, request->rx_len, request->full_duplex, request->addr.cs);
                    break;
#endif
#ifdef CONFIG_INTEL_QRK_I2C
                case (SBA_I2C_MASTER_0):            //I2C_SOC
                case (SBA_I2C_MASTER_1):
                    rc = soc_i2c_transfer(get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, request->rx_buff, request->rx_len, request->addr.slave_addr);
                    break;
#endif
#ifdef CONFIG_SS_SPI
                case (SBA_SS_SPI_MASTER_0):         //SPI_SS
                case (SBA_SS_SPI_MASTER_1):
                    rc = ss_spi_transfer(get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, request->rx_buff, request->rx_len, request->addr.cs);
                    break;
#endif
#ifdef CONFIG_SS_I2C
                case (SBA_SS_I2C_MASTER_0):         //I2C_SS
                case (SBA_SS_I2C_MASTER_1):
                    rc = ss_i2c_transfer(get_bus_id_from_sba(request->bus_id), request->tx_buff, request->tx_len, request->rx_buff, request->rx_len, request->addr.slave_addr);
                    break;
#endif
                default:
                    rc = DRV_RC_INVALID_CONFIG;
                    break;
            }
            break;
        default:
            rc = DRV_RC_INVALID_OPERATION;
            break;
    }

    return rc;
}
