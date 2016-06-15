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
/* adc.h - ADC driver */


/**
 * @file
 *
 * @brief ADC driver header file.
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <device.h>

/**
 * ADC driver name.
 *
 * Name for the singleton instance of the ADC driver.
 *
 */
#define ADC_DRV_NAME "adc"

/**
 * Number of buffers.
 *
 * Number of reception buffers to be supported by the driver.
 */
#define BUFS_NUM 2

/**
 * 6 bits width code.
 *
 * Code number that represents a 6 bits sample width.
 **/
#define WIDTH_6_BIT   0x0

/**
 * 8 bits width code.
 *
 * Code number that represents a 8 bits sample width.
 **/
#define WIDTH_8_BIT   0x1

/**
 * 10 bits width code.
 *
 * Code number that represents a 10 bits sample width.
 **/
#define WIDTH_10_BIT  0x2

/**
 * 12 bits width code.
 *
 * Code number that represents a 12 bits sample width.
 **/
#define WIDTH_12_BIT  0x3

/** This type defines an pointer to an ADC callback routine.*/
typedef void (*adc_callback)( struct device *dev );

/** @brief Sequence entry
 *
 * This structure defines a sequence entry used by the ADC driver
 * to define a lecture or sample from a specific channel.
 */
struct io_adc_seq_entry
{
	/** Clock ticks delay before sampling the ADC.*/
	uint32_t sample_dly;
	/** Channel ID that should be sampled from the ADC*/
	uint8_t  channel_id;
};

/** @brief Sequence table
 *
 * This structure represents a list of sequence entries that are
 * used by the ADC driver to execute a sequence of samplings.
 */
struct io_adc_seq_table
{
	/*Pointer to a sequence entry array*/
	struct  io_adc_seq_entry *entries;
	/*Number of entrues in the sequence entry array*/
	uint8_t num_entries;
};

/** @brief ADC configuration
 * This structure defines the ADC configuration values
 * that define the ADC hardware instance and configuration.
 */
struct adc_config
{
	/**Register base address for hardware registers.*/
	uint32_t reg_base;
	/**IIO address for the IRQ mask register.*/
	uint32_t reg_irq_mask;
	/**IIO address for the error mask register.*/
	uint32_t reg_err_mask;
	/**Interruption vector for the reception ISR.*/
	uint8_t  rx_vector;
	/**Interruption vector for the error ISR.*/
	uint8_t  err_vector;
	/**FIFO TLD*/
	uint16_t fifo_tld;
	/**Input mode*/
	uint8_t  in_mode;
	/**Output mode*/
	uint8_t  out_mode;
	/**Capture mode*/
	uint8_t  capture_mode;
	/**Sequence mode*/
	uint8_t  seq_mode;
	/**Serial delay*/
	uint8_t	 serial_dly;
	/**Sample width*/
	uint8_t  sample_width;
	/**Clock ratio*/
	uint32_t clock_ratio;
};

/**@brief ADC information and data.
 *
 * This structure defines the data that will be used
 * during driver execution.
 */
struct adc_info
{
	/**State of execution of the driver*/
	uint8_t	 state;
	/**Current reception buffer index*/
	uint8_t	 index;
	/**Sequence size*/
	uint32_t seq_size;
	/**Reception buffers length*/
	uint32_t rx_len;
	/**Reception buffers array*/
	uint32_t *rx_buf[BUFS_NUM];
	/**Pointer to the reception callback.*/
	adc_callback rx_cb;
	/**Pointer to the error callback.*/
	adc_callback err_cb;
};

/**@brief ADC driver API
 *
 * This structure holds all API function pointers.
 */
struct adc_driver_api
{
	/**Pointer to the lock routine*/
	uint8_t (*lock)(void);
	/**Pointer to the unlock routine*/
	void (*unlock)(void);
	/**Pointer to the enable routine*/
	void (*enable)(struct device *dev);
	/**Pointer to the disable routine*/
	void (*disable)(struct device *dev);
	/**Pointer to the set_cb routine*/
	void (*set_cb)(struct device *dev, adc_callback cb_rx,
	               adc_callback cb_err);
	/**Pointer to the read routine*/
	uint8_t (*read)(struct device *dev, struct io_adc_seq_table *seq_tbl,
	                uint32_t *data, uint32_t data_len);
};

/**
 *
 * @brief Enable ADC hardware
 *
 * This routine enables the ADC hardware block for data
 * sampling for the specified device.
 *
 * @param dev Pointer to the structure that represents the device to be enabled.
 *
 * @return N/A
 */
inline void adc_enable(struct device *dev)
{
	struct adc_driver_api *api;

	api = (struct adc_driver_api *)dev->driver_api;
	return api->enable( dev );
}

/**
 *
 * @brief Disable ADC hardware
 *
 * This routine disables the ADC hardware block for data
 * sampling for the specified device.
 *
 * @param dev Pointer to the structure that represents the device to be disabled.
 *
 * @return N/A
 */
inline void adc_disable(struct device *dev)
{
	struct adc_driver_api *api;

	api = (struct adc_driver_api *)dev->driver_api;
	return api->disable( dev );
}

/**
 *
 * @brief Lock access to the ADC hardware.
 *
 * This routine locks the ADC hardware, preventing different threads of
 * execution to request data sampling simultaneously.
 *
 * @param dev Pointer to the structure that represents the device to be locked.
 *
 * @return Returns zero if the driver was locked succesfully.
 */
inline uint8_t adc_lock(struct device *dev)
{
	struct adc_driver_api *api;

	api = (struct adc_driver_api *)dev->driver_api;
	return api->lock();
}

/**
 *
 * @brief Unlock access to the ADC hardware.
 *
 * This routine unlocks the ADC hardware, from a previous lock preventing
 * different threads of execution to request data sampling simultaneously.
 *
 * @param dev Pointer to the structure that represents the device to be locked.
 *
 * @return N/A
 */
inline void adc_unlock(struct device *dev)
{
	struct adc_driver_api *api;

	api = (struct adc_driver_api *)dev->driver_api;
	return api->unlock();
}

/**
 *
 * @brief Set callbacks routine
 *
 * This routine sets a callback routines that will be called
 * by the driver everytime that sample data is available
 * for consumption or an error is signaled.
 *
 * @param dev Pointer to the structure that represents the device to asign callbacks.
 * @param cb_rx Pointer to the function that will be set as reception callback.
 * @param cb_err Pointer to the function that will be set as error callback.
 *
 * @return N/A
 */
inline void adc_set_cb(struct device *dev, adc_callback cb_rx,
	adc_callback cb_err)
{
	struct adc_driver_api *api;

	api = (struct adc_driver_api *)dev->driver_api;
	return api->set_cb(dev, cb_rx, cb_err);
}

/**
 *
 * @brief Set a read request.
 *
 * This routine sets a read/sampling request to the ADC hardware block.
 * The read request is described by a sequence table. The read data is not
 * available for consumption until it is indicated by a callback.
 *
 * @param dev Pointer to the structure that represents the device to be read.
 * @param seq_tbl Pointer to the structure that represents the sequence table.
 * @param data Pointer to a data buffer that stores the data read.
 * @param data_len Data buffer length.
 *
 * @return Returns zero if the read was requested succesfully.
 */
inline uint8_t adc_read(struct device *dev,
	struct io_adc_seq_table *seq_tbl,
	uint32_t *data, uint32_t data_len)
{
	struct adc_driver_api *api;

	api = (struct adc_driver_api *)dev->driver_api;
	return api->read(dev, seq_tbl, data, data_len);
}
#endif  /* ADC_H_ */
