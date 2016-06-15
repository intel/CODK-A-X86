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

#ifndef _AIO_COMPARATOR_H_
#define _AIO_COMPARATOR_H_

enum aio_cmp_ref
{
	AIO_CMP_REF_A,		/**< Use reference A. */
	AIO_CMP_REF_B,		/**< Use reference B. */
};

enum aio_cmp_polarity
{
	AIO_CMP_POL_RISE,	/**< Match on rising edge. */
	AIO_CMP_POL_FALL,	/**< Match on falling edge. */
};

typedef void (*aio_cmp_cb)(void *);

typedef int (*aio_cmp_api_disable)(struct device *dev, uint8_t index);

typedef int (*aio_cmp_api_configure)(struct device *dev, uint8_t index,
    enum aio_cmp_polarity polarity, enum aio_cmp_ref refsel,
    aio_cmp_cb cb, void *param);

struct aio_cmp_driver_api {
	aio_cmp_api_disable disable;
	aio_cmp_api_configure configure;
};

/**
 * @brief Disable a particular comparator.
 *
 * This disables a comparator so that it no longer triggers interrupts.
 *
 * @param dev Device struct
 * @param index The index of the comparator to disable
 *
 * @return 0 if successful, otherwise failed.
 */
inline int aio_cmp_disable(struct device *dev, uint8_t index)
{
	struct aio_cmp_driver_api *api;

	api = (struct aio_cmp_driver_api *)dev->driver_api;
	return api->disable(dev, index);
}

/**
 * @brief Configure and enable a particular comparator.
 *
 * This performs configuration and enable a comparator, so that it will
 * generate interrupts when conditions are met.
 *
 * @param dev Device struct
 * @param index The index of the comparator to disable
 * @param polarity Match polarity (e.g. rising or falling)
 * @param refsel Reference for trigger
 * @param cb Function callback (aio_cmp_cb)
 * @param param Parameters to be passed to callback
 *
 * @return 0 if successful, otherwise failed.
 */
inline int aio_cmp_configure(struct device *dev, uint8_t index,
			     enum aio_cmp_polarity polarity,
			     enum aio_cmp_ref refsel,
			     aio_cmp_cb cb, void *param)
{
	struct aio_cmp_driver_api *api;

	api = (struct aio_cmp_driver_api *)dev->driver_api;
	return api->configure(dev, index, polarity, refsel, cb, param);
}

#endif /* _AIO_COMPARATOR_H_ */
