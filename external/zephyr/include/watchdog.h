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

#ifndef _WDT_H_
#define _WDT_H_
#include <stdint.h>

typedef enum { DW_WDT_MODE_RESET = 0, DW_WDT_MODE_INTERRUPT_RESET } wdt_mode_t;

/**
 * WDT configuration struct.
 */
struct wdt_config{
        uint8_t timeout;
        wdt_mode_t mode;
        void (*interrupt_fn)(void);
};

typedef void (*wdt_api_enable)(void);
typedef void (*wdt_api_disable)(void);
typedef int (*wdt_api_set_config)(struct wdt_config *config);
typedef void (*wdt_api_get_config)(struct wdt_config *config);
typedef void (*wdt_api_reload)(void);




struct wdt_driver_api {
	wdt_api_enable enable;
	wdt_api_disable disable;
	wdt_api_get_config get_config;
	wdt_api_set_config set_config;
	wdt_api_reload reload;
};


inline void wdt_enable(struct device *dev)
{
	struct wdt_driver_api *api;

	api = (struct wdt_driver_api *)dev->driver_api;
	api->enable();
}

inline void wdt_disable(struct device *dev)
{
	struct wdt_driver_api *api;

	api = (struct wdt_driver_api *)dev->driver_api;
	api->disable();
}

inline void wdt_get_config(struct device *dev, struct wdt_config *config)
{
	struct wdt_driver_api *api;

	api = (struct wdt_driver_api *)dev->driver_api;
	api->get_config(config);
}

inline int wdt_set_config(struct device *dev, struct wdt_config *config)
{
	struct wdt_driver_api *api;

	api = (struct wdt_driver_api *)dev->driver_api;
	return api->set_config(config);
}

inline void wdt_reload(struct device *dev)
{
	struct wdt_driver_api *api;

	api = (struct wdt_driver_api *)dev->driver_api;
	api->reload();
}

#endif
