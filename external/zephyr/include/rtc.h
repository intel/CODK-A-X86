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

#ifndef _RTC_H_
#define _RTC_H_
#include <stdint.h>


#define RTC_DIVIDER 1

/** Number of RTC ticks in a second */
#define RTC_ALARM_SECOND (32768 / RTC_DIVIDER)
/** Number of RTC ticks in a minute */
#define RTC_ALARM_MINUTE ((RTC_ALARM_SECOND * 60) / RTC_DIVIDER)
/** Number of RTC ticks in an hour */
#define RTC_ALARM_HOUR ((RTC_ALARM_MINUTE * 60) / RTC_DIVIDER)
/** Number of RTC ticks in a day */
#define RTC_ALARM_DAY ((RTC_ALARM_HOUR * 24) / RTC_DIVIDER)



typedef struct {
        uint32_t init_val;
} rtc_config_t;

typedef struct
{
    uint8_t alarm_enable;           /*!< enable/disable alarm  */
    uint32_t alarm_val;             /*!< initial configuration value for the 32bit RTC alarm value  */
    void (*cb_fn)(void);	    /*!< Pointer to function to call when alarm value matches current RTC value */
} rtc_alarm_t;


typedef void (*rtc_api_enable)(void);
typedef void (*rtc_api_disable)(void);
typedef void (*rtc_api_clock_disable)(void);
typedef int (*rtc_api_set_config)(rtc_config_t *config);
typedef int (*rtc_api_set_alarm)(rtc_alarm_t *alarm_val);
typedef uint32_t (*rtc_api_read)(void);

struct rtc_driver_api {
	rtc_api_enable enable;
	rtc_api_disable disable;
	rtc_api_clock_disable clock_disable;
	rtc_api_read read;
	rtc_api_set_config set_config;
	rtc_api_set_alarm set_alarm;
};

inline uint32_t rtc_read(struct device *dev)
{
	struct rtc_driver_api *api;

	api = (struct rtc_driver_api *)dev->driver_api;
	return api->read();
}

inline void rtc_enable(struct device *dev)
{
	struct rtc_driver_api *api;

	api = (struct rtc_driver_api *)dev->driver_api;
	api->enable();
}

inline void rtc_clock_disable(struct device *dev)
{
	struct rtc_driver_api *api;

	api = (struct rtc_driver_api *)dev->driver_api;
	api->clock_disable();
}

inline void rtc_disable(struct device *dev)
{
	struct rtc_driver_api *api;

	api = (struct rtc_driver_api *)dev->driver_api;
	api->disable();
}

inline int rtc_set_config(struct device *dev, rtc_config_t *cfg)
{
	struct rtc_driver_api *api;

	api = (struct rtc_driver_api *)dev->driver_api;
	return api->set_config(cfg);
}

inline int rtc_set_alarm(struct device *dev, rtc_alarm_t *alarm)
{
	struct rtc_driver_api *api;

	api = (struct rtc_driver_api *)dev->driver_api;
	return api->set_alarm(alarm);
}

#endif
