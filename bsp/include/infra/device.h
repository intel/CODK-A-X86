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

#ifndef __DEVICE_H__
#define __DEVICE_H__

/**
 * @defgroup infra_device Device model
 * Device driver tree management.
 * @ingroup drivers
 * @{
 */

#include "infra/pm.h"

#define LINK_DEVICES_TO_BUS(devices) \
	.dev_child = (struct device **)(devices), \
	.dev_child_count = \
		(devices != NULL) ? (sizeof(devices) / sizeof(*devices)) : 0

#define LINK_DEVICES_TO_BUS_NULL \
	.dev_child = NULL, \
	.dev_child_count = 0

#define LINK_BUSES_TO_BUS(devices) \
	.bus_child = (struct bus *)(devices), \
	.bus_child_count = \
		(devices != NULL) ? (sizeof(devices) / sizeof(*devices)) : 0

#define LINK_BUSES_TO_BUS_NULL \
	.bus_child = NULL, \
	.bus_child_count = 0

// Structure prototypes
struct device;
struct bus;
struct driver;

/**
 * Device management structure.
 */
struct __attribute__((packed, aligned(4))) device  {
	struct bus *parent;             /*!< Parent device */
	void *priv;                     /*!< Priv data of device */
	struct driver *driver;          /*!< Driver used for device */
	PM_POWERSTATE powerstate : 8;   /*!< Powerstate of device */
	uint8_t id;                     /*!< ID of device */
};

/**
 * Bus device management structure.
 */
struct __attribute__((packed, aligned(4))) bus {
	struct bus *parent;                     /*!< Parent device */
	void *priv;                             /*!< Priv data of device */
	struct bus_driver *driver;              /*!< Driver used for device */
	struct bus *bus_child;                  /*!< First child bus device */
	struct device **dev_child;              /*!< First child device */
	uint8_t id;                             /*!< Name of device */
	uint8_t bus_child_count;                /*!< Child bus device count */
	uint8_t dev_child_count;                /*!< Child device count */
	PM_POWERSTATE powerstate : 8;           /*!< Powerstate of device */
};


/**
 * Device driver management structure
 */
struct driver {
	int	(*init)(struct device *dev);                            /*!< Pointer to the function to call for device init */
	int	(*suspend)(struct device *dev, PM_POWERSTATE state);    /*!< Pointer to the function to call for suspend event */
	int	(*resume)(struct device *dev);                          /*!< Pointer to the function to call for resume event */
};

/**
 * Bus driver management structure
 */
struct bus_driver {
	int	(*init)(struct bus *dev);                               /*!< Pointer to the function to call for device init */
	int	(*suspend)(struct bus *dev, PM_POWERSTATE state);       /*!< Pointer to the function to call for suspend event */
	int	(*resume)(struct bus *dev);                             /*!< Pointer to the function to call for resume event */
};

/**
 * Prints the device tree of current CPU.
 *
 * This function logs the device tree of the current CPU, using logger service.
 */
void list_devices(void);

/**
 * Suspends all devices in the device tree of current CPU.
 *
 * This function calls the suspend callback of all devices in the device tree.
 * If a suspend callback fails for something else than shutdown, all devices are resumed.
 *
 * @param state suspend type (for shutdown/deepsleep/sleep etc...).
 *
 * @return 0 if success else -1.
 */
int suspend_devices(PM_POWERSTATE state);

/**
 * Resumes all devices in the device tree of current CPU.
 *
 * This function calls the resume callback of all devices in the device tree.
 *
 * @return 0 if success else -1.
 */
int resume_devices(void);

/**
 * Adds all devices to power management infrastructure and init them.
 *
 * This function links all devices to the root device and calls init callback for each device.
 *
 * @param pf_devices Pointer to the platform devices array
 * @param dev_count  Number of devices in the platform devices array
 * @param pf_buses   Pointer to the platform bus devices array
 * @param bus_count  Number of devices in the platform bus devices array
 *
 * @return 0 if success else -1.
 */
int init_devices(struct device **pf_devices, unsigned int dev_count,
		 struct bus *pf_buses,
		 unsigned int bus_count);

/**
 * Initializes all board devices.
 *
 * This function must call init_devices.
 *
 * @return 0 if success else -1.
 */
int init_all_devices(void);

/**
 * Gets a pointer to the device that matches name parameter.
 *
 * This function searches in the device tree a running device matching the name parameter.
 *
 * @param id ID of the device to find
 *
 * @return Pointer to the device if found; NULL in case of error.
 */
struct device *get_device(uint8_t id);

/** @} */

#endif /* __DEVICE_H__ */
