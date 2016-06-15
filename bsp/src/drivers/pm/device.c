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

#include <stddef.h>
#include <errno.h>

#include "infra/device.h"
#include "infra/log.h"
#include "machine.h"

/**
 * \brief Root device driver
 */
static struct bus_driver root_driver = {
	.init		= NULL,
	.suspend	= NULL,
	.resume		= NULL
};

/**
 * \brief Root device that is connected to all others (parent = NULL)
 */
static struct bus root_device = {
	.id		= ROOT_DEVICE_ID,
	.powerstate	= PM_NOT_INIT,
	.priv		= NULL,
	.parent		= NULL,
	.driver		= &root_driver
};

static void print_device(struct bus *device, int level)
{
	unsigned int i;

	if (device == NULL)
		return;
	pr_info(LOG_MODULE_DRV, "(%d) - %d (%d)", level, device->id,
		device->powerstate);

	level++;

	for (i = 0; i < device->bus_child_count; i++)
		print_device(&device->bus_child[i], level);
	for (i = 0; i < device->dev_child_count; i++)
		pr_info(LOG_MODULE_DRV, "(%d) - %d (%d)", level,
			device->dev_child[i]->id,
			device->dev_child[i]->powerstate);
}

void list_devices(void)
{
	print_device(&root_device, 0);
}

static struct device *find_device(struct bus *device, uint8_t id)
{
	unsigned int i;
	struct device *ret = NULL;

	// Check if device is valid
	if ((device == NULL) || (device->powerstate == PM_NOT_INIT))
		return NULL;

	if (device->id == id)
		return (struct device *)device;

	// Check devices
	for (i = 0; i < device->dev_child_count; i++) {
		if ((device->dev_child[i]->powerstate != PM_NOT_INIT) &&
		    (device->dev_child[i]->id == id))
			return device->dev_child[i];
	}
	// Check buses
	for (i = 0;
	     (i < device->bus_child_count) &&
	     (!(ret = find_device(&device->bus_child[i], id)));
	     i++) ;

	return ret;
}

struct device *get_device(uint8_t id)
{
	return find_device(&root_device, id);
}

static int init_device(struct device *device)
{
	int ret;

	pr_debug(LOG_MODULE_DRV, "init dev %d", device->id);
	if ((device->driver->init) && (ret = device->driver->init(device))) {
		device->powerstate = PM_NOT_INIT;
		return ret;
	}
	device->powerstate = PM_RUNNING;
	return 0;
}

static int resume_device(struct device *device)
{
	int ret = 0;

	if (device->powerstate <= PM_SHUTDOWN) {
		pr_error(LOG_MODULE_DRV, "device %d cannot resume (%d)",
			 device->id,
			 device->powerstate);
		return -EINVAL;
	}

	if (device->powerstate == PM_RUNNING)
		// Device already running
		return 0;

	pr_debug(LOG_MODULE_DRV, "resume dev %d", device->id);
	if (!(device->driver->resume) || (!(ret = device->driver->resume(device))))
		device->powerstate = PM_RUNNING;
	return ret;
}

static int suspend_device(struct device *device, PM_POWERSTATE state)
{
	int ret;

	if (device->powerstate <= state)
		// device already suspended
		return 0;

	pr_debug(LOG_MODULE_DRV, "suspend dev %d", device->id);

	if (!(device->driver->suspend) ||
	    (!(ret = device->driver->suspend(device, state)))) {
		device->powerstate = state;
		return 0;
	}
	pr_error(LOG_MODULE_DRV, "suspend (%d) failed for dev %d (%d)", state,
		 device->id,
		 ret);
	return ret;
}

static int init_bus_device(struct bus *device)
{
	unsigned int i;
	int ret = 0;

	pr_debug(LOG_MODULE_DRV, "init bus %d", device->id);

	if ((device->driver->init) && (ret = device->driver->init(device))) {
		device->powerstate = PM_NOT_INIT;
		return ret;
	}
	device->powerstate = PM_RUNNING;

	// Init devices first
	for (i = 0; (i < device->dev_child_count) && (!ret); i++) {
		// Set parent pointer
		device->dev_child[i]->parent = device;
		ret = init_device(device->dev_child[i]);
		if (ret) {
			pr_error(LOG_MODULE_DRV,
				 "failed to init device %d (%d)",
				 device->dev_child[i]->id,
				 ret);
			return ret;
		}
	}
	// Then init bus devices
	for (i = 0; (i < device->bus_child_count) && (!ret); i++) {
		// Set parent pointer
		device->bus_child[i].parent = device;
		ret = init_bus_device(&device->bus_child[i]);
		if (ret) {
			pr_error(LOG_MODULE_DRV,
				 "failed to init bus device %d (%d)",
				 device->bus_child[i].id,
				 ret);
			return ret;
		}
	}
	return 0;
}

static int resume_bus_device(struct bus *device)
{
	unsigned int i;
	int ret = 0;
	int fail = 0;

	pr_debug(LOG_MODULE_DRV, "resume bus %d", device->id);
	if (device->powerstate <= PM_SHUTDOWN) {
		pr_error(LOG_MODULE_DRV, "device %d not ready for resume (%d)",
			 device->id,
			 device->powerstate);
		return -EINVAL;
	}

	if (device->powerstate == PM_RUNNING)
		// Device already running
		return 0;

	if ((device->driver->resume) && (ret = device->driver->resume(device)))
		return ret;
	// Current device resumed
	device->powerstate = PM_RUNNING;

	// Resume all child devices
	for (i = 0; (i < device->dev_child_count) && (!ret); i++) {
		ret = resume_device(device->dev_child[i]);
		if (ret) {
			fail = ret;
			pr_error(LOG_MODULE_DRV,
				 "failed to resume device %d (%d)",
				 device->dev_child[i]->id,
				 ret);
		}
	}
	// Resume all child buses
	for (i = 0; (i < device->bus_child_count) && (!ret); i++) {
		ret = resume_bus_device(&device->bus_child[i]);
		if (ret) {
			fail = ret;
			pr_error(LOG_MODULE_DRV,
				 "failed to resume bus device %d (%d)",
				 device->bus_child[i].id,
				 ret);
		}
	}

	return fail;
}

static int suspend_bus_device(struct bus *device, PM_POWERSTATE state)
{
	int i;
	int dev_index, bus_index;
	int ret = 0;

	// Validate input argument
	if (state == PM_NOT_INIT)
		return -EINVAL;
	if (device->powerstate <= state)
		// Device already suspended
		return 0;

	// Suspend bus devices first
	for (i = device->bus_child_count - 1; i >= 0; i--) {
		if ((ret = suspend_bus_device(&device->bus_child[i], state))) {
			if (state > PM_SHUTDOWN) {
				// Resume bus devices
				bus_index = i;
				goto exit_buses;
			}
			// Suspend operation aborted
			return ret;
		}
	}

	// Then suspend devices
	for (i = device->dev_child_count - 1; i >= 0; i--) {
		if ((ret = suspend_device(device->dev_child[i], state))) {
			if (state > PM_SHUTDOWN) {
				// Resume other devices and buses
				bus_index = -1;
				dev_index = i;
				goto exit_devices;
			}
			// Suspend operation aborted
			return ret;
		}
	}

	// Suspend current bus device
	pr_debug(LOG_MODULE_DRV, "suspend bus %d", device->id);
	if ((device->driver->suspend) &&
	    (ret = device->driver->suspend(device, state))) {
		pr_error(LOG_MODULE_DRV, "suspend (%d) failed for dev %d (%d)",
			 state, device->id,
			 ret);
		if (state > PM_SHUTDOWN) {
			// Resume other devices and buses
			bus_index = device->bus_child_count;
			dev_index = device->dev_child_count;
			goto exit_devices;
		}
		// Suspend operation aborted
		return ret;
	}
	device->powerstate = state;
	return 0;

exit_devices:
	// Resume devices

	for (; (++dev_index) < device->dev_child_count; )
		resume_device(device->dev_child[dev_index]);
exit_buses:
	// Resume bus devices
	for (; (++bus_index) < device->bus_child_count; )
		resume_bus_device(&device->bus_child[bus_index]);
	return ret;
}

int init_devices(struct device **pf_devices, unsigned int dev_count,
		 struct bus *pf_buses,
		 unsigned int bus_count)
{
	if (root_device.powerstate != PM_NOT_INIT)
		// Root device already init
		return 0;
	// Link array with root device
	root_device.dev_child = pf_devices;
	root_device.dev_child_count = dev_count;

	// Link array with root device
	root_device.bus_child = pf_buses;
	root_device.bus_child_count = bus_count;

	return init_bus_device(&root_device);
}

int resume_devices(void)
{
	return resume_bus_device(&root_device);
}

int suspend_devices(PM_POWERSTATE state)
{
	return suspend_bus_device(&root_device, state);
}
