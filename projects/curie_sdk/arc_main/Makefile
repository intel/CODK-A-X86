#
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

T           ?= $(CURDIR)
OUT         := $(T)/out/current/quark_se/arc

KCONFIG_FILE ?= $(OUT)/.config
export KCONFIG_FILE
include $(KCONFIG_FILE)
include $(T)/firmware/build/Makefile.toolchain
OUTPUT_DIR=${OUT}/zephyr
export OUTPUT_DIR
O=$(OUTPUT_DIR)

# default BSP
ifeq ($(CONFIG_QEMU_X86),y)
PLATFORM_CONFIG = quark_se-x86
ARCH = x86
CONF_FILE = prj-Intel.conf
else
PLATFORM_CONFIG = quark_se-arc
ARCH = arc
CONF_FILE = prj-ARC.conf
endif
KERNEL_TYPE = nano




ifeq ($(BUILDVARIANT),release)
EXTRA_CFLAGS += -DNDEBUG=1
endif

EXTRA_CFLAGS += -g -D_UNSUPPORTED_STD_LIBS -nostdlib -nostartfiles -nostdinc

ifneq ($(CONFIG_QEMU_X86),y)
EXTRA_CFLAGS += -mcpu=ARCv2EM -mdiv-rem -mbarrel-shifter -mspfp
endif

EXTRA_CFLAGS += -I$(T)/firmware/framework/include
EXTRA_CFLAGS += -I$(T)/firmware/bsp/include
# TODO remove this one
EXTRA_CFLAGS += -I$(T)/firmware/bsp/include/machine/soc/quark_se/

ifdef CONFIG_ARC_OS_UNIT_TESTS
EXTRA_CFLAGS += -I$(T)/firmware/framework/include/zephyr/
EXTRA_CFLAGS += -I$(T)/firmware/framework/unit_test/os/
EXTRA_CFLAGS += -I$(T)/firmware/bsp/unit_test/os/
EXTRA_CFLAGS += -DOS_ZEPHYR
endif

EXTRA_CFLAGS += -DOS_VXNANO
ifeq ($(CONFIG_QEMU_X86),y)
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/arch/x86/platforms/quark_se-x86/
else
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/arch/arc/platforms/quark_se-arc/
endif
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/include/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/lib/libc/minimal/include
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/kernel/microkernel/include/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/include/microkernel/
EXTRA_CFLAGS += -I$(T)/out/current/quark_se/arc/zephyr/klib/include/
EXTRA_CFLAGS += -I$(T)/out/current/quark_se/arc/zephyr/sysgen/

EXTRA_CFLAGS += -include $(OUT)/zephyr/include/generated/autoconf.h

QUARK_SE_SENSOR_DIR := $(T)/firmware/framework/src/sensors
EXTRA_CFLAGS += -I$(QUARK_SE_SENSOR_DIR)/phy_sensor_api/include/

ifdef CONFIG_PSH_CORE
SENSOR_CORE_OPEN_PATH=$(T)/firmware/framework/include/sensors/sensor_core/psh_core
IPC_COMM_PATH=$(T)/firmware/framework/include/sensors/sensor_core/ipc

EXTRA_CFLAGS +=  -I$(SENSOR_CORE_OPEN_PATH) \
                 -I$(IPC_COMM_PATH)

EXTRA_CFLAGS += -I$(QUARK_SE_SENSOR_DIR)/sensor_core/algo/inc_api
EXTRA_CFLAGS += -I$(QUARK_SE_SENSOR_DIR)/sensor_core/algo/common
endif

ifdef CONFIG_OPEN_CORE
SENSOR_CORE_PATH = $(T)/firmware/framework/include/sensors/sensor_core

EXTRA_CFLAGS += -I$(SENSOR_CORE_PATH)/open_core/
EXTRA_CFLAGS += -I$(SENSOR_CORE_PATH)/psh_core/
EXTRA_CFLAGS += -I$(SENSOR_CORE_PATH)/ipc

# To run UDG algo in open core
EXTRA_CFLAGS += -I$(QUARK_SE_SENSOR_DIR)/sensor_core/algo/inc_api
EXTRA_CFLAGS += -I$(QUARK_SE_SENSOR_DIR)/sensor_core/algo/common
endif

EXTRA_CFLAGS += -Wall -Werror -Os
EXTRA_CFLAGS += -D__CPU_ARC__
EXTRA_CFLAGS += -DCLOCK_SPEED=32

export EXTRA_CFLAGS

# Fixme: zephyr doesn't support passing empty PRJ_SRC
PRJ_SRC += $(T)/firmware/bsp/src/os/zephyr/dummy.c

# Flags used to preprocessor mapping headers for linker script
EXTRA_LINKER_CMD_OPT +=-D CONFIG_QUARK_SE_ARC_RAM_SIZE=$(CONFIG_QUARK_SE_ARC_RAM_SIZE)
EXTRA_LINKER_CMD_OPT +=-I$(T)/firmware/bsp/include/machine/soc/quark_se
EXTRA_LINKER_CMD_OPT +=-I$(T)/firmware/bsp/include/machine/soc/quark_se/arc\

export EXTRA_LINKER_CMD_OPT

.PHONY: FORCE
export USE_CCACHE=0
include $(T)/firmware/external/zephyr/Makefile.inc
