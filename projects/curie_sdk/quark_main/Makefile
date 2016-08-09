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
OUT         := $(T)/out/current/quark_se/quark

KCONFIG_FILE ?= $(OUT)/.config
export KCONFIG_FILE
include $(KCONFIG_FILE)

ifeq ($(BUILDVARIANT),release)
EXTRA_CFLAGS += -DNDEBUG=1
endif

EXTRA_CFLAGS += -g
EXTRA_CFLAGS += -I$(T)/firmware/framework/include
EXTRA_CFLAGS += -I$(T)/firmware/bsp/include
# TODO remove this one
EXTRA_CFLAGS += -I$(T)/firmware/bsp/include/machine/soc/quark_se/

EXTRA_CFLAGS += -D_UNSUPPORTED_STD_LIBS
EXTRA_CFLAGS += -nostdlib -nostartfiles -nostdinc

EXTRA_CFLAGS += -DCLOCK_SPEED=32

EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/include/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/kernel/microkernel/include/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/include/microkernel/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/lib/libc/minimal/include/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/arch/x86/platforms/quark_se-x86/
EXTRA_CFLAGS += -I$(ZEPHYR_BASE)/arch/x86/include

# FIXME: this is a Zephyr configuration parameter
EXTRA_CFLAGS += -DCONFIG_UART_INTERRUPT_DRIVEN

EXTRA_CFLAGS += -D__CPU_QRK__

# ble support, additional include paths.
EXTRA_CFLAGS += -I$(T)/firmware/framework/src/services/ble
EXTRA_CFLAGS += -I$(T)/firmware/framework/include/services

# Fixme: zephyr doesn't support passing empty PRJ_SRC
SOURCE_DIR =  $(T)/firmware/projects/curie_sdk/quark_main/dummy

ifndef CONFIG_OS_UNIT_TESTS

EXTRA_CFLAGS += -Werror

ifdef CONFIG_QUARK_DRIVER_TESTS
EXTRA_CFLAGS += -I$(T)/firmware/framework/unit_test
EXTRA_CFLAGS += -DDRIVER_TESTS -DSERVICE_TESTS
endif

else # CONFIG_OS_UNIT_TESTS

EXTRA_CFLAGS += -DTARGET_QRK
EXTRA_BUILD_CFLAGS += -I$(T)/firmware/framework/unit_test/os
EXTRA_BUILD_CFLAGS += -I$(T)/firmware/framework/include/zephyr

endif

.PHONY: FORCE

# These variables need to be exported as they are used in a sub-process started
# internally by the zephyr build env which requires them
export T
export OUT

#######################
include $(T)/firmware/build/Makefile.toolchain
OUTPUT_DIR=$(OUT)/zephyr
export OUTPUT_DIR
O=$(OUTPUT_DIR)

# default BSP
PLATFORM_CONFIG := quark_se-x86

CONF_FILE = $(CONFIG_ZEPHYR_CONF_FILE)

# This defines the type of main application we want to build
MDEF_FILE  := $(subst ",,$(CONFIG_OS_ZEPHYR_MDEF)) # "
# The main application is built from two libraries:
# - libkapp.a (runtime sources listed explicitly under PRJ_SRC)
# - libkbuildout.a  (framework sources selected by a configuration file)

ifdef CONFIG_OS_ZEPHYR_NANOKERNEL
KERNEL_TYPE = nano
EXTRA_CFLAGS += -DOS_ZEPHYR
EXTRA_CFLAGS += -DOS_VXNANO
EXTRA_CFLAGS += -include $(OUT)/zephyr/include/generated/autoconf.h
endif

ifdef CONFIG_OS_ZEPHYR_MICROKERNEL
KERNEL_TYPE = micro
EXTRA_CFLAGS += -DOS_ZEPHYR
EXTRA_CFLAGS += -I$(OUT)/zephyr/misc/generated/sysgen/
EXTRA_CFLAGS += -include $(OUT)/zephyr/include/generated/autoconf.h
# Flags used to preprocessor mapping headers for linker script
EXTRA_LINKER_CMD_OPT +=-D CONFIG_QUARK_SE_ARC_RAM_SIZE=$(CONFIG_QUARK_SE_ARC_RAM_SIZE)
EXTRA_LINKER_CMD_OPT +=-I$(T)/firmware/bsp/include/machine/soc/quark_se
EXTRA_LINKER_CMD_OPT +=-I$(T)/firmware/bsp/include/machine/soc/quark_se/quark\

EXTRA_CFLAGS += -DOS_VXMICRO
endif

export EXTRA_CFLAGS
export EXTRA_LINKER_CMD_OPT
export T

.PHONY: FORCE
export USE_CCACHE=0
include $(T)/firmware/external/zephyr/Makefile.inc
