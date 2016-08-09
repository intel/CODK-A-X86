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

T ?= $(CURDIR)
OUT := $(T)/out/current/quark_se/bootupdater
TOOLS_PATH ?= $(T)/firmware/tools
BINARY_VERSION_HEADER ?= $(TOOLS_PATH)/scripts/build_utils/add_binary_version_header.py
BOOTUPDATER_LDS ?= $(T)/firmware/bsp/bootable/bootupdater/quark.lds
QUARK_SE_DEVICE_DIR := $(T)/firmware/devices/quark_se

ifeq ($(filter ctb crb morpheus arduino101, $(BOARD)),)
$(error The bootloader updater can only run on the Curie Test/Reference Board, \
	Arduino101 and Morpheus)
endif

ifndef VERSION_MAJOR
$(error "VERSION_MAJOR not defined")
endif
ifndef VERSION_MINOR
$(error "VERSION_MINOR not defined")
endif
ifndef VERSION_PATCH
$(error "VERSION_PATCH not defined")
endif
ifndef VERSION_STRING_SUFFIX
$(error "VERSION_STRING_SUFFIX not defined")
endif

ARCH=x86
include $(T)/firmware/build/Makefile.toolchain

KCONFIG_FILE ?= $(OUT)/.config
include $(KCONFIG_FILE)

ifeq ($(BUILDVARIANT),release)
CFLAGS += -DNDEBUG=1
endif

CFLAGS += -g
CFLAGS += -I$(T)/firmware/bsp/include
CFLAGS += -I$(T)/firmware/bsp/bootable/bootupdater/include
CFLAGS += -D__CPU_QRK__
CFLAGS += -nostartfiles
CFLAGS += -ffreestanding -nostartfiles -fno-exceptions
CFLAGS += -fno-asynchronous-unwind-tables
CFLAGS += -fdata-sections -ffunction-sections

.PHONY: FORCE


$(OUT)/bootupdater.lds: $(BOOTUPDATER_LDS) | $(OUT)
	$(CC) $(CFLAGS) -E -P -o $@ -ansi -D__ASSEMBLY__  -x assembler-with-cpp -P $<

$(OUT)/bootupdater.elf: $(OUT)/bootupdater.lds $(OUT)/kbuild/built-in.a
	@echo $(ANSI_RED)"[bLD]"$(ANSI_OFF) $(@:$(T)/%=%)
	$(AT)$(LD) -o $@ \
		-T $(OUT)/bootupdater.lds \
		--whole-archive $(OUT)/kbuild/built-in.a --no-whole-archive \
		-Map $(OUT)/bootupdater.map

$(OUT)/bootupdater.bin: $(OUT)/bootupdater.elf
	@echo $(ANSI_RED)"[bX]"$(ANSI_OFF) $(@:$(T)/%=%)
	$(AT)$(OBJCOPY) -O binary $< $@
	$(AT)$(T)/firmware/tools/scripts/build_utils/add_binary_version_header.py \
		--major $(VERSION_MAJOR) \
		--minor $(VERSION_MINOR) \
		--patch $(VERSION_PATCH) \
		--version_string ARD1QRK000-$(VERSION_STRING_SUFFIX) $@ $(DEV_NULL)
	@echo $(ANSI_CYAN)"Done boot updater"$(ANSI_OFF)

.PHONY: bootupdater
bootupdater: $(OUT)/bootupdater.bin | $(OUT)

image: bootupdater

all: image

# This included file defines base targets like doc or $(OUT)/built-in.a
include $(T)/firmware/build/base.mk
