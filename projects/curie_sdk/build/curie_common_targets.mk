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

THIS_DIR    := $(shell dirname $(abspath $(lastword $(MAKEFILE_LIST))))
T           := $(abspath $(THIS_DIR)/../../../..)
OUT         := $(T)/out/current
ZEPHYR_BASE := $(T)/firmware/external/zephyr
PROJECT_PATH ?= $(CURDIR)

REF_BUILD_INFO ?= "pub/reference_build_info.json"

# Parallelism is handled in sub-makefiles which contain the CPU intensive tasks
.NOTPARALLEL:

.DEFAULT_GOAL := help

$(OUT)/firmware:
	$(AT)mkdir -p $@

# Bootloader defconfig
BOOTLOADER_ROOT ?= $(T)/firmware/bsp/bootable/bootloader
BOOTLOADER_DEFCONFIG ?= $(BOOTLOADER_ROOT)/board/intel/configs/$(BOARD)_defconfig

include $(T)/firmware/build/common_targets.mk

_project_help:

_project_setup:
	@echo "Header Version String : ARD1XXXXXX-$(VERSION_STRING_SUFFIX)"
	@echo "Configure build with the following config files:"
	@echo "  QUARK_DEFCONFIG  : $(QUARK_DEFCONFIG)"
	@echo "  ARC_DEFCONFIG       : $(ARC_DEFCONFIG)"
	@echo
	$(AT)mkdir -p $(OUT)/quark_se/quark
	$(AT)mkdir -p $(OUT)/quark_se/arc
	$(AT)mkdir -p $(OUT)/quark_se/bootupdater
	$(AT)mkdir -p $(OUT)/quark_se/cos
	@echo $(ANSI_CYAN)"Generating build configs"$(ANSI_OFF)
	$(AT)$(MAKE) quark_defconfig DEFCONFIG=$(QUARK_DEFCONFIG)
	$(AT)$(MAKE) arc_defconfig DEFCONFIG=$(ARC_DEFCONFIG)
	$(AT)$(MAKE) bootupdater_defconfig \
		DEFCONFIG=$(T)/firmware/projects/curie_sdk/bootupdater/defconfig_$(BOARD) \
		PROJECT_PATH=$(T)/firmware/bsp/bootable/bootupdater/
	$(AT)$(MAKE) cos_defconfig \
		DEFCONFIG=$(T)/firmware/projects/curie_sdk/cos/defconfig_$(BOARD) \
		PROJECT_PATH=$(T)/firmware/bsp/bootable/cos/

# FIXME: main core binaries shall go to $(OUT)/firmware/main_core
image: _check_setup_was_done $(OUT)/firmware/bootupdater.bin bootloader $(OUT)/firmware/quark.bin $(OUT)/firmware/arc.bin $(OUT)/firmware/FSRom.bin $(OUT)/firmware/quark.0.bin $(OUT)/firmware/quark.1.bin

$(OUT)/firmware/FSRom.bin: _check_setup_was_done | $(OUT)/firmware
	@echo "\n"$(ANSI_CYAN)"Building FSRom"$(ANSI_OFF)
	$(AT)mkdir -p $(OUT)/bootstrap/quark
	$(AT)$(MAKE) -f $(T)/firmware/bsp/bootable/bootstrap/quark/Makefile \
		-C $(OUT)/bootstrap/quark/ T=$(T) -j 1
	$(AT)cp $(OUT)/bootstrap/quark/FSRom.bin $(OUT)/firmware/FSRom.bin
	@echo $(ANSI_CYAN)"Done FSRom"$(ANSI_OFF)

$(OUT)/firmware/quark.elf: _check_setup_was_done .force
	@echo "\n"$(ANSI_CYAN)"Building Quark"$(ANSI_OFF)
# Generates $(OUT)/quark_se/quark/zephyr/quark.elf
	$(AT)mkdir -p $(OUT)/quark_se/quark/zephyr
	$(AT)/bin/bash -c "export T=$(T); \
		source $(ZEPHYR_BASE)/zephyr-env.sh; \
		$(MAKE) -C $(T)/firmware/projects/curie_sdk/quark_main \
			BUILDVARIANT=$(BUILDVARIANT)"
# Copy the result to $(OUT)/firmware/quark.elf
	$(AT)if [ `cat $(OUT)/BUILDVARIANT.txt` = unittests_nano ]; then \
		cp $(OUT)/quark_se/quark/zephyr/nanokernel.elf $(OUT)/firmware/quark.elf; \
	else \
		cp $(OUT)/quark_se/quark/zephyr/microkernel.elf $(OUT)/firmware/quark.elf; \
	fi

$(OUT)/firmware/arc.elf: _check_setup_was_done .force
	@echo "\n"$(ANSI_CYAN)"Building Arc"$(ANSI_OFF)
	$(AT)/bin/bash -c "export T=$(T); \
		export OUT=$(OUT); \
		source $(ZEPHYR_BASE)/zephyr-env.sh; \
		$(MAKE) -C $(T)/firmware/projects/curie_sdk/arc_main \
			BUILDVARIANT=$(BUILDVARIANT)"
	$(AT)cp $(OUT)/quark_se/arc/zephyr/nanokernel.elf $(OUT)/firmware/arc.elf

$(OUT)/firmware/arc.bin: $(OUT)/firmware/arc.elf
	$(AT)cp $(OUT)/quark_se/arc/zephyr/nanokernel.bin $@
	$(AT)$(T)/firmware/tools/scripts/build_utils/add_binary_version_header.py \
		--major $(VERSION_MAJOR) \
		--minor $(VERSION_MINOR) \
		--patch $(VERSION_PATCH) \
		--version_string ARD1ARC000-$(VERSION_STRING_SUFFIX) $@ $(DEV_NULL)
	@echo $(ANSI_CYAN)"Done Arc"$(ANSI_OFF)

# openocd does not support flashing a binary on 2 banks
# padding binary to max allowed size and split it
# to have 1 binary for each bank
# openocd does not allow flashing an empty binary
#
$(OUT)/firmware/quark.padded.bin: $(OUT)/firmware/quark.bin
	$(AT)cat $< /dev/zero | dd of=$@ bs=2k count=72 $(SILENT_DD)

$(OUT)/firmware/quark.0.bin: $(OUT)/firmware/quark.padded.bin
	$(AT)dd if=$< of=$@ bs=1 count=131072 $(SILENT_DD)

$(OUT)/firmware/quark.1.bin: $(OUT)/firmware/quark.padded.bin
	$(AT)dd if=$< of=$@ bs=1 skip=131072 $(SILENT_DD)

$(OUT)/firmware/quark.bin: $(OUT)/firmware/quark.elf
	$(AT)$(T)/firmware/external/gcc-i586-pc-elf/bin/i586-pc-elf-objcopy -O binary $(OUT)/firmware/quark.elf $@
	$(AT)$(T)/firmware/tools/scripts/build_utils/add_binary_version_header.py \
		--major $(VERSION_MAJOR) \
		--minor $(VERSION_MINOR) \
		--patch $(VERSION_PATCH) \
		--version_string ARD1QRK000-$(VERSION_STRING_SUFFIX) $@ $(DEV_NULL)
	@echo $(ANSI_CYAN)"Done Quark"$(ANSI_OFF)

$(OUT)/firmware/FSRam.bin: $(OUT)/firmware/arc.bin $(OUT)/firmware/quark.bin
	$(AT)$(T)/firmware/projects/curie_sdk/build/scripts/Create384KImage.py \
		-I $(OUT)/firmware/quark.bin -S $(OUT)/firmware/arc.bin -O $(OUT)/firmware/FSRam.bin

$(OUT)/quark_se/quark/bootloader/bootloader.bin: _check_setup_was_done $(OUT)/quark_se/cos/cos.c | $(OUT)/firmware
	@echo "\n"$(ANSI_CYAN)"Building Bootloader"$(ANSI_OFF)
	$(AT)$(MAKE) -f $(T)/firmware/bsp/bootable/bootloader/Makefile T=$(T) \
		BOOTLOADER_ROOT=$(BOOTLOADER_ROOT) \
		BOOTLOADER_DEFCONFIG=$(BOOTLOADER_DEFCONFIG) \
		OUT=$(OUT)/quark_se/quark/bootloader \
		BOARD=$(BOARD) \
		COS_BLOB=$(OUT)/quark_se/cos/cos.c \
		bootloader
	$(AT)ln -sf $(OUT)/quark_se/quark/bootloader/bootloader.bin $(OUT)/firmware/bootloader_quark.bin
	$(AT)ln -sf $(OUT)/quark_se/quark/bootloader/bootloader.elf $(OUT)/firmware/bootloader_quark.elf
	@echo $(ANSI_CYAN)"Done Bootloader"$(ANSI_OFF)

.PHONY: bootloader
bootloader: $(OUT)/quark_se/quark/bootloader/bootloader.bin

$(OUT)/quark_se/bootupdater/bootupdater.bin: _check_setup_was_done $(OUT)/quark_se/
	@echo "\n"$(ANSI_CYAN)"Building Bootupdater"$(ANSI_OFF)
	$(AT)$(MAKE) -C $(T)/firmware/bsp/bootable/bootupdater/ image \
		T=$(T) \
		OUT=$(OUT)/quark_se/bootupdater \
		VERSION_MAJOR=$(VERSION_MAJOR) \
		VERSION_MINOR=$(VERSION_MINOR) \
		VERSION_PATCH=$(VERSION_PATCH) \
		VERSION_STRING_SUFFIX=$(VERSION_STRING_SUFFIX) \
		PROJECT_PATH=$(T)/firmware/bsp/bootable/bootupdater/ \
		BOARD=$(BOARD)

$(OUT)/firmware/bootupdater.bin: $(OUT)/quark_se/bootupdater/bootupdater.bin $(OUT)/firmware
	cp $< $@

bootupdater: $(OUT)/firmware/bootupdater.bin

$(OUT)/quark_se/cos/cos.bin: _check_setup_was_done $(OUT)/quark_se/
	@echo "\n"$(ANSI_CYAN)"Building COS"$(ANSI_OFF)
	$(AT)$(MAKE) -C $(T)/firmware/bsp/bootable/cos/ image \
		T=$(T) \
		OUT=$(OUT)/quark_se/cos \
		PROJECT_PATH=$(T)/firmware/bsp/bootable/cos/ \
		BOARD=$(BOARD)

$(OUT)/quark_se/cos/cos.c: $(OUT)/quark_se/cos/cos.bin
	$(AT)cd $(OUT)/quark_se/cos/ && xxd -i cos.bin cos.c

$(OUT)/firmware/cos.bin: $(OUT)/quark_se/cos/cos.bin
	$(AT)mkdir -p $(OUT)/firmware
	$(AT)cp $< $@

cos: $(OUT)/firmware/cos.bin $(OUT)/quark_se/cos/cos.c

# Flags used to pre processor mapping headers for linker script
EXTRA_LINKERSCRIPT_CMD_OPT = -I$(T)/firmware/bsp/bootable/bootloader/include

pub/build_report: | pub
	$(AT)mkdir -p $(T)/pub/build_report

build_info: image | pub pub/build_report
	$(AT)$(T)/firmware/external/gcc-i586-pc-elf/bin/i586-pc-elf-readelf -e $(OUT)/firmware/quark.elf \
		> $(OUT)/firmware/quark.stat
	$(AT)$(T)/firmware/external/gcc-arc-elf32/bin/arc-elf32-readelf -e $(OUT)/firmware/arc.elf \
		> $(OUT)/firmware/arc.stat
	$(AT)$(T)/firmware/external/gcc-i586-pc-elf/bin/i586-pc-elf-readelf -e $(OUT)/firmware/bootloader_quark.elf \
		> $(OUT)/firmware/bootloader_quark.stat
	$(AT)PYTHONPATH="$(PYTHONPATH):$(T)/firmware/projects/curie_sdk/build/scripts/" \
		python $(T)/firmware/tools/scripts/build_utils/generate_build_info.py \
			$(OUT) $(T)/pub/build_info-$(BUILD_TAG).json \
			> $(OUT)/build_info.json
	$(AT)cat $(OUT)/build_info.json
	$(AT)python $(T)/firmware/tools/scripts/build_utils/generate_memory_report.py \
		$(OUT)/firmware/quark.elf $(OUT)/firmware/quark.bin $(T)/firmware/ $(OUT)/ \
		> $(T)/pub/build_report/$(PROJECT)-$(BOARD)-$(BUILDVARIANT)_quark_footprint_report-$(BUILD_TAG).html
	$(AT)python $(T)/firmware/tools/scripts/build_utils/generate_memory_report.py \
		$(OUT)/firmware/arc.elf $(OUT)/firmware/arc.bin $(T)/firmware/ $(OUT)/ \
		> $(T)/pub/build_report/$(PROJECT)-$(BOARD)-$(BUILDVARIANT)_arc_footprint_report-$(BUILD_TAG).html
	$(AT)python $(T)/firmware/tools/scripts/build_utils/generate_memory_report.py \
		$(OUT)/firmware/bootloader_quark.elf $(OUT)/firmware/bootloader_quark.bin $(T)/firmware/ $(OUT)/ \
		> $(T)/pub/build_report/$(PROJECT)-$(BOARD)-$(BUILDVARIANT)_bootloader_footprint_report-$(BUILD_TAG).html

%_defconfig:
	$(AT)$(MAKE) -f $(T)/firmware/build/config.mk defconfig \
		T=$(T) \
		OUT=$(OUT)/quark_se/$*/ \
		KCONFIG_ROOT=$(T)/firmware/Kconfig

