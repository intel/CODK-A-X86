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

# Include this makefile in your target-specific Makefile.

# In order to work properly, your project Makefile must define the T and OUT
# variable before including this file.

# Other fixed variables
# Use a default build tag when none is set by the caller
NOWDATE         := $(shell date +"%Y%m%d%H%M%S")
BUILD_TAG       ?= custom_build_$(USER)@$(HOSTNAME)$(NOWDATE)

# Parallelism is handled in sub-makefiles which contain the CPU intensive tasks
.NOTPARALLEL:

KBUILD_OUT_DIR := $(OUT)/kbuild

KCONFIG_HEADER := $(KBUILD_OUT_DIR)/config.h

$(KCONFIG_HEADER): $(KCONFIG_FILE)
	@echo "Creating Kconfig header:" $(@:$(T)/%=%)
	$(AT)mkdir -p $(KBUILD_OUT_DIR)
	$(AT)sed $< -e 's/#.*//' > $@
	$(AT)sed -i $@ -e 's/\(CONFIG_.*\)=/#define \1 /'
	# Kconfig uses #define CONFIG_XX 1 instead of CONFIG_XX y for booleans
	$(AT)sed -i $@ -e 's/ y$$/ 1/'

# Provide rules to build the arduino101_firmware "thin" static library built-in.a
# (actually an aggregated list of links to actual object files)

# The framework library (built-in.a) is built based on:
# - the target build configugation -> KCONFIG_FILE,
# - the target build tools -> CC, AR
# - the current CFLAGS and EXTRA_BUILD_CFLAGS
$(KBUILD_OUT_DIR)/built-in.a: $(KCONFIG_HEADER) _generated_sources FORCE
	@echo "Creating framework archive:" $(OUT:$(T)/%=%)
	$(AT)$(MAKE) -C $(T)/arduino101_firmware -f $(T)/arduino101_firmware/build/Makefile.build \
		SRC=. \
		OUT=$(KBUILD_OUT_DIR) \
		KCONFIG=$(KCONFIG_FILE) \
		CFLAGS="-include $(KCONFIG_HEADER) $(CFLAGS) $(KBUILD_CFLAGS) $(EXTRA_BUILD_CFLAGS)" \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
		CC=$(CC) \
		AR=$(AR) \
		PROJECT_PATH=$(PROJECT_PATH)

_generated_sources:
	@echo "Generating source files"
	$(AT)$(MAKE) -C $(T)/arduino101_firmware -f $(T)/arduino101_firmware/build/Makefile.source \
		SRC=. \
		OUT=$(KBUILD_OUT_DIR) \
		KCONFIG=$(KCONFIG_FILE) \
		PROJECT_PATH=$(PROJECT_PATH)

.PHONY: _generated_sources

