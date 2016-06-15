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

# =============================================================================
# Configure a target in the specified source directory
# Usage:
# 	make -f Makefile.config T=<T> OUT=<OUT> \
# 	                        KCONFIG_ROOT=<KCONFIG_ROOT> \
# 	                        DEFCONFIG=<DEFCONFIG>
# Where:
#  <T>   is the root of the project source tree
#  <OUT> is the path to the Kconfig output directory
#  <KCONFIG_ROOT> is the path to the root Kconfig file
#  <DEFCONFIG> is the path to the default configuration file (relevant only for
#  savedefconfig and defconfig targets)
# =============================================================================

T ?= $(CURDIR)

OUT ?= $(CURDIR)/out

ifndef KCONFIG_ROOT
$(error No Kconfig root file specified)
endif

# include rules to build kconfig-frontends host tools
include $(T)/arduino101_firmware/build/host/kconfig.mk

# Kconfig conf output files
KCONFIG_CONFIG=$(OUT)/.config
KCONFIG_AUTOCONFIG := $(OUT)/auto.conf
KCONFIG_AUTOHEADER := $(OUT)/config.h
KCONFIG_TRISTATE := $(OUT)/tristate.config

# The environment to export before running Kconfig tools,
# allowing to control their output
KCONFIG_ENV := \
	KCONFIG_CONFIG=$(KCONFIG_CONFIG) \
	KCONFIG_AUTOCONFIG=$(KCONFIG_AUTOCONFIG) \
	KCONFIG_AUTOHEADER=$(KCONFIG_AUTOHEADER) \
	KCONFIG_TRISTATE=$(KCONFIG_TRISTATE)

# Generate configuration headers from .config file
config: $(KCONFIG_CONFIG) $(KCONFIG_CONF)
	@echo "Checking and expanding build configuration"
	$(AT)(cd $(T)/arduino101_firmware && \
	$(KCONFIG_ENV) $(KCONFIG_CONF) --silentoldconfig $(KCONFIG_ROOT))

# Interactively select configuration items to generate .config file
menuconfig: $(KCONFIG_MCONF)
	(cd $(T)/arduino101_firmware && \
	$(KCONFIG_ENV) $(KCONFIG_MCONF) $(KCONFIG_ROOT))

# Save a configuration to the specified DEFCONFIG
savedefconfig: $(KCONFIG_CONF)
	$(AT)(cd $(T)/arduino101_firmware && \
	$(KCONFIG_ENV) $(KCONFIG_CONF) \
	--savedefconfig=$(if $(DEFCONFIG),$(DEFCONFIG),$(OUT)/defconfig) \
	$(KCONFIG_ROOT))

# Load a configuration from the specified DEFCONFIG
defconfig: $(KCONFIG_CONF)
	$(if $(DEFCONFIG),,@echo "Error: DEFCONFIG must be set"; false)
	$(AT)(cd $(T)/arduino101_firmware && \
	$(KCONFIG_ENV) $(KCONFIG_CONF) \
	--defconfig$(if $(DEFCONFIG),=$(DEFCONFIG)) $(KCONFIG_ROOT))

$(KCONFIG_CONFIG): defconfig $(DEFCONFIG)

$(KCONFIG_AUTOHEADER): config $(KCONFIG_CONFIG)
