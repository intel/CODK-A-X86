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

# Content to be included in a project's top-level Makefile.mk
# This file defines shared variables and targets for:
# - creating doxygen documentation
# - versioning the binary images
# - dealing with multiple build configurations

# In order to work properly, your top-level project Makefile.mk must define the
# following variables:
# - T: top level source directory
# - OUT: build directory
# - BUILDVARIANT: a build variant name like debug, release, unittests etc..
# - BOARD: a small case board name like 'ctb', 'testboard' etc.. Use 'all_boards'
#          if the project has a single board
# - PROJECT: the mall case, short name for the project, e.g. 'quark_se'
# Furthermore, you may want to override all variables set with ?= to customize
# the build behaviour.

ifndef T
$(error No top level source directory T specified)
endif

ifndef OUT
$(error No output directory (OUT) specified)
endif

ifeq ($(findstring setup,$(MAKECMDGOALS)),setup)

# Some build parameters MUST be specified during setup

ifndef BUILDVARIANT
$(error No build variant (BUILDVARIANT) specified)
endif

ifndef BOARD
$(error No board (BOARD) specified)
endif

ifndef PROJECT
$(error No project short name (PROJECT) specified)
endif

else

# Restore build parameters specified during setup
-include $(OUT)/build_setup.mk

endif # Setup

ifdef PROJECT_PATH
export PROJECT_PATH
endif

# The top-level directory (the one containing the build/ directory where this file is)
TDOME_ROOT      ?= $(T)/arduino101_firmware

#### Versioning, setup and build info
# Use a default build tag when none is set by the caller
NOWDATE     := $(shell date +"%Y%m%d%H%M%S")
BUILD_TAG   ?= custom_build_$(USER)@$(HOSTNAME)$(NOWDATE)

# Set to 1.0.0 when PV is reached
VERSION_MAJOR  ?= 0
VERSION_MINOR  ?= 0
VERSION_PATCH  ?= 0

# Get build number from environment or generate from YYWW
ifeq ($(BUILD_NUMBER),)
BUILD_NUMBER_PADDED := $(shell date +"%M%S")
else
BUILD_NUMBER_PADDED := $(shell printf "%04d" $(BUILD_NUMBER))
endif
BUILD_NUMBER_TRUNCATED = $(shell printf "%.4s" $(BUILD_NUMBER_PADDED))

# If BUILD_TYPE is defined, use its first letter
ifneq ($(BUILD_TYPE),)
BUILD_LETTER = $(shell printf "%c" $(BUILD_TYPE) | tr a-z A-Z)
else
# Custom Build
BUILD_LETTER = C
BUILD_TYPE = custom
endif

# By default use the following:
# Year: %g, Workweek: %V, Type: C (=Custom build), BuildNumber: %M%S
VERSION_STRING_SUFFIX ?= $(shell date +"%g%V")$(BUILD_LETTER)$(BUILD_NUMBER_TRUNCATED)

ifeq ($(ARTIFACTORY_BUILD_URL),)
DOWNLOAD_DIR_URL := file://$(T)/pub
else
DOWNLOAD_DIR_URL := $(ARTIFACTORY_BUILD_URL)
endif

#### Declare global variables for pretty output
include $(TDOME_ROOT)/build/verbosity.mk

pub:
	$(AT)mkdir -p $(T)/$@

.PHONY: setup clean list .force setup

setup: | pub
	@echo "Setup with the following options:"
	@echo "   PROJECT       : $(PROJECT)"
	@echo "   BOARD         : $(BOARD)"
	@echo "   BUILDVARIANT  : $(BUILDVARIANT)"
	@echo "   BUILD_TAG     : $(BUILD_TAG)"
	@echo "   BUILD_TYPE    : $(BUILD_TYPE)"
	@echo "   WORKWEEK      : $(WORKWEEK)"
	@echo "   BUILDNUMBER   : $(BUILDNUMBER)"
	@echo
	@echo Setup buildenv in out/$(PROJECT)_$(BOARD)_$(BUILDVARIANT)
	$(AT)mkdir -p $(T)/out/$(PROJECT)_$(BOARD)_$(BUILDVARIANT)
	$(AT)rm -f $(OUT)
	$(AT)ln -s $(T)/out/$(PROJECT)_$(BOARD)_$(BUILDVARIANT) $(OUT)
	@echo "$(PROJECT)-$(BOARD)-$(BUILDVARIANT)-$(BUILD_TAG)" > $(OUT)/package_prefix.txt
	@echo "$(BUILDVARIANT)" > $(OUT)/BUILDVARIANT.txt
	@echo "{\"PROJECT\": \"$(PROJECT)\", \"BOARD\": \"$(BOARD)\", \"BUILDVARIANT\": \"$(BUILDVARIANT)\", \"BUILD_TAG\": \"$(BUILD_TAG)\", \"BUILD_TYPE\": \"$(BUILD_TYPE)\", \"DOWNLOAD_DIR_URL\": \"$(DOWNLOAD_DIR_URL)\"}" > $(OUT)/build_setup.json
	@echo "PROJECT_PATH=$(PROJECT_PATH)" > $(OUT)/build_setup.mk
	@echo "PROJECT=$(PROJECT)" >> $(OUT)/build_setup.mk
	@echo "BOARD=$(BOARD)" >> $(OUT)/build_setup.mk
	@echo "BUILDVARIANT=$(BUILDVARIANT)" >> $(OUT)/build_setup.mk
	$(AT)$(MAKE) _project_setup

_check_setup_was_done:
	@if [ ! -f $(OUT)/package_prefix.txt ]; then echo Please run \"make setup\" first ; exit 1 ; fi

_no_targets:
list:
	$(AT)sh -c "$(MAKE) -p _no_targets | awk -F':' '/^[a-zA-Z0-9][^\$$#\/\\t=]*:([^=]|$$)/ {split(\$$1,A,/ /);for(i in A)print A[i]}' | sort"

clean:
	@echo $(ANSI_RED)"[tRM]"$(ANSI_OFF) $@
	$(AT)rm -rf $(T)/out

#### host rules
include $(TDOME_ROOT)/build/host/check.mk

#### help rules
include $(TDOME_ROOT)/build/help.mk
