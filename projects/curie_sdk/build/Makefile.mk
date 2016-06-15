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

# Top level makefile, repo tool should create a link on this file at the root
# of the build environement.

PROJECT_TARGETS := $(filter setup image package,$(MAKECMDGOALS))

ifneq ($(PROJECT_TARGETS),)
$(warning Usage:)
$(warning     cd <project_dir>; make $(PROJECT_TARGETS))
$(warning Or:)
$(warning     make -C <project_dir> $(PROJECT_TARGETS))
$(error [$(PROJECT_TARGETS)] must be invoked directly from the project Makefile)
endif

T   := $(CURDIR)
OUT := $(T)/out/current

# Use a default build tag when none is set by the caller
NOWDATE         := $(shell date +"%Y%m%d%H%M%S")
BUILD_TAG       ?= custom_build_$(USER)@$(HOSTNAME)$(NOWDATE)

# Host framework tests
# FIXME: should go in its own project

include $(T)/arduino101_firmware/build/verbosity.mk

# Continuous integration targets

clean:
	@echo $(ANSI_RED)"[tRM]"$(ANSI_OFF) $@
	$(AT)rm -rf $(T)/out

#### Meta-data
REF_BUILD_INFO ?= "pub/reference_build_info.json"
build_report:
	@if [ ! -e $(T)/pub/build_info-$(BUILD_TAG).json ]; then echo "No build info file found for build tag: $(BUILD_TAG). Make sure you use a consistent BUILD_TAG value while building and creating build report." ; exit 1 ; fi
	@python $(T)/arduino101_firmware/tools/scripts/build_utils/generate_build_report.py $(T)/pub/build_info-$(BUILD_TAG).json --input_ref_json $(REF_BUILD_INFO) > $(T)/pub/build_report-$(BUILD_TAG).html
	@echo Build report created in pub/build_report-$(BUILD_TAG).html
