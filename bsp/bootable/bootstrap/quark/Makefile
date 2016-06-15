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

########################################
# Top Level
########################################

THIS_DIR = $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
CODE_DIR = $(THIS_DIR)/src
BREAK_BEFORE_OS?=0

########################################
# Tools
########################################

CROSS_PATH?=$(T)/arduino101_firmware/external/gcc-i586-pc-elf/bin
CROSS?=i586-pc-elf-

TOOL_PREFIX=$(CROSS_PATH)/$(CROSS)
OBJCOPY=$(TOOL_PREFIX)objcopy
OBJDUMP=$(TOOL_PREFIX)objdump
SLINK=$(TOOL_PREFIX)ar
DLINK=$(TOOL_PREFIX)ld

PP=$(TOOL_PREFIX)gcc
CC=$(PP)
ASM=$(PP)

########################################
# Compile flags
########################################

ARCH=pentium
CC_FLAGS = -O0 -Wall -Wextra -c -fmessage-length=0 -mtune=$(ARCH) -march=$(ARCH)
CC_FLAGS+= -m32 -static -nodefaultlibs -nostartfiles -Wno-ignored-qualifiers
CC_FLAGS+= -mno-ms-bitfields -ffunction-sections -fdata-sections -msoft-float
ifeq ($(BREAK_BEFORE_OS), 1)
CC_FLAGS+= -DBREAK_BEFORE_OS
endif #BREAK_BEFORE_OS

ASM_FLAGS=-c -x assembler -imacros $(CODE_DIR)/fst_constants.h -m32 -march=$(ARCH) -msoft-float
PP_FLAGS=-E -x assembler-with-cpp -include $(CODE_DIR)/fst_constants.h

IMAGE_ENTRY_POINT := _ModuleEntryPoint
FW_MODULE_NAME := Flat32
OBJCOPY_ADDDEBUGFLAG := --add-gnu-debuglink=$(DEBUG_DIR)/$(MODULE_NAME).debug
DLINK_FLAGS := -nostdlib -n -q --gc-sections --script=$(CODE_DIR)/../gcc4.4-ld-script.ld --entry $(IMAGE_ENTRY_POINT) -m elf_i386 --oformat=elf32-i386
DLINK_FLAGS += $(if $(VERBOSE), --verbose ,)

OUPUT_IMAGE_SIZE_K=7

########################################
# All
########################################
all: rv f32 genbin

########################################
# Force break after the reset vector
########################################
fbarv: rv f32 genbinBreakAfterReset

########################################
# ResetVec.S16 compile
########################################
rv:
	@echo $(ANSI_RED)"[bsXX]"$(ANSI_OFF) ResetVec.dmp
	$(AT)$(PP) $(PP_FLAGS) $(INC) $(CODE_DIR)/ResetVec.S16 > ResetVec.i
	$(AT)$(ASM) -o ResetVec.obj $(ASM_FLAGS) ResetVec.i
	$(AT)$(OBJCOPY) -O binary ResetVec.obj ResetVec.com
	$(AT)$(OBJDUMP) -S ResetVec.obj > ResetVec.dmp

########################################
# Flat32.S compile
########################################
f32:
	@echo $(ANSI_RED)"[bsXX]"$(ANSI_OFF) $(FW_MODULE_NAME).com
	$(AT)$(PP) $(PP_FLAGS) $(INC) $(CODE_DIR)/Flat32.S > Flat32.i
	$(AT)$(ASM) $(ASM_FLAGS) -o Flat32.o Flat32.i
	$(AT)$(CC) $(CC_FLAGS) -o romStartup.o $(CODE_DIR)/romStartup.c
	$(AT)$(DLINK) $(DLINK_FLAGS) Flat32.o romStartup.o -o Flat32.obj
	$(AT)$(OBJCOPY) -O binary Flat32.obj $(FW_MODULE_NAME).com

########################################
# Post build, add jump to Flat32.S code
########################################
genbin:
	@echo $(ANSI_RED)"[bsAJ]"$(ANSI_OFF) $(FW_MODULE_NAME).com
	$(AT)$(CODE_DIR)/../AddJumpToFlat32S.py $(FW_MODULE_NAME) $(OUPUT_IMAGE_SIZE_K) 0 $(DEV_NULL)

genbinBreakAfterReset:
	@echo $(ANSI_RED)"[bsAJ]"$(ANSI_OFF) $(FW_MODULE_NAME).com
	$(AT)$(CODE_DIR)/../AddJumpToFlat32S.py $(FW_MODULE_NAME) $(OUPUT_IMAGE_SIZE_K) 1 $(DEV_NULL)

########################################
# clean
########################################
clean:
	-$(AT)rm *.bin *.i* *.com *.dmp *.o *.obj

