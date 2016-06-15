T ?= $(CURDIR)
OUT := $(T)/out/current/quark_se/cos
TOOLS_PATH ?= $(T)/arduino101_firmware/tools
COS_LDS ?= $(T)/arduino101_firmware/bsp/bootable/cos/arc.lds
QUARK_SE_DEVICE_DIR := $(T)/arduino101_firmware/devices/quark_se

ifeq ($(filter ctb crb morpheus arduino101, $(BOARD)),)
$(error The Charging OS can only run on the Curie Test/Reference Board, \
	Arduino101 and Morpheus)
endif

ARCH=arc
include $(T)/arduino101_firmware/build/Makefile.toolchain

KCONFIG_FILE ?= $(OUT)/.config
include $(KCONFIG_FILE)

ifeq ($(BUILDVARIANT),release)
CFLAGS += -DNDEBUG=1
endif

CFLAGS += -g
CFLAGS += -I$(T)/arduino101_firmware/bsp/include
CFLAGS += -I$(T)/arduino101_firmware/bsp/bootable/cos/include
CFLAGS += -D__CPU_ARC__
CFLAGS += -nostartfiles
CFLAGS += -ffreestanding -nostartfiles -fno-exceptions
CFLAGS += -fno-asynchronous-unwind-tables
CFLAGS += -fdata-sections -ffunction-sections
# See later on if we can make this code position independent
#CFLAGS += -pie

CFLAGS += -mARCv2EM -mav2em

.PHONY: FORCE


$(OUT)/cos.lds: $(COS_LDS) | $(OUT)
	$(CC) $(CFLAGS) -E -P -o $@ -ansi -D__ASSEMBLY__  -x assembler-with-cpp -P $<

$(OUT)/cos.elf: $(OUT)/cos.lds $(OUT)/kbuild/built-in.a
	@echo $(ANSI_RED)"[bLD]"$(ANSI_OFF) $(@:$(T)/%=%)
	$(AT)$(LD) -o $@ \
		-T $(OUT)/cos.lds \
		--whole-archive $(OUT)/kbuild/built-in.a --no-whole-archive \
		-Map $(OUT)/cos.map

$(OUT)/cos.bin: $(OUT)/cos.elf
	@echo $(ANSI_RED)"[bX]"$(ANSI_OFF) $(@:$(T)/%=%)
	$(AT)$(OBJCOPY) -O binary $< $@
	@echo $(ANSI_CYAN)"Done "$(ANSI_OFF)

.PHONY: cos
cos: $(OUT)/cos.bin | $(OUT)

image: cos

all: image

# This included file defines base targets like doc or $(OUT)/built-in.a
include $(T)/arduino101_firmware/build/base.mk
