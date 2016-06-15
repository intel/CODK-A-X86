PROJECT      ?= quark_se_arduino
BOARD        ?= arduino101
ifneq ($(BOARD),arduino101)
$(error This project can only run on the Arduino101 Board)
endif
BUILDVARIANT ?= debug

QUARK_DEFCONFIG ?= $(PROJECT_PATH)/quark/defconfig
ARC_DEFCONFIG ?= $(PROJECT_PATH)/arc/defconfig

include ../curie_sdk/build/curie_common_targets.mk
