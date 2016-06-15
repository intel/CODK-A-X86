obj-$(CONFIG_CFW) += framework/
obj-y += bsp/
obj-y += devices/

subdir-cflags-y += -I$(CONFIG_MACHINE_INCLUDE_PATH)
subdir-cflags-y += $(EXTRA_CFLAGS)

# Allow extending the build tree by specifying a path where a Kbuild.mk is expected
ifneq (${CONFIG_EXTERNAL_DIRECTORIES},)
obj-y += $(subst ",,${CONFIG_EXTERNAL_DIRECTORIES})
endif

ifdef PROJECT_PATH
# Add project directory to the build tree (it shall contain a Kbuild.mk)
obj-y += $(PROJECT_PATH)/
# Also add the project root include path recursively to the CFLAGS
subdir-cflags-y += -I$(PROJECT_PATH)/include
endif

