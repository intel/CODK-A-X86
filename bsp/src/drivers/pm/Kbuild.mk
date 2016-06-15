obj-$(CONFIG_WAKELOCK) += wakelocks.o
obj-y += device.o
ifeq ($(CONFIG_PM_PUPDR),y)
obj-y += pm_pupdr_common.o
ifeq ($(CONFIG_QUARK_SE_QUARK),y)
subdir-cflags-y += -I$(T)/arduino101_firmware/bsp/bootable/bootloader/include/usb/
obj-y += pm_pupdr.o
obj-$(CONFIG_TCMD) += pm_pupdr_tcmd.o
endif
endif
# FIXME: Move usb_pm config in Kconfig when ready
ifeq ($(CONFIG_BOARD_QUARK_SE_APP),y)
ifeq ($(CONFIG_QUARK_SE_ARC),y)
obj-$(CONFIG_USB_PM) += usb_pm.o
endif
endif
ifeq ($(CONFIG_QUARK_SE_CURIE),y)
ifeq ($(CONFIG_QUARK_SE_QUARK),y)
obj-$(CONFIG_USB_PM) += usb_pm.o
endif
endif
obj-$(CONFIG_NORDIC_SUSPEND_BLOCKER_PM) += nordic_pm.o
