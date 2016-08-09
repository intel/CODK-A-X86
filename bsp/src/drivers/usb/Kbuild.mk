subdir-cflags-y += -I$(T)/firmware/bsp/bootable/bootloader/include/usb/
obj-$(CONFIG_USB_SHARED_INTERFACE)	+= driver_interface_wrapper.o
obj-y += function_drivers/
