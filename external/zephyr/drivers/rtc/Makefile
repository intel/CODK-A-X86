ccflags-y +=-I$(srctree)/include/drivers
ccflags-y +=-I$(srctree)/arch/$(ARCH)
ccflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(subst $(DQUOTE),,$(CONFIG_PLATFORM))
asflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(subst $(DQUOTE),,$(CONFIG_PLATFORM))

obj-$(CONFIG_DW_RTC) += dw_rtc.o
