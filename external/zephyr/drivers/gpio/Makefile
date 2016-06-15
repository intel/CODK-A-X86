ccflags-y +=-I$(srctree)/drivers
ccflags-y +=-I$(srctree)/include/drivers
ccflags-y +=-I$(srctree)/arch/$(ARCH)/$(subst $(DQUOTE),,$(CONFIG_PLATFORM))
ccflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(subst $(DQUOTE),,$(CONFIG_PLATFORM))

obj-$(CONFIG_GPIO_DW) = gpio-dw.o

