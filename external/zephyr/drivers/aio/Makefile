ccflags-y +=-I$(srctree)/arch/$(ARCH)
ccflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(subst $(DQUOTE),,$(CONFIG_PLATFORM))
asflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(subst $(DQUOTE),,$(CONFIG_PLATFORM))

obj-$(CONFIG_DW_AIO_COMPARATOR) += dw_aio_comparator.o
