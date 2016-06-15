ccflags-y +=-I$(srctree)/include
ccflags-y +=-I$(srctree)/include/arch/$(ARCH)
ccflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(strip $(CONFIG_PLATFORM))

obj-$(CONFIG_DW_ADC) += dw_ss_adc.o
obj-$(CONFIG_DW_ADC) += dw_adc.o
