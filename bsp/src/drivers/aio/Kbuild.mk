ifeq ($(CONFIG_SOC_COMPARATOR),y)
obj-y += soc_comparator.o
obj-$(CONFIG_TCMD_COMPARATOR) += comparator_tcmd.o
endif
