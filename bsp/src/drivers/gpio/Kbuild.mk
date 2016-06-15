ifeq ($(CONFIG_GPIO),y)
obj-$(CONFIG_TCMD) += gpio_tcmd.o
endif
obj-$(CONFIG_SOC_GPIO) += soc_gpio.o
obj-$(CONFIG_SS_GPIO) += ss_gpio_iface.o

