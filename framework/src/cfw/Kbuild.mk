obj-y += cfw_debug.o
obj-y += cproxy.o
obj-$(CONFIG_CFW_CLIENT) += client_api.o
obj-$(CONFIG_CFW_SERVICE) += service_api.o
obj-$(CONFIG_CFW_MASTER) += service_manager.o
obj-$(CONFIG_CFW_PROXY) += service_manager_proxy.o
