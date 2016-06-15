ifeq ($(CONFIG_INTEL_QRK_RTC),y)
obj-y += intel_qrk_rtc.o
obj-$(CONFIG_TCMD) += rtc_tcmd.o
endif
ifeq ($(CONFIG_INTEL_QRK_AON_PT),y)
obj-$(CONFIG_INTEL_QRK_AON_PT) += intel_qrk_aonpt.o
endif
