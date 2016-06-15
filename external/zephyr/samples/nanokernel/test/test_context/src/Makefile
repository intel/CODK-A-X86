ccflags-y += ${PROJECTINCLUDE} -I${srctree}/samples/include
ccflags-y +=-I$(srctree)/arch/$(ARCH)/platforms/$(strip $(CONFIG_PLATFORM))
ccflags-$(CONFIG_PLATFORM_MINT_VALLEY) += -DFIBER_STACKSIZE=256

obj-y = context.o
