obj-y += balloc.o
obj-y += common.o
obj-y += interrupt.o
obj-y += queue.o
obj-y += sync.o
obj-y += timer.o
CFLAGS_balloc.o += -I$(CONFIG_MEM_POOL_DEF_PATH)