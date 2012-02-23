# Makefile for 2.6 kernels
# 
# Will compile and install for other kernel than the currently running,
# given the TARGET parameter (should be the name of a directory in
# /lib/modules) e.g.
# make TARGET=2.6.32.10-90.fc12.x86_64
#
ifneq ($(KERNELRELEASE),)
obj-m	:= xillybus.o

EXTRA_CFLAGS += -DCONFIG_XILLYBUS_PCIE
else
ifeq ($(TARGET),)
TARGET := $(shell uname -r)
endif
PWD := $(shell pwd)
KDIR := /lib/modules/$(TARGET)/build

default:
	@echo $(TARGET) > module.target
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	@rm -f *.ko *.o modules.order Module.symvers *.mod.? .xillybus.* *~
	@rm -rf .tmp_versions module.target

install: xillybus.ko 
	install --mode 0644 xillybus.ko /lib/modules/$(shell cat module.target)/kernel/drivers/char/
	/sbin/depmod -a $(shell cat module.target)
xillybus.ko:
	$(MAKE)
endif
