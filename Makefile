# Makefile2.6
ifneq ($(KERNELRELEASE),)
#kbuild syntax. dependency relationshsip of files and target modules are listed here.
obj-m := can-xilinx-ps7-of-platform.o can-xilinx-ps7.o 

else
PWD := $(shell pwd)
KVER ?= $(shell uname -r)
KDIR := /home/jds/linux-3.3-digilent
all:
	$(MAKE) -C $(KDIR) M=$(PWD)
clean:
	rm -rf *.o *.ko *.mod.c *.mod.o *.symvers *.order *.tmp_versions *.cmd
endif