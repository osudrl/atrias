# Devin Koepl

LINUX_SOURCE_DIR=/usr/src/linux-`uname -r`
MODULES_DIR=/lib/modules/`uname -r`

CC=$(shell rtai-config --cc)

LXRT_CFLAGS=$(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS=$(shell rtai-config --lxrt-ldflags)

MODULE_NAME=rtai_controller

all: kern install

kern:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="$(PWD)" modules
	#$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="$(MODULES_DIR)" modules

install:
	modprobe -r $(MODULE_NAME)
	cp $(MODULE_NAME).ko $(MODULES_DIR)/drl
	modprobe $(MODULE_NAME)	

clean:
	rm -rf *.o *.mod.* *.ko Module.* modules.*
	modprobe -r $(MODULE_NAME)
