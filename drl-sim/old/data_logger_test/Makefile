# Devin Koepl

LINUX_SOURCE_DIR=/usr/src/linux-`uname -r`
MODULES_DIR=/lib/modules/`uname -r`

CC=$(shell rtai-config --cc)

LXRT_CFLAGS=$(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS=$(shell rtai-config --lxrt-ldflags)

MODULE_NAME=rtai_controller

all: kern uspace install

kern: $(MODULE_NAME)_wrapper.h $(MODULE_NAME)_wrapper.c
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="$(PWD)" modules

uspace: uspace.c
	$(CC) $(LXRT_CFLAGS) -o uspace uspace.c $(LXRT_LDFLAGS)

install:
	cp $(MODULE_NAME).ko $(MODULES_DIR)/drl
	modprobe -r $(MODULE_NAME)
	modprobe $(MODULE_NAME)	

clean:
	rm -rf *.o *.mod.* *.ko Module.* modules.*
	modprobe -r $(MODULE_NAME)
