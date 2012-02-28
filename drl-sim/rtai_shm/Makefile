# Devin Koepl

EXTRA_CFLAGS := \
	-I /usr/realtime/include \
	-I /opt/etherlab/include	\
	$(shell rtai-config --module-cflags) \
	-I /usr/include \
	-ffast-math \
	-mhard-float

LINUX_SOURCE_DIR=/usr/src/linux
MODULES_DIR=/lib/modules/`uname -r`

CC=$(shell rtai-config --cc)

LXRT_CFLAGS=$(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS=$(shell rtai-config --lxrt-ldflags)

obj-m := rt_shm.o 

rt_shm-objs := shm.o

all: kern

kern: shm.c
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="$(PWD)" modules

clean:
	rm -rf *.o *.mod.* Module.* modules.*
	
