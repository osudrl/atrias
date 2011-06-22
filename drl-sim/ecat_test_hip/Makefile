# Devin Koepl

# Kbuild	

obj-m := rtai_controller.o 

rtai_controller-objs := rtai_controller_wrapper.o

EXTRA_CFLAGS := \
	-I /usr/realtime/include \
	-I /home/drl/atrias/drl-sim/atrias/include \
	-I /opt/etherlab/include	\
	$(shell rtai-config --module-cflags) \
	-I /usr/include \
	-ffast-math \
	-mhard-float \
	-D COMPILE_FOR_RTAI
#-I /home/drl/atrias/drl-sim/atrias_controllers/include \
#-I /home/drl/atrias/drl-sim/drl_library/include \
#-I /home/drl/atrias/drl-sim/gui/include \

KBUILD_EXTRA_SYMBOLS := \
	/usr/src/ethercat-default/Module.symvers \
	/usr/realtime/modules/Modules.symvers \
	/usr/src/linux-`uname -r`/Module.symvers

# Makefile

CC = $(shell rtai-config --cc)

LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

LINUX_DIR = $(shell rtai-config --linux-dir)

all: rtai_controller_wrapper.c
	$(MAKE) -C $(LINUX_DIR) CC=$(CC) SUBDIRS=$$PWD V=$(V) modules

install:
	insmod rtai_controller.ko

uninstall:
	rmmod rtai_controller.ko

clean:
	$(RM) $(LINUX_DIR)/.tmp_versions/*_rt.mod *.o *.ko *.mod.c .*.cmd Module.symvers modules.order
