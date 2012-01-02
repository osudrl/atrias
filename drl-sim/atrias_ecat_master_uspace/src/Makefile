# Makefile

CONTROLLERS=no_controller.o \
    motor_torque_controller.o \
    motor_position_controller.o \
    leg_torque_controller.o \
    leg_position_controller.o \
    leg_angle_sin_wave.o \
    raibert_controller.o    \
    test_controller.o    

obj-m := uspace_controller.o 

uspace_controller-objs := uspace_controller_wrapper.o controller_wrapper_states.o control_switcher_state_machine.o $(CONTROLLERS)

INCLUDES := \
    -I $(shell rospack find atrias_controllers)/include \
    -I $(shell rospack find atrias)/include \
    -I $(shell rospack find drl_library)/include \
    -I $(shell rospack find gui)/include \
    -I /opt/etherlab/include    \
    -I /usr/include \

CFLAGS := \
    -ffast-math \
    -mhard-float \
    -msse

#KBUILD_EXTRA_SYMBOLS := \
#    /usr/src/ethercat-default/Module.symvers \
#    /usr/realtime/modules/Modules.symvers \
#    /usr/src/linux-`uname -r`/Module.symvers

#CC = $(shell rtai-config --cc)
CC = gcc

#LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
#LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

#LINUX_DIR = $(shell rtai-config --linux-dir)
#LINUX_DIR = /usr/src/linux

all: links uspace_controller_wrapper.c
	#$(MAKE) -C $(LINUX_DIR) CC=$(CC) SUBDIRS=$$PWD V=$(V) modules
	$(CC) $(INCLUDES) $(CFLAGS) uspace_controller_wrapper.c

links:
	# Create symbolic links for C++ source files.
	ln -sf ../../atrias_controllers/src/control_switcher_state_machine.cpp control_switcher_state_machine.c
	ln -sf ../../atrias_controllers/src/no_controller.cpp no_controller.c
	ln -sf ../../atrias_controllers/src/motor_torque_controller.cpp motor_torque_controller.c
	ln -sf ../../atrias_controllers/src/motor_position_controller.cpp motor_position_controller.c
	ln -sf ../../atrias_controllers/src/leg_torque_controller.cpp leg_torque_controller.c
	ln -sf ../../atrias_controllers/src/leg_position_controller.cpp leg_position_controller.c
	ln -sf ../../atrias_controllers/src/leg_angle_sin_wave.cpp leg_angle_sin_wave.c
	ln -sf ../../atrias_controllers/src/raibert_controller.cpp raibert_controller.c
	ln -sf ../../atrias_controllers/src/test_controller.cpp test_controller.c

#install:
#    insmod rtai_controller.ko
#
#uninstall:
#    rmmod rtai_controller.ko

#clean:
#    $(RM) $(LINUX_DIR)/.tmp_versions/*_rt.mod *.o *.ko *.mod.c .*.cmd Module.symvers modules.order
