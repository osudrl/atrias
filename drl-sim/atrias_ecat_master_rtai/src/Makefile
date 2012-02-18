# Devin Koepl

# Kbuild

CONTROLLERS=no_controller.o \
	motor_torque_controller.o \
	motor_position_controller.o \
	leg_torque_controller.o \
	leg_position_controller.o \
	leg_angle_sin_wave.o \
	raibert_controller.o \
	hubicki_controller.o \
	test_controller.o  \
        leg_force_controller.o

obj-m := rtai_controller.o 

rtai_controller-objs := rtai_controller_wrapper.o controller_wrapper_states.o control_switcher_state_machine.o $(CONTROLLERS)

EXTRA_CFLAGS := \
	-I /usr/realtime/include \
	-I $(shell rospack find atrias_controllers)/include \
	-I $(shell rospack find atrias)/include \
	-I $(shell rospack find drl_library)/include \
	-I $(shell rospack find gui)/include \
	-I /opt/etherlab/include	\
	$(shell rtai-config --module-cflags) \
	-I /usr/include \
	-ffast-math \
	-mhard-float \
	-msse \
	-D COMPILE_FOR_RTAI

KBUILD_EXTRA_SYMBOLS := \
	/usr/src/ethercat-default/Module.symvers \
	/usr/realtime/modules/Modules.symvers \
	/usr/src/linux-`uname -r`/Module.symvers

# Makefile

CC = $(shell rtai-config --cc)

LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

LINUX_DIR = $(shell rtai-config --linux-dir)

all: links rtai_controller_wrapper.c
	$(MAKE) -C $(LINUX_DIR) CC=$(CC) SUBDIRS=$$PWD V=$(V) modules

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
	ln -sf ../../atrias_controllers/src/hubicki_controller.cpp hubicki_controller.c
	ln -sf ../../atrias_controllers/src/test_controller.cpp test_controller.c
	ln -sf ../../atrias_controllers/src/leg_force_controller.cpp leg_force_controller.c

install:
	insmod rtai_controller.ko

uninstall:
	rmmod rtai_controller.ko

clean:
	$(RM) $(LINUX_DIR)/.tmp_versions/*_rt.mod *.o *.ko *.mod.c .*.cmd Module.symvers modules.order
