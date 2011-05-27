# Devin Koepl

LINUX_SOURCE_DIR=/usr/src/linux-`uname -r`
MODULES_DIR=/lib/modules/`uname -r`

CC=$(shell rtai-config --cc)

LXRT_CFLAGS=$(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS=$(shell rtai-config --lxrt-ldflags)

MODULE_NAME=rtai_controller

all: links kern user install

kern:
	$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="$(PWD)" modules
	#$(MAKE) -C "$(LINUX_SOURCE_DIR)" M="$(MODULES_DIR)" modules

user:
	#$(CC) $(LXRT_CFLAGS) -I ../include/ -o ros_kern_interface ros_kern_interface.c $(LXRT_LDFLAGS)

install:
	modprobe -r $(MODULE_NAME)
	cp $(MODULE_NAME).ko $(MODULES_DIR)/drl
	modprobe $(MODULE_NAME)

links:
	# Create symbolic links for C++ source files.
	ln -sf ../atrias_controllers/src/control_switcher_state_machine.cpp control_switcher_state_machine.c

	ln -sf ../atrias_controllers/src/no_controller.cpp no_controller.c
	ln -sf ../atrias_controllers/src/motor_torque_controller.cpp motor_torque_controller.c
	ln -sf ../atrias_controllers/src/motor_position_controller.cpp motor_position_controller.c
	ln -sf ../atrias_controllers/src/leg_torque_controller.cpp leg_torque_controller.c
	# Christian needs to make this one ready for the module first, for now just link no controller.
	#ln -sf ../atrias/controllers/leg_position_controller.cpp leg_position_controller.c
	#ln -sf ../atrias_controllers/src/no_controller.cpp leg_position_controller.c
	ln -sf ../atrias_controllers/src/leg_angle_sin_wave.cpp leg_angle_sin_wave.c
	ln -sf ../atrias_controllers/src/raibert_controller.cpp raibert_controller.c	

clean:
	rm -rf *.o *.mod.* *.ko Module.* modules.*
	modprobe -r $(MODULE_NAME)
