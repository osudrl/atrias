# Devin Koepl

TRG = ec_control_wrapper

CC = `rtai-config --cc`
LXRT_CFLAGS = `rtai-config --lxrt-cflags`
LXRT_LDFLAGS = `rtai-config --lxrt-ldflags`

ECAT_CFLAGS = -I /opt/etherlab/include \
	-L /opt/etherlab/lib \
	-l ethercat

DRL_CFLAGS = -I `rospack find atrias`/include \
	-I `rospack find atrias_controllers`/include \
	-I `rospack find drl_library`/include

CFLAGS = $(LXRT_CFLAGS) $(ECAT_CFLAGS) $(DRL_CFLAGS) -l m

$(TRG): $(TRG).o control_wrapper_states.o
	$(CC) $(CFLAGS) -o $(TRG) control_wrapper_states.o  $(TRG).o $(LXRT_LDFLAGS)

$(TRG).o: $(TRG).c $(TRG).h
	$(CC) $(CFLAGS) -o $(TRG).o -c $(TRG).c

control_wrapper_states.o: control_wrapper_states.c control_wrapper_states.h
	$(CC) $(CFLAGS) -o control_wrapper_states.o -c control_wrapper_states.c

#control_switcher_state_machine.o: control_switcher_state_machine.c
#	$(CC) $(DRL_CFLAGS) -o control_switcher_state_machine.o

clean:
	rm -rf *.o
