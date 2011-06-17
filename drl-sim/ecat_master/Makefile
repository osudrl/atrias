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

$(TRG): $(TRG).c $(TRG).h
	$(CC) $(LXRT_CFLAGS) $(ECAT_CFLAGS) $(DRL_CFLAGS) $(TRG).c -o $(TRG) $(LXRT_LDFLAGS)

clean:
