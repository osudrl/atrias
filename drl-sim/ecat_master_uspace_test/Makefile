# Devin Koepl

TRG = ec_user

CC = $(shell rtai-config --cc)
LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

ECAT_CFLAGS = -I /opt/etherlab/include -L /opt/etherlab/lib $(TRG).c -l ethercat

$(TRG): $(TRG).c $(TRG).h
	$(CC) $(LXRT_CFLAGS) $(ECAT_CFLAGS) -o $(TRG) $(LXRT_LDFLAGS)

clean:
