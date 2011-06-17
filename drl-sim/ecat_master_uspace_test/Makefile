# Devin Koepl

CC = $(shell rtai-config --cc)
LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

ec_user: ec_user.c
	$(CC) $(LXRT_CFLAGS) -I /opt/etherlab/include -L /opt/etherlab/lib ec_user.c -l ethercat -o ec_user $(LXRT_LDFLAGS)

clean:
