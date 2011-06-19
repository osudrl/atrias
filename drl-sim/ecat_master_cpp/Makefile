# Devin Koepl

TRG = ec_user

#CC = $(shell rtai-config --cc)
CC = g++
LXRT_CFLAGS = $(shell rtai-config --lxrt-cflags)
LXRT_LDFLAGS = $(shell rtai-config --lxrt-ldflags)

ECAT_CFLAGS = -I /opt/etherlab/include -L /opt/etherlab/lib -l ethercat

$(TRG): $(TRG).cpp $(TRG).h
	$(CC) $(LXRT_CFLAGS) $(ECAT_CFLAGS) $(TRG).cpp -o $(TRG) $(LXRT_LDFLAGS)

clean:
