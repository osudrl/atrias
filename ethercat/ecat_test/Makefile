all: test

test: main.cpp main.h
	g++ -I/opt/etherlab/include -o test main.cpp -L /opt/etherlab/lib -lethercat -lrt

clean:
	rm -f test

.PHONY: clean
