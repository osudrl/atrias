all: test

test: main.cpp
	g++ -I/opt/etherlab/include -o test main.cpp -L /opt/etherlab/lib -lethercat

clean:
	rm -f test

.PHONY: clean
