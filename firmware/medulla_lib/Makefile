all:
	make -C src

clean:
	make clean -C src

clean-docs:
	rm -r doc/html

clean-all: clean clean-docs

program:
	make program -C src

docs:
	doxygen
