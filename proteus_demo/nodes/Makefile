
CFLAGS = $$(python-config --cflags)

default_target: all

all:clean build test

build:
	swig -python rawlasertwist.i
	$(CC) $(CFLAGS) -fpic -c rawlasertwistmodule.c rawlasertwist_wrap.c 
	$(CC) $(CFLAGS) -shared rawlasertwistmodule.o rawlasertwist_wrap.o -o _rawlasertwist.so

clean:
	rm -f *.o *.so *_wrap.c *.pyc rawlasertwist.py

test:
	python test.py

