CC=g++
CFLAGS=-c -Wall -O3
LIBS=boost_system

all: test

test: test.o vc0706.o
	$(CC) test.o vc0706.o -l $(LIBS) -o test

test.o: test.cpp
	$(CC) $(CFLAGS) test.cpp -I../include

vc0706.o: ../include/vc0706.hpp ../src/vc0706.cpp
	$(CC) $(CFLAGS) ../src/vc0706.cpp -I../include

clean:
	rm -rf *.o test

