SRC = test.c

LD_FLAGS = -L/usr/local/lib \
	 -lopencv_core \
	 -lopencv_highgui

INC_PATH = -I/usr/local/include/opencv

all: test.o
	g++ -o test test.o $(LD_FLAGS)

test.o:
	g++ -c test.c -o test.o $(INC_PATH)

clean:
	rm -fr test test.o
