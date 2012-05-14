
CFLAGS  = -Wall -pedantic \
		  -I. -I/usr/local/include
LDFLAGS = -L/usr/local/lib \
		  -lopencv_core \
		  -lopencv_highgui \
		  -lopencv_calib3d

all: calibration.o
	g++ $(CFLAGS) $(LDFLAGS) -o calibration $^

calibration.o: calibration.cpp
	g++ $(CFLAGS) -o $@ -c $<

clean:
	rm *.o calibration
