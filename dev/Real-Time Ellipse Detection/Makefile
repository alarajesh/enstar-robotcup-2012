CC = g++
CFLAGS = -Wall -pedantic \
		 -I. \
		 -I/usr/local/include/opencv/ \
		 -I/usr/local/include/opencv2/

LDFLAGS = \
		  -L/usr/local/lib \
		  -lopencv_core \
		  -lopencv_highgui
EXEC = ellipse

all: $(EXEC)
	@echo "Compilation done."

$(EXEC): ellipse.o params.o
	$(CC) -o $@ $^ $(CFLAGS) $(LDFLAGS)

%.o:%.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	@echo "Cleaning"
	@rm -rf *.o $(EXEC)
	@rm -rf *~
