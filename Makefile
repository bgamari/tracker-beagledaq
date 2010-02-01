INCLUDES = -Ieigen
CXXFLAGS = -g3 -std=gnu++0x -Wall ${INCLUDES}

all : tracker

tracker : main.o spi_device.o max5590.o max1270.o tracker.o
	g++ -o $@ $+

clean :
	rm *.o

