CXXFLAGS = -std=gnu++0x

all : tracker

tracker : main.o spi_device.o max5590.o max1270.o
	g++ -o $@ $<

clean :
	rm *.o

