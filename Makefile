CXXFLAGS = -std=gnu++0x

all : tracker

tracker : main.o max5590.o spi_device.o
	g++ -o $@ $<

