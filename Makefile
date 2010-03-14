INCLUDES = -Ieigen
CXXFLAGS = -g3 -std=gnu++0x -Wall ${INCLUDES}

all : tracker

tracker : main.o spi_device.o max5590.o max1270.o tracker.o
	g++ -o $@ $+

clean :
	rm -f *.o

% : %.o

max1270_test : spi_device.o
max5134_test : spi_device.o
max5590_test : spi_device.o
max1302_test : spi_device.o

tests : max1270_test max5134_test max5590_test max1302_test

