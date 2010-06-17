INCLUDES = -Ieigen
CXXFLAGS = -O3 -ggdb -std=gnu++0x -Wall ${INCLUDES} #-pg
LDFLAGS = -lrt -lboost_program_options-mt -lboost_thread-mt -lreadline

all : tracker

tracker : main.o spi_device.o max5590.o max1270.o tracker.o pid.o parameters.o stage.o
	$(CXX) $(LDFLAGS) -o $@ $+

raster_dump : spi_device.o max5590.o max1270.o stage.o

clean :
	rm -f *.o

% : %.o

max1270_test : spi_device.o
max5134_test : spi_device.o
max5590_test : spi_device.o
max1302_test : spi_device.o
max1302_bench : spi_device.o

tests : max1270_test max5134_test max5590_test max1302_test raster_dump

# For automatic header dependencies
.deps/%.d : %
	@mkdir -p .deps
	@cpp -std=c++0x ${INCLUDES} -MM $< > $@

SOURCES = $(wildcard *.cpp) $(wildcard *.c)
-include $(addprefix .deps/,$(addsuffix .d,$(SOURCES)))

