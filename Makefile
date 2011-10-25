CXX = arm-linux-gnueabi-g++
INCLUDES = -Ieigen -I. -I/usr/local/include
#PLATFORM_FLAGS = -mcpu=cortex-a8 -mfpu=neon -ftree-vectorize -mfloat-abi=softfp
PLATFORM_FLAGS = -mcpu=cortex-a8 -ftree-vectorize -mfloat-abi=softfp
CXXFLAGS = ${PLATFORM_FLAGS} -O2 -ggdb -std=gnu++0x -Wall ${INCLUDES} #-pg
LDFLAGS = -lrt -lreadline

.PHONY : all
all : tracker tracker-otf

version.cpp ::
	@echo "const char* version = \"$(shell git rev-parse HEAD)\";" > version.cpp
	@echo "const char* branch = \"$(shell git name-rev HEAD | cut -d ' ' -f 2)\";" >> version.cpp

tracker : main.o hardware/spi_device.o hardware/beagledaq.o channels.o tracker.o pid.o parameters.o stage.o version.o utils.o
	$(CXX) $+ $(LDFLAGS) -o $@

tracker-otf : main_otf.o hardware/spi_device.o hardware/beagledaq.o channels.o otf_tracker.o pid.o parameters.o stage.o version.o utils.o
	$(CXX) $+ $(LDFLAGS) -o $@

raster_dump : hardware/spi_device.o stage.o

.PHONY : clean
clean :
	rm -f *.o
	rm -Rf .deps

% : %.o

hardware/dac_test : hardware/spi_device.o
hardware/adc_test : hardware/spi_device.o
hardware/adc_bench : hardware/spi_device.o

.PHONY : tests
tests : hardware/max5134_test hardware/max1302_test raster_dump

# For automatic header dependencies
.deps/%.d : %
	@mkdir -p .deps
	@cpp -std=c++0x ${INCLUDES} -MM $< > $@

SOURCES := $(wildcard *.cpp) $(wildcard *.c)
SOURCES := $(filter-out version.cpp,$(SOURCES)) # Avoid circular dependency
-include $(addprefix .deps/,$(addsuffix .d,$(SOURCES)))

