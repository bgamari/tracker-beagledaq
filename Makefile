INCLUDES = -Ieigen
#PLATFORM_FLAGS = -mcpu=cortex-a8 -mfpu=neon -ftree-vectorize -mfloat-abi=softfp
PLATFORM_FLAGS = -mcpu=cortex-a8 -ftree-vectorize -mfloat-abi=softfp
CXXFLAGS = ${PLATFORM_FLAGS} -O2 -ggdb -std=gnu++0x -Wall ${INCLUDES} #-pg
LDFLAGS = -lrt -lboost_program_options -lboost_thread -lreadline

.PHONY : all
all : tracker tracker-otf

version.cpp ::
	@echo "const char* version = \"$(shell git rev-parse HEAD)\";" > version.cpp
	@echo "const char* branch = \"$(shell git name-rev HEAD | cut -d ' ' -f 2)\";" >> version.cpp

tracker : main.o hardware/spi_device.o tracker.o pid.o parameters.o stage.o version.o utils.o
	$(CXX) $(LDFLAGS) -o $@ $+

tracker-otf : main_otf.o hardware/spi_device.o otf_tracker.o pid.o parameters.o stage.o version.o utils.o
	$(CXX) $(LDFLAGS) -o $@ $+

raster_dump : spi_device.o max5590.o max1270.o stage.o

.PHONY : clean
clean :
	rm -f *.o
	rm -Rf .deps

% : %.o

hardware/tracker_dac_test : hardware/spi_device.o
hardware/tracker_adc_test : hardware/spi_device.o
hardware/tracker_adc_bench : hardware/spi_device.o

.PHONY : tests
tests : hardware/max5134_test hardware/max1302_test raster_dump

# For automatic header dependencies
.deps/%.d : %
	@mkdir -p .deps
	@cpp -std=c++0x ${INCLUDES} -MM $< > $@

SOURCES := $(wildcard *.cpp) $(wildcard *.c)
SOURCES := $(filter-out version.cpp,$(SOURCES)) # Avoid circular dependency
-include $(addprefix .deps/,$(addsuffix .d,$(SOURCES)))

