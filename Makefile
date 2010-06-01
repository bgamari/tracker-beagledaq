INCLUDES = -Ieigen
CXXFLAGS = -lrt -O2 -ggdb -std=gnu++0x -Wall ${INCLUDES} #-pg

all : tracker

tracker : main.o spi_device.o max5590.o max1270.o tracker.o pid.o
	$(CXX) $(CXXFLAGS) -o $@ $+

clean :
	rm -f *.o

% : %.o

max1270_test : spi_device.o
max5134_test : spi_device.o
max5590_test : spi_device.o
max1302_test : spi_device.o
max1302_bench : spi_device.o

tests : max1270_test max5134_test max5590_test max1302_test

# For automatic header dependencies
.deps/%.d : %
	@mkdir -p .deps
	@makedepend  ${INCLUDES} -f - $< 2>/dev/null | sed 's,\($*\.o\)[ :]*,\1 $@ : ,g' >$@

SOURCES = $(wildcard *.cpp) $(wildcard *.c)
-include $(addprefix .deps/,$(addsuffix .d,$(SOURCES)))

