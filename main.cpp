#include <stdint.h>
#include "bitfield.h"
#include "max5590.h"
#include "max1270.h"

static const char* in_dev = "/dev/spidev1.0";
static const char* out_dev = "/dev/spidev1.1";

static uint16_t delay = 0;
static uint8_t mode = 0;
static uint8_t bits = 16;
static uint32_t speed = 500000;


struct max1270_inputs {
	max1270& adc;
	std::vector<int> channels;

	max1270_inputs(max1270& adc, std::vector<int> channels) :
		adc(adc), channels(channels) { }

	std::vector<uint16_t> get() {
		adc.get(channels);
	}
};

struct max5590_outputs {
	max5590& dac;
	std::vector<int> channels;

	max5590_outputs(max5590& dac) : dac(dac), channels(channels) { }

	void set(std::vector<uint16_t> values) {
		std::vector<std::pair<int, uint16_t> > v;
		for (int i=0; i<channels.size(); i++) {
			std::pair<int, uint16_t> p(channels[i], values[i]);
			v.push_back(p);
		}
		dac.set(v);
	}
};

int main(int argc, char** argv)
{
}

