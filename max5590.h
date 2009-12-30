#pragma once

#include <cstdint>
#include <vector>
#include <utility>

#include "spi_device.h"


class max5590 : spi_device {
public:
	max5590(const char* dev) : spi_device(dev) { }

	/*
	 * write_outputs()
	 *
	 */
	void write_outputs(std::vector<std::pair<int, uint16_t> > channels);
	
	enum command {
		LOAD_A=0x0,
		LOAD_B,
		LOAD_C,
		LOAD_D,
		LOAD_E,
		LOAD_F,
		LOAD_G,
		LOAD_H,
		LOAD_DAC,
	};

	uint16_t build_command(enum command cmd, short data);
};

