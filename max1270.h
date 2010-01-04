#pragma once

#include <vector>
#include <cstdint>
#include <cstring>

#include "spi_device.h"

class max1270 : spi_device {
public:
	max1270(const char* dev) : spi_device(dev) { }

	std::vector<uint16_t> get(std::vector<int> channels);
};

