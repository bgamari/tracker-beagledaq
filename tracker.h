#pragma once

#include <vector>

struct input_channels {
	virtual std::vector<uint16_t> get() = 0;
};

struct output_channels {
	virtual void set(std::vector<uint16_t> values) = 0;
};

void track(input_channels inputs, output_channels outputs);

