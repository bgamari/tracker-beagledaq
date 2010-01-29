#pragma once

#include <cstdint>
#include <tr1/array>


struct input_data {
	std::tr1::array<uint16_t,4> psd;
	std::tr1::array<uint16_t,3> feedback;
};

struct input_channels {
	virtual input_data get() = 0;
};


struct output_data {
	std::tr1::array<uint16_t,3> stage;
};

template <int N>
struct output_channels {
	virtual void set(output_data data) = 0;
};

void track(input_channels& inputs, output_channels& outputs);

