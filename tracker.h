#pragma once

#include <cstdint>
#include <tr1/array>

template <int N>
struct input_channels {
	virtual std::tr1::array<uint16_t,N> get() = 0;
};

template <int N>
struct output_channels {
	virtual void set(std::tr1::array<uint16_t,N> values) = 0;
};

template <int n_in, int n_out>
void track(input_channels<n_in>& inputs, output_channels& outputs);

