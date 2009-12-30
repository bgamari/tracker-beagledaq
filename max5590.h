#pragma once

#include <cstdio>
#include "spi_device.h"

typedef uint16_t sample_t;

class max5590 : spi_device {
public:
	max5590(const char* dev) : spi_device(dev) { }

	/*
	 * write_outputs()
	 *
	 */
	template<int N>
	void write_outputs(std::tr1::array<int,N> channels, std::tr1::array<sample_t,N> value)
	{
		uint16_t tx[N];

		for (int i=0; i<N; i++) {
			if (channels[i] > 7)
				throw new "invalid channel"

			command cmd = (command) channels[i];
			tx[i] = build_command(cmd, value);
		}
		write(tx, NULL, N);
	}
	
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

private:
	uint16_t build_command(enum command cmd, short data);
};

