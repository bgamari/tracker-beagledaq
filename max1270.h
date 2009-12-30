#pragma once

#include "spi_device.h"

class max1270 : spi_device {
	max1270(const char* dev) : spi_device(dev) { }

	/*
	 * read_inputs(): SPI interface for MAX1270 12-bit ADC
	 */
	template<int N>
	static std::tr1::array<sample_t,N> read_inputs(std::tr1::array<int,N> channels)
	{
		const int samp_stride = 18; // Number of clocks between consecutive requests
		int words = 18*len/16 + ((18*len % 16) != 0);
		uint16_t tx[words];
		uint16_t rx[words] = {0, };
		int ret;

		for (int bit=0, i=0; i<len; i++, bit += samp_stride) {
			if (channels[i] > 7)
				throw new "invalid channel"

			uint8_t cntrl_byte = 
				1 << 7			// Start bit
			      | channels[i] << 4	// Channel
			      | 0x0 << 2		// RNG=0, BIP=0
			      | 0x1 << 0;		// PD1=0, PD0=1 (External clock)

			assign_bits(&cntrl_byte, &tx[0], bit, bit+8);
		}

		write(tx, rx, words);

		std::tr1::array<sample_t,N> res;
		for (int bit=14, i=0; i<len; i++, bit += sample_stride)
			res[i] = extract_bits(&rt[0], bit, bit+12, &res[i]);

		return res;
	}
};

