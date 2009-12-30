#include "max1270.h"
#include "bitfield.h"

/*
 * read_inputs(): SPI interface for MAX1270 12-bit ADC
 */
std::vector<uint16_t> max1270::read_inputs(std::vector<int> channels)
{
	int n = channels.size();
	const int sample_stride = 18; // Number of clocks between consecutive requests
	const int words = 18*n/16 + ((18*n % 16) != 0);

	uint16_t* tx = new uint16_t[words];
	uint16_t* rx = new uint16_t[words];
	memset(tx, 0, 2*words);
	memset(rx, 0, 2*words);
	
	for (int bit=0, i=0; i<n; i++, bit += sample_stride) {
		if (channels[i] > 7)
			throw "invalid channel";

		uint8_t cntrl_byte = 
			1 << 7			// Start bit
		      | channels[i] << 4	// Channel
		      | 0x0 << 2		// RNG=0, BIP=0
		      | 0x1 << 0;		// PD1=0, PD0=1 (External clock)

		assign_bits(&cntrl_byte, &tx[0], bit, bit+8);
	}

	message(tx, rx, words);

	std::vector<uint16_t> res(n);
	for (int bit=14, i=0; i<n; i++, bit += sample_stride)
		extract_bits(&rx[0], bit, bit+12, &res[i]);

	delete [] tx;
	delete [] rx;

	return res;
}

