#pragma once

#include <vector>
#include <cstdint>
#include <cstring>

#include "spi_device.h"

class max1270 : spi_device {
public:
	max1270(const char* dev) : spi_device(dev) { }

	class command : spi_device::command { };

	class take_sample : public command {
		int channel;
		uint16_t& sample_out;
		unsigned int length() const { return 4; }
		void pack(uint8_t* buf) const {
			buf[0] =  (1 << 7)		// Start bit
				| (channel << 4)	// Channel
				| (0x0 << 2)		// RNG=0, BIP=0
				| (0x1 << 0);		// PD1=0, PD0=1 (External clock)
		}
		void unpack(const uint8_t* buf) {
			sample_out = 0;
			sample_out |= (buf[1] & 0xe0) >> 5;
			sample_out |= (buf[2] & 0xff) << 3;
			sample_out |= (buf[3] & 0x01) << 11;
		}
	public:
		take_sample(int channel, uint16_t& sample_out) : channel(channel), sample_out(sample_out) { }
	};

	void submit(std::vector<command*> cmds) {
		submit(cmds);
	}
};

