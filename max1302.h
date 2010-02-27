/* tracker - Back-focal plane droplet tracker
 *
 * Copyright © 2010 Ben Gamari
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/ .
 *
 * Author: Ben Gamari <bgamari@physics.umass.edu>
 */


#include "spi_device.h"

class max1302 : spi_device
{
public:
	class start_conversion_cmd : public spi_device::command {
		int channel;
		uint16_t& sample_out;
		unsigned int length() const { return 4; }
		void pack(uint8_t* buf) const {
			buf[0] = (1<<7) | (channel & 0x7) << 4;
			buf[1] = 0x00;
			buf[2] = 0x00;
			buf[2] = 0x00;
		}
		void unpack(uint8_t* buf) const {
			sample_out = (buf[0] << 8) & (buf[1] << 0);
		}
	public:
		start_conversion_cmd(int channel, uint16_t& sample_out) :
			channel(channel), sample_out(sample_out) { }
	};

	enum input_range {
		SE_NO_CHANGE = 0x0,
		SE_MINUS_VREF4_PLUS_VREF4 = 0x1,
		SE_MINUS_VREF2_ZERO = 0x2,
		SE_ZERO_MINUS_VREF2 = 0x3,
		SE_MINUS_VREF2_PLUS_VREF2 = 0x4,
		SE_MINUS_VERF2_ZERO = 0x5,
		SE_ZERO_PLUS_VREF = 0x6,
		SE_MINUS_VREF_PLUS_VREF = 0x7,
		DIFF_NO_CHANGE = 0xa,
		DIFF_MINUS_VREF2_PLUS_VREF2 = 0xb,
		DIFF_MINUS_VREF_PLUS_VREF = 0xc,
		DIFF_MINUS_2_VREF_PLUS_2VREF = 0xf 
	};

	class mode_cntrl_cmd : public spi_device::command {
		int channel;
		input_range range;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = (1<<7) | (channel & 0x7) << 4 | range;
			buf[1] = 0x00;
		}
		void unpack(uint8_t* buf) const { }
	public:
		mode_cntrl_cmd(int channel, input_range range) :
			channel(channel), range(range) { }
	};


	enum input_mode {
		EXT_CLOCK	= 0x0,
		EXT_ACQ 	= 0x1,
		INT_CLOCK 	= 0x2,
		RESET		= 0x4,
		PARTIAL_PWR_DOWN= 0x6,
		FULL_PWD_DOWN	= 0x7,
	};

	class input_config_cmd : public spi_device::command {
		int channel;
		input_mode mode;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = (1<<7) | (mode<<4) | (1<<3);
			buf[1] = 0x00;
		}
		void unpack(uint8_t* buf) const { }
	public:
		input_config_cmd(int channel, input_mode mode) :
			channel(channel), mode(mode) { }
	};


	max1302(const char* dev) : spi_device(dev) { }

	void submit(std::vector<command*> cmds) {
		submit(cmds);
	}
};

