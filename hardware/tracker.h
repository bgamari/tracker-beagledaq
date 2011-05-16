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


#pragma once

#include <cstdio>
#include <vector>
#include <cstdint>
#include <cstring>
#include <bitset>

#include "channels.h"
#include "spi_device.h"

class max1302 : spi_device
{
public:
	class command : public spi_device::command { };

	class start_conversion_cmd : public command {
		int channel;
		uint16_t* sample_out;
		unsigned int length() { return 4; }
		void pack(uint8_t* buf) {
			buf[0] = (1<<7) | (channel & 0x7) << 4;
			buf[1] = 0x00;
			buf[2] = 0x00;
			buf[3] = 0x00;
		}
		void unpack(const uint8_t* buf) {
			*sample_out = (buf[2] << 8) | (buf[3] << 0);
		}
	public:
		start_conversion_cmd(int channel, uint16_t* sample_out) :
			channel(channel), sample_out(sample_out) { }
	};

	enum input_range {
		SE_NO_CHANGE = 0x0,
		SE_MINUS_VREF4_PLUS_VREF4 = 0x1,
		SE_MINUS_VREF2_ZERO = 0x2,
		SE_ZERO_PLUS_VREF2 = 0x3,
		SE_MINUS_VREF2_PLUS_VREF2 = 0x4,
		SE_MINUS_VREF_ZERO = 0x5,
		SE_ZERO_PLUS_VREF = 0x6,
		SE_MINUS_VREF_PLUS_VREF = 0x7,		/* Default */
		DIFF_NO_CHANGE = 0xa,
		DIFF_MINUS_VREF2_PLUS_VREF2 = 0xb,
		DIFF_MINUS_VREF_PLUS_VREF = 0xc,
		DIFF_MINUS_2_VREF_PLUS_2VREF = 0xf 
	};

	class input_config_cmd : public command {
		int channel;
		input_range range;
		unsigned int length() { return 2; }
		void pack(uint8_t* buf) {
			buf[0] = (1<<7) | (channel & 0x7) << 4 | range;
			buf[1] = 0x00;
		}
		void unpack(const uint8_t* buf) { }
	public:
		input_config_cmd(int channel, input_range range) :
			channel(channel), range(range) { }
	};

	enum device_mode {
		EXT_CLOCK	= 0x0,
		EXT_ACQ 	= 0x1,
		INT_CLOCK 	= 0x2,
		RESET		= 0x4,
		PARTIAL_PWR_DOWN= 0x6,
		FULL_PWD_DOWN	= 0x7,
	};

	class mode_control_cmd : public command {
		device_mode mode;
		unsigned int length() { return 2; }
		void pack(uint8_t* buf) {
			buf[0] = (1<<7) | (mode<<4) | (1<<3);
			buf[1] = 0x00;
		}
		void unpack(const uint8_t* buf) { }
	public:
		mode_control_cmd(device_mode mode) : mode(mode) { }
	};

	max1302(const char* dev) : spi_device(dev) {
		set_max_speed(1*MHZ);
	}

	void submit(std::vector<command*>& cmds) {
		spi_device::submit(cmds);
	}
};

template<unsigned int N>
struct max1302_inputs : input_channels<N> {
	max1302& dev;
	array<int,N> channels;
	// Normalization parameters:
	float scale; // Full-scale range, in units of Vref
	float offset; // Offset, in units of Vref

	max1302_inputs(max1302& dev, const array<int,N> channels,
			max1302::input_range range) :
		dev(dev), channels(channels)
	{
		std::vector<max1302::command*> cmds;
		
		switch (range) {
		case max1302::SE_ZERO_PLUS_VREF:
			offset = 0; scale = 1; break;
		case max1302::SE_MINUS_VREF_PLUS_VREF:
			offset = -1; scale = 2; break;
		case max1302::SE_ZERO_PLUS_VREF2:
			offset = 0; scale = 0.5; break;
		default:
			fprintf(stderr, "Unknown range\n");
			abort();
		}
		
		for (unsigned int i=0; i<N; i++) {
			cmds.push_back(new max1302::mode_control_cmd(max1302::EXT_CLOCK));
			cmds.push_back(new max1302::input_config_cmd(channels[i], range));
		}
		dev.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}

	Matrix<float,1,N> get() const
	{
		array<uint16_t,N> int_vals;
		Matrix<float,1,N> values;
		std::vector<max1302::command*> cmds;
		cmds.reserve(N);

		for (unsigned int i=0; i<N; i++)
			cmds.push_back(new max1302::start_conversion_cmd(channels[i], &int_vals[i]));
		dev.submit(cmds);
		for (unsigned int i=0; i<N; i++) {
			int_vals[i] *= 2; // HACK: Hardware seems to be off by factor of two
			values[i] = scale * int_vals[i] / 0xffff + offset;
			delete cmds[i];
		}
		return values;
	}
};

class max5134 : spi_device {
public:
	typedef std::bitset<4> chan_mask;

	max5134(const char* dev) : spi_device(dev) {
		set_max_speed(10*MHZ);
	}

	struct command : spi_device::command {
		unsigned int length() { return 3; }
		void unpack(const uint8_t* buf) { }
	};

	class noop_cmd : public command {
		void pack(uint8_t* buf) {
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
		}
	public:
		noop_cmd() { }
	};

	class load_dac_cmd : public command {
		chan_mask dacs;
		void pack(uint8_t* buf) {
			buf[0] = 0x01;
			buf[1] = (uint8_t) dacs.to_ulong();
			buf[2] = 0x00;
		}
	public:
		load_dac_cmd(chan_mask dacs) : dacs(dacs) { }
	};

	class clear_cmd : public command {
		void pack(uint8_t* buf) {
			buf[0] = 0x02;
			buf[1] = 0x00;
			buf[2] = 0x00;
		}
	public:
		clear_cmd() { }
	};

	class pwr_cntrl_cmd : public command {
		chan_mask dacs;
		bool ready_en;
		void pack(uint8_t* buf) {
			buf[0] = 0x03;
			buf[1] = (uint8_t) dacs.to_ulong();
			buf[2] = ready_en ? (1<<7) : 0x00;
		}
	public:
		pwr_cntrl_cmd(chan_mask dacs, bool ready_en) : dacs(dacs), ready_en(ready_en) { }
	};

	class linearity_cmd : public command {
		bool lin;
		void pack(uint8_t* buf) {
			buf[0] = 0x05;
			buf[1] = 0x00;
			buf[2] = lin ? (1<<1) : 0x00;
		}
	public:
		linearity_cmd(bool lin) : lin(lin) { }
	};

	class write_cmd : public command {
		chan_mask dacs;
		uint16_t value;
		void pack(uint8_t* buf) {
			buf[0] = (0x01 << 4) | ((uint8_t) dacs.to_ulong());
			buf[1] = (value >> 8) & 0xff;
			buf[2] = (value >> 0) & 0xff;
		}
	public:
		write_cmd(chan_mask dacs, uint16_t value) : dacs(dacs), value(value) { }
	};


	class write_thru_cmd : public command {
		chan_mask dacs;
		uint16_t value;
		void pack(uint8_t* buf) {
			buf[0] = (0x03 << 4) | ((uint8_t) dacs.to_ulong());
			buf[1] = (value >> 8) & 0xff;
			buf[2] = (value >> 0) & 0xff;
		}
	public:
		write_thru_cmd(chan_mask dacs, uint16_t value) : dacs(dacs), value(value) { }
	};

	void submit(std::vector<command*>& cmds) {
		spi_device::submit(cmds);
	}
};

template<unsigned int N>
struct max5134_outputs : output_channels<N> {
	max5134& dev;
	array<max5134::chan_mask,N> channels;

	max5134_outputs(max5134& dev, const array<max5134::chan_mask,N> channels) :
		dev(dev), channels(channels) { }

	void set(const Matrix<float,1,N> values) const
	{
		max5134::chan_mask all_mask;
		std::vector<max5134::command*> cmds;
		cmds.reserve(N+1);
		for (unsigned int i=0; i<N; i++) {
			if (values[i] < 0.0 || values[i] > 1.0)
				throw clamped_output_error(values);
			uint16_t out_val = lrint(values[i]*0xffff);
			cmds.push_back(new max5134::write_cmd(channels[i], out_val));
			all_mask |= channels[i];
		}
		cmds.push_back(new max5134::load_dac_cmd (all_mask));

		dev.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}
};

