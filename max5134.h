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

#include <vector>
#include <cstdint>
#include <cstring>

#include "spi_device.h"

class max5134 : spi_device {
public:
	max5134(const char* dev) : spi_device(dev) { }

	class command : spi_device::command {
		unsigned int length() const { return 3; }
		void unpack(uint8_t* buf) { }
	};

	class noop_cmd : command {
		void pack(uint8_t* buf) const {
			buf[0] = 0x00;
			buf[1] = 0x00;
			buf[2] = 0x00;
		}
	public:
		noop_cmd() { }
	};

	class load_dac_cmd : command {
		std::bitset<4> dacs;
		void pack(uint8_t* buf) const {
			buf[0] = 0x01;
			buf[1] = (uint8_t) dacs.to_ulong();
			buf[2] = 0x00;
		}
	public:
		load_dac_cmd(std::bitset<4> dacs) : dacs(dacs) { }
	};

	class clear_cmd : command {
		void pack(uint8_t* buf) const {
			buf[0] = 0x02;
			buf[1] = 0x00;
			buf[2] = 0x00;
		}
	public:
		clear_cmd() { }
	};

	class pwr_cntrl_cmd : command {
		std::bitset<4> dacs;
		bool ready_en;
		void pack(uint8_t* buf) const {
			buf[0] = 0x03;
			buf[1] = (uint8_t) dac_regs.to_ulong();
			buf[2] = ready_en ? (1<<7) : 0x00;
		}
	public:
		pwr_cntrl_cmd(std::bitset<4> dacs, bool ready_en) : dacs(dacs), ready_en(ready_en) { }
	};

	class linearity_cmd : command {
		bool lin;
		void pack(uint8_t* buf) const {
			buf[0] = 0x05;
			buf[1] = 0x00;
			buf[2] = lin ? (1<<1) : 0x00;
		}
	public:
		linearity_cmd(bool lin) : lin(lin) { }
	};

	class write_cmd : command {
		std::bitset<4> dacs;
		uint16_t value;
		void pack(uint8_t* buf) const {
			buf[0] = (0x01 << 4) | ((uint8_t) dacs.to_ulong());
			buf[1] = (value >> 8) & 0xff;
			buf[2] = (value >> 0) & 0xff;
		}
	public:
		write_cmd(std::bitset<4> dacs, uint16_t value) : dacs(dacs), valud(value) { }
	};


	class write_thru_cmd : command {
		std::bitset<4> dacs;
		uint16_t value;
		void pack(uint8_t* buf) const {
			buf[0] = (0x03 << 4) | ((uint8_t) dacs.to_ulong());
			buf[1] = (value >> 8) & 0xff;
			buf[2] = (value >> 0) & 0xff;
		}
	public:
		write_thru_cmd(std::bitset<4> dacs, uint16_t value) : dacs(dacs), value(value) { }
	};

	void submit(std::vector<command*> cmds) {
		submit(cmds, true);
	}
};

