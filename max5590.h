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

#include <cstdint>
#include <vector>
#include <bitset>
#include <utility>
#include <array>

#include "spi_device.h"

#define MAX5590_18V 0

class max5590 : spi_device {
public:
	max5590(const char* dev) : spi_device(dev) {
#if MAX5590_18V
		set_max_speed(10*MHZ);
#else
		set_max_speed(20*MHZ);
#endif
	}

	class command : public spi_device::command {
		void unpack(const uint8_t* buf) { }
	};

	enum input_reg { A=0x0, B, C, D, E, F, G };
	class load_input_cmd : public command {
		input_reg reg;
		uint16_t value;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = reg | (value & 0xf);
			buf[1] = (value & 0x0ff0) >> 4;
		}
	public:
		load_input_cmd(input_reg reg, uint16_t value) : reg(reg), value(value) { }
	};
	class load_dac_cmd : public command {
		std::bitset<8> dacs;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = 0x80;
			buf[1] = (uint8_t) dacs.to_ulong();
		}
	public:
		load_dac_cmd(std::bitset<8> dacs) : dacs(dacs) { }
	};
	class load_all_inputs_cmd : public command {
		uint16_t value;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = 0x90 | (value & 0xf);
			buf[1] = (value & 0x0ff0) >> 4;
		}
		void unpack(const uint8_t* buf) { }
	public:
		load_all_inputs_cmd(uint16_t value) : value(value) { }
	};
	class load_all_inputs_dacs_cmd : public command {
		uint16_t value;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = 0xa0 | (value & 0xf);
			buf[1] = (value & 0x0ff0) >> 4;
		}
		void unpack(const uint8_t* buf) { }
	public:
		load_all_inputs_dacs_cmd(uint16_t value) : value(value) { }
	};

	enum shutdown_mode {
		SHUTDOWN_1K_GND=0x0, SHUTDOWN_100K_GND=0x1, POWER_ON=0x3
	};
private:
	class set_shutdown_mode_cmd : public command {
		uint8_t cmd;
		std::array<shutdown_mode,4> modes;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = cmd;
			for (int i=0; i<4; i++)
				buf[1] = modes[0] << 0 |
					modes[1] << 2 |
					modes[2] << 4 |
					modes[3] << 6;
		}
		void unpack(const uint8_t* buf) { }
	public:
		set_shutdown_mode_cmd(uint8_t cmd, std::array<shutdown_mode,4> modes) : cmd(cmd), modes(modes) { }
	};

public:
	class set_abcd_shutdown_mode_cmd : public set_shutdown_mode_cmd {
	public:
		set_abcd_shutdown_mode_cmd(std::array<shutdown_mode,4> modes) :
			set_shutdown_mode_cmd(0xb0, modes) { }
	};
	class set_efgh_shutdown_mode_cmd : public set_shutdown_mode_cmd {
	public:
		set_efgh_shutdown_mode_cmd(std::array<shutdown_mode,4> modes) :
			set_shutdown_mode_cmd(0xb2, modes) { }
	};
	class set_shutdown_cntrl_cmd : public command {
		std::bitset<8> bits;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = 0xb4;
			buf[1] = bits.to_ulong();
		}
	public:
		set_shutdown_cntrl_cmd(std::bitset<8> bits) : bits(bits) { }
	};

	/* 
	 * settling time:
	 *    0: Slow
	 *    1: Fast
	 */
	class set_settling_time_mode_cmd : public command {
		std::bitset<8> mode;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = 0xb8;
			buf[1] = mode.to_ulong();
		}
	public:
		set_settling_time_mode_cmd(std::bitset<8> mode) : mode(mode) { }
	};
	enum upio_sel { NONE, UPIO1, UPIO2, BOTH };
	enum upio_config {
		LDAC_I=0x0,
		SET_I=0x1,
		MID_I=0x2,
		CLR_I=0x3,
		PDL_I=0x4,
		SHDN1K=0x6,
		SHDN100K=0x7,
		DOUTRB=0x8,
		DOUTDC0=0x9,
		DOUTDC1=0xa,
		GPI=0xb,
		GPOL=0xc,
		GPH=0xd,
		TOGG=0xe,
		FAST=0xf
	};
	class set_upio_config_cmd : public command {
		upio_sel sel;
		upio_config config;
		unsigned int length() const { return 2; }
		void pack(uint8_t* buf) const {
			buf[0] = 0xb6;
			buf[1] = (sel << 6) | (config << 2);
		}
	public:
		set_upio_config_cmd(upio_sel sel, upio_config config) : sel(sel), config(config) { }
	};

public:
	void submit(std::vector<command*> cmds) {
		submit(cmds);
	}
};

