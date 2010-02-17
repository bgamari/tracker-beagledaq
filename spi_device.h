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
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <vector>


class spi_device {
	int fd;

protected:
	class command {
		friend class spi_device;
	protected:
		virtual unsigned int length() const = 0;
		virtual void pack(uint8_t* buf) const = 0;
		virtual void unpack(const uint8_t* buf) = 0;
	};

	spi_device(const char* dev) {
		fd = open(dev, O_RDWR);
		if (fd < 0)
			throw "can't open device";
	}

	void set_mode(uint8_t mode);
	void set_bits_per_word(uint8_t bits);
	void set_max_speed(uint32_t speed);

	void send_msg(void* tx, void* rx, int len);
	void submit(std::vector<command*> cmds);
};

