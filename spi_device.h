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

public:
	class command {
		friend class spi_device;
	protected:
		virtual unsigned int length() const = 0;
		virtual void pack(uint8_t* buf) const = 0;
		virtual void unpack(const uint8_t* buf) { };
	};

protected:
	spi_device(const char* dev) {
		fd = open(dev, O_RDWR);
		if (fd < 0)
			throw "can't open device";
	}

	void set_mode(uint8_t mode);
	void set_bits_per_word(uint8_t bits);
	void set_max_speed(uint32_t speed);
#define KHZ 1000
#define MHZ 1000*KHZ

	void send_msg(void* tx, void* rx, int len);

	template<class command>
	void submit(std::vector<command*>& cmds) {
		std::vector<uint8_t> buf;
		struct spi_ioc_transfer* xfer = new spi_ioc_transfer[cmds.size()];

		int msg_length = 0;
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			msg_length += (*c)->length();

		buf.reserve(msg_length);

		uint8_t* b = &buf[0];
		unsigned int i = 0;
		for (auto c=cmds.begin(); c != cmds.end(); c++) {
			spi_device::command& cmd = **c;
			cmd.pack(b);

			xfer[i].tx_buf = (__u64) b;
			xfer[i].rx_buf = (__u64) b;
			xfer[i].cs_change = true;
			xfer[i].len = cmd.length();
			b += cmd.length();
			i++;
		}

		int status = ioctl(fd, SPI_IOC_MESSAGE(cmds.size()), xfer);
		if (status < 0)
			throw "failed sending spi message";

		b = &buf[0];
		for (auto c=cmds.begin(); c != cmds.end(); c++) {
			spi_device::command& cmd = **c;
			cmd.unpack(b);
			b += cmd.length();
		}
	}
};

