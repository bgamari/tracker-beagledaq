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

// spi mode
void spi_device::set_mode(uint8_t mode)
{
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
		fprintf(stderr, "failed to set mode\n");
		exit(1);
	}
}

// bits per word
void spi_device::set_bits_per_word(uint8_t bits) {
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
		fprintf(stderr, "failed to set bits-per-word\n");
		exit(1);
	}
}

// max speed (Hz)
void spi_device::set_max_speed(uint32_t speed)
{
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		fprintf(stderr, "failed to set max write speed\n");
		exit(1);
	}
	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		fprintf(stderr, "failed to set max read speed\n");
		exit(1);
	}
}

void spi_device::send_msg(void* tx, void* rx, int len)
{
	struct spi_ioc_transfer tr;

	tr.tx_buf = (unsigned long) tx;
	tr.rx_buf = (unsigned long) rx;
	tr.len = len;

	if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
		fprintf(stderr, "failed to send message\n");
		exit(1);
	}
}

