#pragma once

#include <cstdint>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

class spi_device {
	int fd;

protected:
	spi_device(const char* dev) {
		fd = open(dev, O_RDWR);
		if (fd < 0)
			throw "can't open device";
	}

	void set_mode(uint8_t mode);
	void set_bits_per_word(uint8_t bits);
	void set_speed(uint32_t speed);

	void write(void* tx, void* rx, int len);
};

