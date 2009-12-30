#pragma once

class spi_device {
	int fd;

protected:
	spi_device(const char* dev) {
		fd = open(device, O_RDWR);
		if (fd < 0)
			pabort("can't open device");
	}

	void set_mode(uint8_t mode);
	void set_bits_per_word(uint8_t bits);
	void set_speed(uint32_t speed);

	void write(void* tx, void* rx, int len)
	{
		int ret;
		struct spi_ioc_transfer tr = {
			.tx_buf = (unsigned long)tx,
			.rx_buf = (unsigned long)rx,
			.len = len
		};

		ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
		if (ret == 1)
			throw "failed sending spi message";
	}
};

