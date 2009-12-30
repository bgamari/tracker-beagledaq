#include "spi_device.h"

// spi mode
void spi_device::set_mode(uint8_t mode)
{
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
}

// bits per word
void spi_device::set_bits_per_word(uint8_t bits) {
	int ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");
}

// max speed hz
void spi_device::set_speed(uint32_t speed)
{
	int ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
}

