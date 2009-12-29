#include <stdint.h>
#include <stdio.h>
#include <linux/spi/spidev.h>

static const char* in_dev = "/dev/spidev1.0";
static const char* out_dev = "/dev/spidev1.1";

static uint16_t delay = 0;
static uint8_t mode = 0;
static uint8_t bits = 16;
static uint32_t speed = 500000;

static uint16_t read_input(int fd, int chan)
{
	int ret;
	uint16_t res;
	uint16_t tx[4];
	uint16_t rx[ARRAY_SIZE(tx)] = {0, };

	if (chan > 7) {
		fprintf(stderr, "Invalid channel: %d\n", chan);
		exit(1);
	}

	tx[0] = 1 << 7		// Start bit
	      | chan << 4	// Channel
	      | 0x0 << 2	// RNG=0, BIP=0
	      | 0x1 << 0;	// PD1=0, PD0=1 (External clock)
	tx[1] = 0x00;
	tx[2] = 0x00;
	tx[3] = 0x00;

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1)
		fprintf(stderr, "can't send spi message");

	res = (rx[0] & 0x3) << 10;
	res |= (rx[1] & 0xffc0) >> 6;
	return res;
}

static void set_bits(uint16_t* buf, int start_bit, int end_bit, uint16_t* data)
{
	int i = start_bit / sizeof(uint16_t);
	int j = start_bit % sizeof(uint16_t);


}

static void read_inputs(int fd, int* channels, int len, int* res)
{
	int ret;
	int bytes = ceil(18*len/16);
	uint16_t tx[bytes];
	uint16_t rx[ARRAY_SIZE(tx)] = {0, };

	for (int i=0; i<len; i++) {
		if (channels[i] > 7) {
			fprintf(stderr, "Invalid channel: %d\n", channels[i]);
			exit(1);
		}

		tx[0] = 1 << 7			// Start bit
		      | channels[i] << 4	// Channel
		      | 0x0 << 2		// RNG=0, BIP=0
		      | 0x1 << 0;		// PD1=0, PD0=1 (External clock)
		tx[1] = 0x00;
		tx[2] = 0x00;
		tx[3] = 0x00;
	}

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1)
		fprintf(stderr, "can't send spi message");

	res = (rx[0] & 0x3) << 10;
	res |= (rx[1] & 0xffc0) >> 6;
	return res;

}

static void write_output(int fd, chan_t chan, uint16_t value)
{
	int ret;
	uint16_t res;
	uint16_t tx[1];

	if (chan > 7) {
		fprintf(stderr, "Invalid channel: %d\n", chan);
		exit(1);
	}

	tx[0] = (chan & 0x0008) << 12 | (value & 0x0fff)

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = NULL,
		.len = ARRAY_SIZE(tx),
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret == 1)
		fprintf(stderr, "can't send spi message");
}

void configure_spi(int fd) {
	int ret;

	// spi mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	// bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	// max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
}

int main(int argc, char** argv)
{
	int fd;
       
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	configure_spi(fd);
}

