#include <array>

#include <stdint.h>
#include <linux/spi/spidev.h>
#include "bitfield.h"

static const char* in_dev = "/dev/spidev1.0";
static const char* out_dev = "/dev/spidev1.1";

static uint16_t delay = 0;
static uint8_t mode = 0;
static uint8_t bits = 16;
static uint32_t speed = 500000;


int main(int argc, char** argv)
{
	int fd;
       
	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	configure_spi(fd);
}

