#include "max5590.h"

uint16_t max5590::build_command(enum command cmd, short data)
{
	return ((0xf & cmd) << 12) | (0x0fff & data);
}

