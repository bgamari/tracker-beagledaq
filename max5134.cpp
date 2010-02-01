#include "max5134.h"

void max5134::write_through(std::vector<int> channels)
{
	uint32_t data rx[1], tx[1] = { 
		(0x3<<20) |
		
}

