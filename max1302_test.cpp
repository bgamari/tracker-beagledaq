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

#include <array>
#include <vector>
#include <unistd.h>
#include "max1302.h"

using std::vector;
using std::array;

void sample(max1302 adc) {
	array<uint16_t,4> s;
	vector<spi_device::command*> cmds = {
		new max1302::start_conversion_cmd(0, s[0]),
		new max1302::start_conversion_cmd(0, s[1]),
		new max1302::start_conversion_cmd(0, s[2]),
		new max1302::start_conversion_cmd(0, s[3]),
	};
	adc.submit(cmds);
	for (auto i=cmds.begin(); i != cmds.end(); i++)
		delete *i;
}

int main(int argc, char** argv) {
	max1302 adc("/dev/spidev3.1");
	
	do {
		sample(adc);
		sleep(1);
	} while (true);
}

