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
#include "max5590.h"

using std::vector;
using std::array;

void set(max5590 dac) {
	vector<max5590::command*> cmds = {
		new max5590::load_input_cmd(max5590::input_reg::A, 100),
		new max5590::load_input_cmd(max5590::input_reg::B, 1000),
		new max5590::load_input_cmd(max5590::input_reg::C, 10000),
		new max5590::load_input_cmd(max5590::input_reg::D, 5000),
		new max5590::load_dac_cmd(std::bitset<8>(0xf)),
	};
	dac.submit(cmds);
	for (auto i=cmds.begin(); i != cmds.end(); i++)
		delete *i;
}

int main(int argc, char** argv) {
	max5590 dac("/dev/spidev4.0");
	
	do {
		set(dac);
		sleep(1);
	} while (true);
}

