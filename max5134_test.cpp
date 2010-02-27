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
#include "max5134.h"

using std::vector;
using std::array;

void set(max5134 dac) {
	vector<max5134::command*> cmds = {
		new max5134::write_thru_cmd({0}, 100),
		new max5134::write_thru_cmd({1}, 1000),
		new max5134::write_thru_cmd({2}, 10000),
		new max5134::write_thru_cmd({3}, 5000),
	};
	dac.submit(cmds);
}

int main(int argc, char** argv) {
	max5134 dac("/dev/spidev4.0");
	
	do {
		set(dac);
		sleep(1);
	} while (true);
}

