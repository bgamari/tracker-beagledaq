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
#include <cmath>
#include "max5134.h"

using std::vector;
using std::array;

void set(max5134 dac, uint16_t value) {
	vector<max5134::command*> cmds = {
		new max5134::write_thru_cmd(0x1, value),
		new max5134::write_thru_cmd(0x2, value),
		new max5134::write_thru_cmd(0x4, value),
		new max5134::write_thru_cmd(0x8, value),
	};
	dac.submit(cmds);
	for (auto i=cmds.begin(); i != cmds.end(); i++)
		delete *i;
}

int main(int argc, char** argv) {
	max5134 dac("/dev/spidev4.0");
	
	vector<max5134::command*> cmds = {
		new max5134::pwr_cntrl_cmd(0xf, false),
	};
	dac.submit(cmds);

	float t = 0;
	do {
		float value = 0.5*sinf(t) + 0.5;
		set(dac, 0xffff*value);
		t += 0.0001;
		usleep(1*1);
	} while (true);
}

