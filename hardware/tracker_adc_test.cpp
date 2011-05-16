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
#include <cstdio>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include "max1302.h"

using std::vector;
using std::array;

void sample(max1302 adc) {
	array<uint16_t,4> s;
	vector<max1302::start_conversion_cmd> starts = {
		max1302::start_conversion_cmd(0, &s[0]),
		max1302::start_conversion_cmd(1, &s[1]),
		max1302::start_conversion_cmd(2, &s[2]),
		max1302::start_conversion_cmd(3, &s[3]),
	};
	vector<max1302::command*> cmds = { &starts[0], &starts[1], &starts[2], &starts[3] };
	adc.submit(cmds);
	for (unsigned int i=0; i<cmds.size(); i++)
		printf("%d\t", s[i]);
	printf("\n");
}

int main(int argc, char** argv) {
        const char* device = "/dev/spidev3.0";
        if (argc > 1)
                device = argv[1];
	max1302 adc(device);
	vector<max1302::command*> cmds;
	
	sched_param sp;
	sp.sched_priority = 10;
	if (sched_setscheduler(0, SCHED_RR, &sp))
		fprintf(stderr, "Warning: Failed to acquire SCHED_RR\n");

	for (int i=0; i<8; i++) {
		cmds.push_back(new max1302::input_config_cmd(i, max1302::SE_ZERO_PLUS_VREF));
		cmds.push_back(new max1302::mode_control_cmd(max1302::EXT_CLOCK));
	}
	adc.submit(cmds);
	for (auto i=cmds.begin(); i!=cmds.end(); i++)
		delete *i;

	const unsigned int rep = 10000;
	unsigned int n=0;
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC, &start);
	do {
		if (n % rep == 0) {
			struct timespec stop;
			clock_gettime(CLOCK_MONOTONIC, &stop);
			float t = (stop.tv_nsec - start.tv_nsec)*1e-9 + (stop.tv_sec - start.tv_sec);
			float rate = 1.0*4*rep/t; // samp/sec
			fprintf(stderr, "%f samp/sec\t%f usec/samp\n", rate, 1e6/rate);
			start = stop;
		}
		sample(adc);
		n++;
		//usleep(5*100);
	} while (true);
}

