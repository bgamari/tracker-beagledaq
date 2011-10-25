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


#include "beagledaq.h"

void beagledaq_init() {
	beagle_daq bd;
	//photodiode_in = new beagledaq_inputs(bd, 0, array<int,N>{{0,1,2}});
	stage_in = new beagledaq_inputs<3>(bd, 0, array<int,3>{{0,1,2}});
	psd_in = new beagledaq_inputs<4>(bd, 0, array<int,4>{{0,1,2}});
	stage_out = new beagledaq_outputs<3>(bd, 0, array<int,3>{{0,1,2}});
}
