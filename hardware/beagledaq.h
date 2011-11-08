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


#pragma once

void beagledaq_init();

#include <beagledaq/beagledaq.h>
#include "channels.h"

template<unsigned int N>
struct beagledaq_inputs : input_channels<N> {
	beagle_daq& bd;
	int adc;
	array<int,N> channels;
	bool posOnly; // Only positive voltages are to be measured
	
	beagledaq_inputs(beagle_daq& bd, int adc, const array<int,N> channels, bool posOnly=true) :
		bd(bd), adc(adc), channels(channels), posOnly(posOnly) { }

	Matrix<float,1,N> get() const
	{
		array<int16_t, 8> samp = bd.adcs[adc]->read();
		Matrix<float,1,N> ret;
		if (posOnly)
			for (unsigned int i=0; i<N; i++)
				ret[i] = 1. * samp[channels[i]] / 0x7fff;
		else 
			for (unsigned int i=0; i<N; i++)
				ret[i] = 1. * (samp[channels[i]] + 0x7fff) / 0xffff;
		return ret;
	}
};

template<unsigned int N>
struct beagledaq_outputs : output_channels<N> {
	beagle_daq& bd;
	int dac;
	array<int,N> channels;

	beagledaq_outputs(beagle_daq& bd, int dac, const array<int,N> channels) :
		bd(bd), dac(dac), channels(channels) { }

	void set(const Matrix<float,1,N> values) const
	{
		std::vector<dac8568::command*> cmds;
		cmds.reserve(N);
		for (unsigned int i=0; i<N; i++) {
			if (values[i] < 0.0 || values[i] > 1.0)
				throw clamped_output_error(values);
			uint16_t out_val = lrint(values[i]*0xffff);
			if (i < N-1)
				cmds.push_back(new dac8568::write_cmd(dac8568::write_cmd::write_mode::WRITE, channels[i], out_val));
			else
				cmds.push_back(new dac8568::write_cmd(dac8568::write_cmd::write_mode::WRITE_UPDATE_ALL, channels[i], out_val));
		}

		bd.dacs[dac]->submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}
};

