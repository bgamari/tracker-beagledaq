
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
#include <array>
#include <Eigen/Eigen>
#include "max1302.h"
#include "max5134.h"

using namespace Eigen;
using std::array;

template<unsigned int N>
struct input_channels {
	virtual Matrix<float,1,N> get() = 0;
};

template<unsigned int N>
struct output_channels {
	virtual void set(const Matrix<float,1,N> values) = 0;
};

struct clamped_output_error {
	Vector3f values;
	clamped_output_error(Vector3f values) : values(values) { }
};

template<unsigned int N>
struct test_inputs : input_channels<N> {
	Matrix<float,1,N> get() {
		Matrix<float,1,N> v = Matrix<float,1,N>::Zero();
		return v;
	}
};

template<unsigned int N>
struct test_outputs : output_channels<N> {
	void set(const Matrix<float,1,N> values) {
		printf("stage_pos: ");
		for (unsigned int i=0; i<N; i++)
			printf(" %f", values[i]);
		printf("\n");
	}
};

template<unsigned int N>
struct max1302_inputs : input_channels<N> {
	max1302& dev;
	array<int,N> channels;
	// Normalization parameters:
	float scale; // Full-scale range, in units of Vref
	float offset; // Offset, in units of Vref

	max1302_inputs(max1302& dev, const array<int,N> channels,
			max1302::input_range range) :
		dev(dev), channels(channels)
	{
		std::vector<max1302::command*> cmds;
		
		switch (range) {
		case max1302::SE_ZERO_PLUS_VREF:
			offset = 0; scale = 1; break;
		case max1302::SE_MINUS_VREF_PLUS_VREF:
			offset = -1; scale = 2; break;
		case max1302::SE_ZERO_PLUS_VREF2:
			offset = 0; scale = 0.5; break;
		default:
			fprintf(stderr, "Unknown range\n");
			abort();
		}
		
		for (unsigned int i=0; i<N; i++) {
			cmds.push_back(new max1302::mode_control_cmd(max1302::EXT_CLOCK));
			cmds.push_back(new max1302::input_config_cmd(channels[i], range));
		}
		dev.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}

	Matrix<float,1,N> get() {
		array<uint16_t,N> int_vals;
		Matrix<float,1,N> values;
		std::vector<max1302::command*> cmds;
		cmds.reserve(N);

		for (unsigned int i=0; i<N; i++)
			cmds.push_back(new max1302::start_conversion_cmd(channels[i], &int_vals[i]));
		dev.submit(cmds);
		for (unsigned int i=0; i<N; i++) {
			int_vals[i] *= 2; // HACK: Hardware seems to be off by factor of two
			values[i] = scale * int_vals[i] / 0xffff + offset;
			delete cmds[i];
		}
		return values;
	}
};

template<unsigned int N>
struct max5134_outputs : output_channels<N> {
	max5134& dev;
	array<max5134::chan_mask,N> channels;

	max5134_outputs(max5134& dev, const array<max5134::chan_mask,N> channels) :
		dev(dev), channels(channels) { }

	void set(const Matrix<float,1,N> values)
	{
		max5134::chan_mask all_mask;
		std::vector<max5134::command*> cmds;
		cmds.reserve(N+1);
		for (unsigned int i=0; i<N; i++) {
			if (values[i] < 0.0 || values[i] > 1.0)
				throw clamped_output_error(values);
			uint16_t out_val = values[i]*0xffff;
			cmds.push_back(new max5134::write_cmd(channels[i], out_val));
			all_mask |= channels[i];
		}
		cmds.push_back(new max5134::load_dac_cmd (all_mask));

		dev.submit(cmds);
		for (auto c=cmds.begin(); c != cmds.end(); c++)
			delete *c;
	}
};


