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

#include "parameters.h"
#include <cstdint>
#include <array>
#include <Eigen/Eigen>
#include <boost/program_options.hpp>

static std::vector<parameter*> parameters;
void init_parameters();

using namespace Eigen;

template<unsigned int N>
struct input_channels {
	virtual Matrix<float,1,N> get() = 0;
};

template<unsigned int N>
struct output_channels {
	virtual void set(const Matrix<float,1,N> values) = 0;
};

class stage {
	output_channels<3>& out;
	input_channels<3>& fb;
	Vector3f last_pos;
	Matrix<float, 4,3> R;

public:
	stage(output_channels<3>& out, input_channels<3>& fb)
		: out(out), fb(fb) { }

	/*
	 * calibrate():
	 * Perform basic first-order OLS regression to map feedback coordinate
	 * space to stage input space
	 */
	void calibrate(unsigned int n_pts=10);
	void move(const Vector3f pos);
	Vector3f get_last_pos();
};

void track(input_channels<4>& psd_inputs,
		stage& stage_outputs, input_channels<3>& fb_inputs);

