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

#include <cstdint>
#include <tr1/array>
#include <Eigen/Core>

USING_PART_OF_NAMESPACE_EIGEN

template<unsigned int N>
struct input_channels {
	virtual Matrix<float,1,N> get() = 0;
};

template<unsigned int N>
struct output_channels {
protected:
	Matrix<float,1,N> last_values;
public:
	virtual void set(Matrix<float,1,N> values) = 0;
	Matrix<float,1,N> get_last() { return last_values; }
};

void track(input_channels<4>& psd_inputs,
		output_channels<3>& stage_outputs, input_channels<3>& fb_inputs);

