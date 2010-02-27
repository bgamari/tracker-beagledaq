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

struct input_data {
	Vector2f psd_pos;
	float psd_sum;
	Vector3f fb_pos;
};

struct output_data {
	Vector3f stage_pos;
};

struct input_channels {
	virtual input_data get() = 0;
};

struct output_channels {
	virtual void set(output_data values) = 0;
};

void track(input_channels& inputs, output_channels& outputs);

