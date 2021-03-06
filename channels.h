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

using namespace Eigen;
using std::array;

template<unsigned int N>
struct input_channels {
	virtual Matrix<float,1,N> get(bool trigger=true) const = 0;
};

template<unsigned int N>
struct output_channels {
protected:
        Matrix<float,1,N> last;
public:
        const Matrix<float,1,N> get() const {
                return last;
        }
	virtual void set(const Matrix<float,1,N> values) {
                last = values;
        }
};

struct clamped_output_error {
	Vector3f values;
	clamped_output_error(Vector3f values) : values(values) { }
};

template<unsigned int N>
struct test_inputs : input_channels<N> {
	Matrix<float,1,N> get(bool trigger=true) const
	{
		Matrix<float,1,N> v = Matrix<float,1,N>::Zero();
		return v;
	}
};

template<unsigned int N>
struct test_outputs : output_channels<N> {
	void set(const Matrix<float,1,N> values)
	{
                this->last = values;
		for (unsigned int i=0; i<N; i++)
			printf(" %f", values[i]);
		printf("\n");
	}
};

extern input_channels<1>* photodiode_in;
extern input_channels<3>* stage_in;
extern input_channels<4>* psd_in;
extern output_channels<3>* stage_out;

extern void (*init_hardware)();

