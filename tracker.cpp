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


#include <utility>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <time.h>
#include "tracker.h"

using std::tr1::array;

Vector3f rough_cal_start = (Vector3f() << 0, 0, 0).finished();
Vector3f rough_cal_xy_step = (Vector3f() << 0.01, 0.01, 0).finished();
Vector3i rough_cal_xy_pts = (Vector3i() << 100, 100, 1).finished();

Vector3f rough_cal_z_step = (Vector3f() << 0, 0, 0.01).finished();
Vector3i rough_cal_z_pts = (Vector3i() << 1, 1, 100).finished();

unsigned int fine_cal_pts = 10000;

struct point_callback {
	/*
	 * operator()
	 *
	 * Returns: false to abort route. true otherwise.
	 *
	 */
	virtual bool operator()(Vector3f& pos) = 0;
};

struct route {
	virtual Vector3f get_pos() = 0;
	virtual void operator++() = 0;
	virtual bool has_more() = 0;
};

static void execute_route(output_channels& channels,
		route& route, point_callback* cb=NULL)
{
	for (; route.has_more(); ++route) {
		Vector3f pos = route.get_pos();
		output_data d = { pos };
		channels.set(d);
		if (cb)
			if (! (*cb)(pos))
				break;
	}
}

template <class Engine, class Distribution>
struct random_route : route {
	array<boost::variate_generator<Engine&, Distribution>,3>& rngs;
	unsigned int n_pts;
	Vector3f a;

	random_route(array<boost::variate_generator<Engine&, Distribution>,3>& rngs, unsigned int n_pts) :
		rngs(rngs), n_pts(n_pts) { }

	void operator++() {
		n_pts--;
		for (int i=0; i<3; i++)
			a[i] = rngs[i]();
	}

	Vector3f get_pos() { return a; }

	bool has_more() {
		return n_pts > 0;
	}
};

struct raster_route : route {
	const Vector3f start;
	const Vector3f step;

	const Vector3i points;
	Vector3i pos; // current position in steps
	Vector3i dirs;

	raster_route(Vector3f start, Vector3f step, Vector3i points) :
		start(start), step(step), points(points), pos(Vector3i::Zero()), dirs(Vector3i::Ones()) {  }

	Vector3f get_pos() {
		return step.cwise() * pos.cast<float>();
	}

	void operator++() {
		unsigned int i=0;
		while (true) {
			pos[i] += dirs[i];

			if (pos[i] <= 0 || pos[i] >= points[i])
				dirs[i] *= -1;
			else
				break;
		}
	}

	bool has_more() {
		for (int i=0; i<3; i++) {
			if (pos[i] <= points[i])
				return true;
		}
		return false;
	}
};

struct find_min_max : point_callback {
	input_channels& inputs;

	float min_sum, max_sum;
	array<Vector3f, 3> min_pos, max_pos;
	array<Vector3f, 3> min_fb_pos, max_fb_pos;

	find_min_max(input_channels& inputs) : inputs(inputs) { }

	bool operator()(Vector3f& pos) {
		input_data in = inputs.get();
		for (int i=0; i<3; i++) {
			if (in.psd_sum > max_sum) {
				max_sum = in.psd_sum;
				max_fb_pos[i] = in.fb_pos;
				max_pos[i] = pos;
			}
			if (in.psd_sum < min_sum) {
				min_sum = in.psd_sum;
				min_fb_pos[i] = in.fb_pos;
				min_pos[i] = pos;
			}
		}
		usleep(100);
		return true;
	}
};

struct collect_cb : point_callback {
	input_channels& inputs;
	typedef std::pair<Vector3f, input_data> datum;
	std::vector<datum> data;

	collect_cb(input_channels& inputs) : inputs(inputs) { }

	bool operator()(Vector3f& pos) {
		datum d(pos, inputs.get());
		data.push_back(d);
		return true;
	}
};

static Vector3f rough_calibrate(input_channels& inputs, output_channels& outputs)
{
	raster_route route_xy(rough_cal_start, rough_cal_xy_step, rough_cal_xy_pts);
	find_min_max scan_xy(inputs);

	execute_route(outputs, route_xy, &scan_xy);
	Vector3f laser_pos_xy = (scan_xy.max_pos[0] + scan_xy.min_pos[0]) / 2; // FIXME
	
	// Scan in Z direction
	raster_route route_z(laser_pos_xy, rough_cal_z_step, rough_cal_z_pts);
	find_min_max scan_z(inputs);
	execute_route(outputs, route_z, &scan_z);
	Vector3f laser_pos = (scan_z.max_pos[0] + scan_z.min_pos[0]) / 2;
	return laser_pos;
}

static Matrix<float, 9,3> solve_response_matrix(MatrixXf R)
{
	MatrixXf RRi = (R.transpose() * R).inverse();
	MatrixXf beta = (RRi * R.transpose()) * S;
	return beta.transpose();
}

static Matrix<float, 9,3> fine_calibrate(Vector3f rough_pos, input_channels& inputs, output_channels& outputs)
{
	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	typedef boost::variate_generator<engine&, distribution> vg;
	engine e;
	array<vg,3> rngs = {{
		vg(e, distribution(-100, +100)),
		vg(e, distribution(-100, +100)),
		vg(e, distribution(-100, +100))
	}};
	random_route<engine, distribution> rt(rngs, fine_cal_pts);
	collect_cb cb(inputs);

	execute_route(outputs, rt, &cb);
	return solve_response_matrix(r);
}

void feedback(Matrix<float,9,3> M, input_channels& inputs, output_channels& outputs)
{
	while (1) {
		input_data in = inputs.get();
		Vector3f delta = calulate_delta(in);
		output_data out = { delta };
		outputs.set(out);
	}
}

void track(input_channels& inputs, output_channels& outputs)
{
	Vector3f rough_pos = rough_calibrate(inputs, outputs);
	auto coeffs = fine_calibrate(rough_pos, inputs, outputs);
	feedback(coeffs, inputs, outputs);
}

