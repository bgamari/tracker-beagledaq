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



#include "tracker.h"

#include <time.h>
#include <utility>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <time.h>
#include <cstdio>
#include <Eigen/Array>
#include <Eigen/LU>


using std::vector;
using std::tr1::array;
using Eigen::Dynamic;

const float rough_cal_xy_step = 0.01;
const unsigned int rough_cal_xy_pts = 20;
const unsigned int rough_cal_z_step = 0.01;
const unsigned int rough_cal_z_pts = 40;

unsigned int fine_cal_pts = 1000;

const unsigned int feedback_delay = 1000;	// us

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

static void smooth_move(stage_outputs& stage,
		Vector3f to, unsigned int move_time=1000)
{
	// How long to wait in between smoothed position updates
	const unsigned int smooth_delay = 10; // us
	const unsigned int smooth_pts = move_time / smooth_delay;
	Vector3f initial = stage.get_position();
	Vector3f step = (to - initial) / smooth_pts;

	// Smoothly move into position
	for (unsigned int i=0; i<smooth_pts; i++) {
		Vector3f pos = initial + i*step;
		stage.move(pos);
		usleep(smooth_delay);
	}
	stage.move(to);
}

static void execute_route(stage_outputs& stage,
		route& route, point_callback* cb=NULL, unsigned int point_delay=1000)
{
	for (; route.has_more(); ++route) {
		Vector3f pos = route.get_pos();
		//printf("Pt\t%f\t%f\t%f\n", pos[0], pos[1], pos[2]);
		smooth_move(stage, pos);
		usleep(point_delay);
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

	unsigned int n;

	raster_route(Vector3f start, Vector3f step, Vector3i points) :
		start(start), step(step), points(points), n(0) {  }

	Vector3f get_pos() {
		Vector3i pos;
		unsigned int m = n;
		for (int i=0; i<3; i++) {
			pos[i] = m % points[i];
			m /= points[i];
		}
		return start + step.cwise() * pos.cast<float>();
	}

	void operator++() {
		n++;
	}

	bool has_more() {
		unsigned int N=1;
		for (int i=0; i<3; i++)
			N *= points[i];
		return n < N;
	}
};

struct collect_cb : point_callback {
	input_channels& inputs;
	struct point {
		Vector3f position;
		input_data data;
	};
	std::vector<point> data;

	collect_cb(input_channels& inputs) : inputs(inputs) { }

	bool operator()(Vector3f& pos) {
		point p = { pos, inputs.get() };
		data.push_back(p);
		return true;
	}
};

template<unsigned int axis>
bool compare_position(vector<collect_cb::point>::iterator a, vector<collect_cb::point>::iterator b)
{
	return a->data.psd_pos[axis] < b->data.psd_pos[axis];
}

static Vector3f rough_calibrate(input_channels& inputs, stage_outputs& stage)
{
	Vector3f start, step;
        Vector3i pts;

	float tmp = 0.5 - rough_cal_xy_step * rough_cal_xy_pts / 2;
	start = (Vector3f() << tmp, tmp, 0.5).finished();
	step = (Vector3f() << rough_cal_xy_step, rough_cal_xy_step, 0).finished();
	pts = (Vector3i() << rough_cal_xy_pts, rough_cal_xy_pts, 1).finished();
	raster_route route_xy(start, step, pts);
	collect_cb xy_data(inputs);
	
	execute_route(stage, route_xy, &xy_data);

	Vector3f min_x_pos = min(xy_data.data.begin(), xy_data.data.end(), compare_position<0>)->position;
	Vector3f max_x_pos = max(xy_data.data.begin(), xy_data.data.end(), compare_position<0>)->position;
	Vector3f min_y_pos = min(xy_data.data.begin(), xy_data.data.end(), compare_position<1>)->position;
	Vector3f max_y_pos = max(xy_data.data.begin(), xy_data.data.end(), compare_position<1>)->position;

	Vector3f laser_pos_xy;
        laser_pos_xy.x() = (min_x_pos.x() + max_x_pos.x()) / 2;
	laser_pos_xy.y() = (min_y_pos.y() + max_y_pos.y()) / 2;
	laser_pos_xy.z() = 0;
	
	// Scan in Z direction
	step = (Vector3f() << 0, 0, rough_cal_z_step).finished();
	pts = (Vector3i() << 1, 1, rough_cal_z_pts).finished();
	raster_route route_z(laser_pos_xy, step, pts);
	collect_cb z_data(inputs);
	execute_route(stage, route_z, &z_data);

	//Vector3f
	Vector3f laser_pos; // = (scan_z.max_pos[0] + scan_z.min_pos[0]) / 2;
	return laser_pos;
}

static Matrix<float, 3,9> solve_response_matrix(Matrix<float, Dynamic,9> R, Matrix<float, Dynamic,3> S)
{
	Matrix<float, 9,9> RRi = (R.transpose() * R).inverse();
	Matrix<float, 9,3> beta = RRi * R.transpose() * S;
	Matrix<float, 3,9> bt = beta.transpose();
	return bt;
}

/*
 * pack_inputs(): Pack input data into vector with higher order terms
 */
static Matrix<float,1,9> pack_inputs(input_data data) {
	Matrix<float,1,9> R;

	// First order
	R(0) = data.psd_pos[0];
	R(1) = data.psd_pos[1];
	R(2) = data.psd_sum;
	
	// Second order
	R(3) = R(0)*R(0);
	R(4) = R(1)*R(1);
	R(5) = R(2)*R(2);

	// Cross terms
	R(6) = R(0)*R(1);
	R(7) = R(0)*R(2);
	R(8) = R(1)*R(2);
	return R;
}

static Matrix<float, 3,9> fine_calibrate(Vector3f rough_pos, input_channels& inputs, stage_outputs& stage)
{
	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	typedef boost::variate_generator<engine&, distribution> vg;
	engine e;
	const float scan_range = 0.01;
	array<vg,3> rngs = {{
		vg(e, distribution(rough_pos[0]-scan_range, rough_pos[0]+scan_range)),
		vg(e, distribution(rough_pos[1]-scan_range, rough_pos[1]+scan_range)),
		vg(e, distribution(rough_pos[2]-scan_range, rough_pos[2]+scan_range))
	}};
	random_route<engine, distribution> rt(rngs, fine_cal_pts);
	collect_cb cb(inputs);

	execute_route(stage, rt, &cb, 10000);

	// Fill R and S matricies with collected data
	Matrix<float, Dynamic,9> R(fine_cal_pts, 9);
        Matrix<float, Dynamic,3> S(fine_cal_pts, 3);
	for (unsigned int i=0; i < fine_cal_pts; i++) {
		input_data data = cb.data[i].data;
		R.row(i) = pack_inputs(data);
		for (unsigned int j=0; j<3; j++)
			S(i,j) = data.fb_pos[j];
	}
	return solve_response_matrix(R, S);
}

static Vector3f calculate_delta(Matrix<float, 3,9> R, input_data in) {
	Matrix<float, 9,1> v = pack_inputs(in);
	return R*v;
}

void feedback(Matrix<float,3,9> R, input_channels& inputs, stage_outputs& stage)
{
	while (true) {
		input_data in = inputs.get();
		Vector3f delta = calculate_delta(R, in);
		printf("%f\t%f\t%f\n", delta[0], delta[1], delta[2]);
		stage.move(delta);
		usleep(feedback_delay);
	}
}

struct dump_inputs_cb : point_callback {
	input_channels& in;
	dump_inputs_cb(input_channels& in) : in(in) { }
	bool operator()(Vector3f& pos) {
		// pos_x pos_y pos_z\tpsd_x psd_y\tpsd_sum\tfb_x fb_y fb_z\n
		input_data d = in.get();
		for (int i=0; i<3; i++)
			printf("%f ", pos[i]);
		printf("\t");
		for (int i=0; i<2; i++)
			printf("%f ", d.psd_pos[i]);
		printf("\t%f\t", d.psd_sum);
		for (int i=0; i<3; i++)
			printf("%f ", d.fb_pos[i]);
		printf("\n");
		usleep(2000);
		return true;
	}
};

//#define TEST
#ifdef TEST
void track(input_channels& inputs, stage_outputs& stage)
{
	float step_sz = 0.02;
	unsigned int npts = 20;
	float tmp = 0.5 - step_sz * npts / 2;

	Vector3f start = (Vector3f() << tmp, tmp, 0.5).finished();
	Vector3f step = (Vector3f() << step_sz, step_sz, 0).finished();
        Vector3i pts = (Vector3i() << npts, npts, 1).finished();
	raster_route route(start, step, pts);
	dump_inputs_cb cb(inputs);
	printf("# pos_x pos_y pos_z\tpsd_x psd_y\tpsd_sum\tfb_x fb_y fb_z\n");
	execute_route(stage, route, &cb);
}
#else
void track(input_channels& inputs, stage_outputs& stage)
{
	Vector3f rough_pos = rough_calibrate(inputs, stage);
	fprintf(stderr, "Rough Cal: %f %f %f\n", rough_pos[0], rough_pos[1], rough_pos[2]);
	getchar();
	auto coeffs = fine_calibrate(rough_pos, inputs, stage);
	getchar();
	feedback(coeffs, inputs, stage);
}
#endif

