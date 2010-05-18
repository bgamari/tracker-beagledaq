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

static void smooth_move(output_channels<3>& stage,
		Vector3f to, unsigned int move_time=1000)
{
	// How long to wait in between smoothed position updates
	const unsigned int smooth_delay = 10; // us
	const unsigned int smooth_pts = move_time / smooth_delay;
	Vector3f initial = stage.get_last();
	Vector3f step = (to - initial) / smooth_pts;

	// Smoothly move into position
	for (unsigned int i=0; i<smooth_pts; i++) {
		Vector3f pos = initial + i*step;
		stage.set(pos);
		usleep(smooth_delay);
	}
	stage.set(to);
}

static void execute_route(output_channels<3>& stage,
		route& route, vector<point_callback*> cbs=vector<point_callback*>(), unsigned int point_delay=1000)
{
	for (; route.has_more(); ++route) {
		Vector3f pos = route.get_pos();
		//printf("Pt\t%f\t%f\t%f\n", pos[0], pos[1], pos[2]);
		smooth_move(stage, pos);
		usleep(point_delay);
		for (auto cb = cbs.begin(); cb != cbs.end(); cb++)
			if (! (**cb)(pos))
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

template<unsigned int N>
struct collect_cb : point_callback {
	input_channels<N>& inputs;
	struct point {
		Vector3f position;
		Matrix<float,1,N> values;
	};
	std::vector<point> data;

	collect_cb(input_channels<N>& inputs) : inputs(inputs) { }

	bool operator()(Vector3f& pos) {
		point p = { pos, inputs.get() };
		data.push_back(p);
		return true;
	}
};

template<unsigned int axis>
bool compare_position(collect_cb<4>::point a, collect_cb<4>::point b)
{
	return a.values[axis] < b.values[axis];
}

static Vector3f rough_calibrate(input_channels<4>& psd_inputs, output_channels<3>& stage_outputs)
{
	Vector3f start, step;
        Vector3i pts;

	float tmp = 0.5 - rough_cal_xy_step * rough_cal_xy_pts / 2;
	start = (Vector3f() << tmp, tmp, 0.5).finished();
	step = (Vector3f() << rough_cal_xy_step, rough_cal_xy_step, 0).finished();
	pts = (Vector3i() << rough_cal_xy_pts, rough_cal_xy_pts, 1).finished();
	raster_route route_xy(start, step, pts);
	collect_cb<4> xy_data(psd_inputs);
	
	printf("Rough calibrate\n");
	execute_route(stage_outputs, route_xy, {&xy_data});

	FILE* f = fopen("rough", "w");
	for (auto i = xy_data.data.begin(); i != xy_data.data.end(); i++)
		fprintf(f, "%f %f %f\t%f %f %f %f\n", i->position[0], i->position[1], i->position[2],
				i->values[0], i->values[1], i->values[2], i->values[3]);
	fclose(f);

	// Find extrema of Vx
	Vector3f min_x_pos = min_element(xy_data.data.begin(), xy_data.data.end(), compare_position<0>)->position;
	Vector3f max_x_pos = max_element(xy_data.data.begin(), xy_data.data.end(), compare_position<0>)->position;
	// Find extrema of Vy
	Vector3f min_y_pos = min_element(xy_data.data.begin(), xy_data.data.end(), compare_position<1>)->position;
	Vector3f max_y_pos = max_element(xy_data.data.begin(), xy_data.data.end(), compare_position<1>)->position;

	printf("Min X: %f, %f\n", min_x_pos.x(), min_x_pos.y());
	printf("Max X: %f, %f\n", max_x_pos.x(), max_x_pos.y());
	printf("Min Y: %f, %f\n", min_y_pos.x(), min_y_pos.y());
	printf("Max Y: %f, %f\n", max_y_pos.x(), max_y_pos.y());

	Vector3f laser_pos;
        laser_pos.x() = (min_x_pos.x() + max_x_pos.x()) / 2;
	laser_pos.y() = (min_y_pos.y() + max_y_pos.y()) / 2;
	laser_pos.z() = 0.5 - rough_cal_z_step * rough_cal_z_pts/2;

	// Scan in Z direction
	step = (Vector3f() << 0, 0, rough_cal_z_step).finished();
	pts = (Vector3i() << 1, 1, rough_cal_z_pts).finished();
	raster_route route_z(laser_pos, step, pts);
	collect_cb<4> z_data(psd_inputs);
	execute_route(stage_outputs, route_z, {&z_data});

	// Find extrema of Vz
	Vector3f min_z_pos = min_element(z_data.data.begin(), z_data.data.end(), compare_position<2>)->position;
	Vector3f max_z_pos = max_element(z_data.data.begin(), z_data.data.end(), compare_position<2>)->position;
	printf("Min Z: %f, %f, %f\n", min_z_pos.x(), min_z_pos.y(), min_z_pos.z());
	printf("Max Z: %f, %f, %f\n", max_z_pos.x(), max_z_pos.y(), max_z_pos.z());
	
	laser_pos.z() = (max_z_pos.z() + min_z_pos.z()) / 2;
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
static Matrix<float,1,9> pack_inputs(Vector4f data) {
	Matrix<float,1,9> R;

	// First order
	R(0) = data[0];
	R(1) = data[1];
	R(2) = data[2];
	
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

static Matrix<float, 3,9> fine_calibrate(Vector3f rough_pos, input_channels<4>& psd_inputs, output_channels<3>& stage_outputs, input_channels<3>& fb_inputs)
{
	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	typedef boost::variate_generator<engine&, distribution> vg;
	engine e;
	const float scan_range = 0.01;
	array<vg,3> rngs = {{
		vg(e, distribution(rough_pos.x()-scan_range, rough_pos.x()+scan_range)),
		vg(e, distribution(rough_pos.y()-scan_range, rough_pos.y()+scan_range)),
		vg(e, distribution(rough_pos.z()-scan_range, rough_pos.z()+scan_range))
	}};
	random_route<engine, distribution> rt(rngs, fine_cal_pts);
	collect_cb<4> psd_collect(psd_inputs);
	collect_cb<3> fb_collect(fb_inputs);

	execute_route(stage_outputs, rt, {&psd_collect, &fb_collect}, 10000);

	// Fill R and S matricies with collected data
	Matrix<float, Dynamic,9> R(fine_cal_pts, 9);
        Matrix<float, Dynamic,3> S(fine_cal_pts, 3);
	for (unsigned int i=0; i < fine_cal_pts; i++) {
		R.row(i) = pack_inputs(psd_collect.data[i].values);
		for (unsigned int j=0; j<3; j++)
			S(i,j) = fb_collect.data[j].values[j];
	}
	return solve_response_matrix(R, S);
}

static Vector3f calculate_delta(Matrix<float, 3,9> R, Vector4f psd_datum) {
	Matrix<float, 9,1> v = pack_inputs(psd_datum);
	return R*v;
}

void feedback(Matrix<float,3,9> R, input_channels<4>& psd_inputs, output_channels<3>& stage_outputs, input_channels<3>& fb_inputs)
{
	while (true) {
		Vector4f in = psd_inputs.get();
		Vector3f delta = calculate_delta(R, in);
		printf("%f\t%f\t%f\n", delta[0], delta[1], delta[2]);
		stage_outputs.set(delta);
		usleep(feedback_delay);
	}
}

template<unsigned int N>
struct dump_inputs_cb : point_callback {
	input_channels<N>& in;
	dump_inputs_cb(input_channels<N>& in) : in(in) { }
	bool operator()(Vector3f& pos) {
		// pos_x pos_y pos_z\tpsd_x psd_y\tpsd_sum\tfb_x fb_y fb_z\n
		Matrix<float,1,N> d = in.get();
		printf("%f\t%f\t%f\t", pos[0], pos[1], pos[2]);
		for (unsigned int i=0; i<N; i++)
			printf("%f\t", d[i]);
		printf("\n");
		return true;
	}
};

//#define RASTER_TEST
#ifdef RASTER_TEST
void track(input_channels<4>& psd_inputs,
		output_channels<3>& stage_outputs, input_channels<3>& fb_inputs)
{
	float step_sz = 0.005;
	unsigned int npts = 40;
	float tmp = 0.5 - step_sz * npts / 2;

	Vector3f start = (Vector3f() << tmp, tmp, 0.5).finished();
	Vector3f step = (Vector3f() << step_sz, step_sz, 0).finished();
        Vector3i pts = (Vector3i() << npts, npts, 1).finished();
	raster_route route(start, step, pts);
	dump_inputs_cb<4> cb(psd_inputs);
	printf("# pos_x\tpos_y\tpos_z\tpsd_x\tpsd_y\tpsd_sum_x\tpsd_sum_y\n");
	execute_route(stage_outputs, route, {&cb});
}
#else
void track(input_channels<4>& psd_inputs,
		output_channels<3>& stage_outputs, input_channels<3>& fb_inputs)
{
	Vector3f rough_pos = rough_calibrate(psd_inputs, stage_outputs);
	stage_outputs.set(rough_pos);
	fprintf(stderr, "Rough Cal: %f %f %f\n", rough_pos[0], rough_pos[1], rough_pos[2]);
	getchar();
	auto coeffs = fine_calibrate(rough_pos, psd_inputs, stage_outputs, fb_inputs);
	getchar();
	return;
	feedback(coeffs, psd_inputs, stage_outputs, fb_inputs);
}
#endif

