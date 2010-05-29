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
#include <array>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <time.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Eigen/Eigen>


using std::vector;
using std::array;
using Eigen::Dynamic;

const float stage_cal_range = 0.2;

const float rough_cal_xy_step = 0.01;
const unsigned int rough_cal_xy_pts = 20;
const float rough_cal_z_step = 0.02;
const unsigned int rough_cal_z_pts = 20;

const float fine_cal_range = 0.01;
const unsigned int fine_cal_pts = 1000;

const unsigned int feedback_delay = 100;	// us

void dump_matrix(MatrixXf A, const char* filename)
{
	Eigen::IOFormat fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");
        std::ofstream os(filename);
        os << A.format(fmt);
        os.close();
}

Vector4f scale_psd_position(Vector4f in)
{
#define SCALE_INPUTS 0
#if SCALE_INPUTS
        in.x() /= in[2];
        in.y() /= in[3];
#endif
        return in;
}

struct point_callback {
	/*
	 * Return false to abort route. true otherwise.
	 */
	virtual bool operator()(Vector3f& pos) = 0;
};

/*
 * route: Represents a path of points through 3-space
 */
struct route {
	virtual Vector3f get_pos() = 0;
	virtual void operator++() = 0;
	virtual bool has_more() = 0;
};

		
void stage::calibrate(unsigned int n_pts) {
	Matrix<float, Dynamic,4> X(n_pts, 4);
        Matrix<float, Dynamic,3> Y(n_pts, 3);

	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	engine e;
	boost::variate_generator<engine&, distribution> vg(e,
			distribution(0.5-stage_cal_range, 0.5+stage_cal_range));
	
	for (unsigned int i=0; i<n_pts; i++) {
		Vector3f out_pos;
		out_pos << vg(), vg(), vg();
		out.set(out_pos);
		usleep(100*1000);
		Vector3f fb_pos = fb.get();

		X.row(i)[0] = 1; // constant
		X.row(i).tail<3>() = fb_pos.transpose();
                Y.row(i) = out_pos;
	}
        R = X.svd().solve(Y);
}

void stage::move(Vector3f pos)
{
	Vector4f npos; // position with constant component
	npos[0] = 1;
	npos.tail<3>() = pos;

	Vector3f p = R.transpose() * npos;
	out.set(p);
	last_pos = pos;
}

Vector3f stage::get_last_pos() {
	return last_pos;
}

static void smooth_move(stage& stage,
		Vector3f to, unsigned int move_time)
{
	// How long to wait in between smoothed position updates
	const unsigned int smooth_delay = 10; // us
	const unsigned int smooth_pts = move_time / smooth_delay;
	Vector3f initial = stage.get_last_pos();
	Vector3f step = (to - initial) / smooth_pts;

	// Smoothly move into position
	for (unsigned int i=0; i<smooth_pts; i++) {
		Vector3f pos = initial + i*step;
		stage.move(pos);
		usleep(smooth_delay);
	}
	stage.move(to);
}

static void execute_route(stage& stage, route& route,
		vector<point_callback*> cbs=vector<point_callback*>(),
		unsigned int point_delay=1000, unsigned int move_time=100)
{
	for (; route.has_more(); ++route) {
		Vector3f pos = route.get_pos();
		smooth_move(stage, pos, move_time);
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

	random_route(array<boost::variate_generator<Engine&, Distribution>,3>& rngs,
			unsigned int n_pts) :
		rngs(rngs), n_pts(n_pts)
	{
		for (int i=0; i<3; i++)
			a[i] = rngs[i]();
	}

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
		for (int i=0; i < 3; i++) {
			pos[i] = m % points[i];
			m /= points[i];
                }
#define BIDIR_SCAN
#ifdef BIDIR_SCAN
                int dir = 1;
                for (int i=2; i >= 0; i--) {
                        if (dir < 0)
                                pos[i] = points[i] - pos[i] - 1;
                        if (pos[i] % 2)
                                dir *= -1;
		}
#endif
		return start.array() + step.array() * pos.cast<float>().array();
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

template<unsigned int axis>
static Vector3f find_bead(vector<collect_cb<4>::point> data) {
	Eigen::IOFormat fmt = Eigen::IOFormat(4, 0, " ", "\t");
	collect_cb<4>::point min = * min_element(data.begin(), data.end(),
			compare_position<axis>);
	collect_cb<4>::point max = * max_element(data.begin(), data.end(),
			compare_position<axis>);

	std::cerr << "extrema(" << axis << ")\t";
	std::cerr << min.position.format(fmt) << "\t" << min.values.format(fmt) << "\t";
	std::cerr << max.position.format(fmt) << "\t" << max.values.format(fmt) << "\n";

	return (min.position + max.position) / 2;
}

static Vector3f rough_calibrate(input_channels<4>& psd_inputs, stage& stage)
{
	Vector3f start, step;
        Vector3i pts;

	float tmp = 0.5 - rough_cal_xy_step * rough_cal_xy_pts / 2;
	start << tmp, tmp, 0.5;
	step << rough_cal_xy_step, rough_cal_xy_step, 0;
	pts << rough_cal_xy_pts, rough_cal_xy_pts, 1;
	raster_route route_xy(start, step, pts);
	collect_cb<4> xy_data(psd_inputs);
	
	execute_route(stage, route_xy, {&xy_data}, 100);

#define DUMP_ROUGH_CAL
#ifdef DUMP_ROUGH_CAL
	FILE* f = fopen("rough", "w");
	fprintf(f, "# pos_x pos_y pos_z\tpsd_x psd_y sum_x sum_y\n");
	for (auto i = xy_data.data.begin(); i != xy_data.data.end(); i++)
		fprintf(f, "%f %f %f\t%f %f %f %f\n",
				i->position[0], i->position[1], i->position[2],
				i->values[0], i->values[1], i->values[2], i->values[3]);
	fclose(f);
#endif

	Vector3f laser_pos;
	// Find extrema of Vx, Vy
	laser_pos.x() = find_bead<0>(xy_data.data).x();
	laser_pos.y() = find_bead<1>(xy_data.data).y();

	// Scan in Z direction
	laser_pos.z() = 0.5 - rough_cal_z_step * rough_cal_z_pts/2;
	step = (Vector3f() << 0, 0, rough_cal_z_step).finished();
	pts = (Vector3i() << 1, 1, rough_cal_z_pts).finished();
	raster_route route_z(laser_pos, step, pts);
	collect_cb<4> z_data(psd_inputs);
	execute_route(stage, route_z, {&z_data}, 10);
	// Find extrema of Vz
	laser_pos.z() = find_bead<2>(z_data.data).z();

	return laser_pos;
}

/*
 * pack_psd_inputs(): Pack input data into vector with higher order terms
 */
static Matrix<float,1,10> pack_psd_inputs(Vector4f data) {
	Matrix<float,1,10> R;

	// Offset
	R[0] = 1;

	// First order
	R[1] = data[0];			// Vx
	R[2] = data[1];			// Vy
	R[3] = -data[2] + data[3];	// Vsum = -Vsum_x + Vsum_y
	
	// Second order
	R[4] = R[1]*R[1];		// Vx^2
	R[5] = R[2]*R[2];		// Vy^2
	R[6] = R[3]*R[3];		// Vsum^2

	// Cross terms
	R[7] = R[1]*R[2];		// Vx*Vy
	R[8] = R[1]*R[3];		// Vx*Vsum
	R[9] = R[2]*R[3];		// Vy*Vsum

	return R;
}

static Matrix<float, 3,10> fine_calibrate(Vector3f rough_pos,
		input_channels<4>& psd_inputs, stage& stage,
		input_channels<3>& fb_inputs)
{
	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	typedef boost::variate_generator<engine&, distribution> vg;
	engine e;
	array<vg,3> rngs = {{
		vg(e, distribution(rough_pos.x()-fine_cal_range, rough_pos.x()+fine_cal_range)),
		vg(e, distribution(rough_pos.y()-fine_cal_range, rough_pos.y()+fine_cal_range)),
		vg(e, distribution(rough_pos.z()-fine_cal_range, rough_pos.z()+fine_cal_range))
	}};
	random_route<engine, distribution> rt(rngs, fine_cal_pts);
	collect_cb<4> psd_collect(psd_inputs);
	collect_cb<3> fb_collect(fb_inputs);

	execute_route(stage, rt, {&psd_collect, &fb_collect}, 10);

	// Fill R and S matricies with collected data
	Matrix<float, Dynamic,10> R(fine_cal_pts, 10);
        Matrix<float, Dynamic,3> S(fine_cal_pts, 3);
	for (unsigned int i=0; i < fine_cal_pts; i++) {
                Vector4f values = scale_psd_position(psd_collect.data[i].values);
		R.row(i) = pack_psd_inputs(values);
		S.row(i) = fb_collect.data[i].values - rough_pos.transpose();
	}

#define DUMP_FINE_CAL
#ifdef DUMP_FINE_CAL
	FILE* f = fopen("fine", "w");
	fprintf(f, "# pos_x pos_y pos_z\tfb_x fb_y fb_z\tpsd_x psd_y sum_x sum_y\n");
	for (unsigned int i=0; i < fine_cal_pts; i++) {
		fprintf(f, "%f %f %f\t%f %f %f\t%f %f %f %f\n",
				fb_collect.data[i].position.x(), fb_collect.data[i].position.y(), fb_collect.data[i].position.z(),
				fb_collect.data[i].values.x(), fb_collect.data[i].values.y(), fb_collect.data[i].values.z(),
				psd_collect.data[i].values[0], psd_collect.data[i].values[1], psd_collect.data[i].values[2], psd_collect.data[i].values[3]);
	}
	fclose(f);
#endif

	// Solve regression coefficients
        SVD<Matrix<float, Dynamic,10> > svd(R);
        Matrix<float, 10,3> bt = svd.solve(S);
        dump_matrix(R, "R");
        dump_matrix(S, "S");
        dump_matrix(bt, "beta");
	return bt.transpose();
}

void feedback(Matrix<float,3,10> R, input_channels<4>& psd_inputs,
		stage& stage, input_channels<3>& fb_inputs)
{
        const float max_delta = 0.5;
        FILE* f = fopen("pos", "w");
	while (true) {
                Vector4f psd = psd_inputs.get();
                psd = scale_psd_position(psd);
		Vector3f fb = fb_inputs.get();
		Matrix<float, 10,1> psd_in = pack_psd_inputs(psd);
		Vector3f delta = R * psd_in;
                if (delta.norm() > max_delta) {
                        fprintf(stderr, "Error: Delta exceeded maximum, likely lost tracking\n");
                        continue;
                }

                Vector3f new_pos = fb - delta;
		//new_pos.z() = 0.5; // TODO
		fprintf(f, "%f\t%f\t%f\n", new_pos.x(), new_pos.y(), new_pos.z());

		stage.move(new_pos);
		usleep(feedback_delay);
	}
        fclose(f);
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
void track(input_channels<4>& psd_inputs, stage& stage, input_channels<3>& fb_inputs)
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
	execute_route(stage, route, {&cb});
}
#else
void track(input_channels<4>& psd_inputs, stage& stage, input_channels<3>& fb_inputs)
{
	Vector3f rough_pos = rough_calibrate(psd_inputs, stage);
	stage.move(rough_pos);
	fprintf(stderr, "Rough Cal: %f %f %f\n", rough_pos[0], rough_pos[1], rough_pos[2]);
	//getchar();
        fprintf(stderr, "Starting fine calibration...\n");
	Matrix<float, 3,10> coeffs = fine_calibrate(rough_pos, psd_inputs, stage, fb_inputs);
        fprintf(stderr, "Fine calibration complete\n");

#define DUMP_COEFFS
#ifdef DUMP_COEFFS
        dump_matrix(coeffs, "coeffs");
#endif

	//getchar();
        fprintf(stderr, "Tracking...\n");
	feedback(coeffs, psd_inputs, stage, fb_inputs);
}
#endif

