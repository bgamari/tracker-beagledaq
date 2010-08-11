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

#include "channels.h"

#include <vector>
#include <boost/random.hpp>
#include <Eigen/Eigen>

using std::vector;

typedef Matrix<unsigned int,3,1> Vector3u;

class stage {
protected:
        Vector3f last_pos;
public:
        const output_channels<3>& out;
        stage(const output_channels<3>& out) : out(out) { }
	virtual void move(const Vector3f pos);
        void smooth_move(Vector3f to, unsigned int move_time);
	Vector3f get_last_pos() const;
};

class fb_stage : public stage {
	Matrix<float, 4,3> R;
public:
	const input_channels<3>& fb;
        float cal_range;
	fb_stage(const output_channels<3>& out, const input_channels<3>& fb, float cal_range=0.4)
		: stage(out), fb(fb), cal_range(cal_range) { }

	/*
	 * calibrate():
	 * Perform basic first-order OLS regression to map feedback coordinate
	 * space to stage input space
	 */
	void calibrate(unsigned int n_pts=40, unsigned int n_samp=6);
	void move(const Vector3f pos);
};

/*
 * point_callback: A callback function called on every point of a route
 */
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

void execute_route(stage& stage, route& route,
		vector<point_callback*> cbs=vector<point_callback*>(),
		unsigned int point_delay=1000, unsigned int move_time=100);

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
	const Vector3u points;

	unsigned int n;

	raster_route(Vector3f start, Vector3f step, Vector3u points) :
		start(start), step(step), points(points), n(0) {  }

        Vector3f get_pos();
        void operator++();
	bool has_more();
};

// Collect a single reading from a set of input channels at each point
template<unsigned int N>
struct collect_cb : point_callback {
	input_channels<N>& inputs;
	struct point {
		Vector3f position;
		Matrix<float,N,1> values;
	};
	std::vector<point> data;

	collect_cb(input_channels<N>& inputs) : inputs(inputs) { }

        bool operator()(Vector3f& pos) {
		point p = { pos, inputs.get() };
		data.push_back(p);
		return true;
	}
};

// Collect multiple readings from each point
template<unsigned int N>
struct multi_collect_cb : point_callback {
	input_channels<N>& inputs;
	struct point {
		Vector3f position;
		vector<Matrix<float,N,1>> values;
	};
	std::vector<point> data;
        unsigned int n_pts, delay;

	multi_collect_cb(input_channels<N>& inputs,
                        unsigned int n_pts, unsigned int delay=0) :
                inputs(inputs), n_pts(n_pts), delay(delay) { }

        bool operator()(Vector3f& pos) {
		point p;
                p.position = pos;
                for (unsigned int i=0; i < n_pts; i++) {
                        p.values.push_back(inputs.get());
                        usleep(delay);
                }
		data.push_back(p);
		return true;
	}
};

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

