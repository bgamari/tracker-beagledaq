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
 * route: Represents a path of points through 3-space
 */
struct route {
	virtual Vector3f get_pos() = 0;
	virtual void operator++() = 0;
	virtual bool has_more() = 0;
};

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

