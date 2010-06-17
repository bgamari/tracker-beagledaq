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

#include "stage.h"

void stage::move(const Vector3f pos) {
        out.set(pos);
        last_pos = pos;
}

Vector3f stage::get_last_pos() const {
	return last_pos;
}

void fb_stage::calibrate(unsigned int n_pts) {
	Matrix<float, Dynamic,4> X(n_pts, 4);
        Matrix<float, Dynamic,3> Y(n_pts, 3);

	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	engine e;
	boost::variate_generator<engine&, distribution> vg(e,
			distribution(0.5-cal_range, 0.5+cal_range));
	
        stage raw_stage(out);
        raw_stage.move({0.5, 0.5, 0.5});
	FILE* f = fopen("stage-cal", "w");
	for (unsigned int i=0; i<n_pts; i++) {
		Vector3f out_pos;
		out_pos << vg(), vg(), vg();
                smooth_move(raw_stage, out_pos, 4*1000);
		usleep(100*1000);
		Vector3f fb_pos = fb.get();

		X.row(i)[0] = 1; // constant
		X.row(i).tail<3>() = fb_pos.transpose();
                Y.row(i) = out_pos;
		
		fprintf(f, "%f %f %f\t%f %f %f\n",
				out_pos.x(), out_pos.y(), out_pos.z(),
				fb_pos.x(), fb_pos.y(), fb_pos.z());
	}
	fclose(f);
        R = X.svd().solve(Y);
}

void fb_stage::move(const Vector3f pos)
{
	Vector4f npos; // position with constant component
	npos[0] = 1;
	npos.tail<3>() = pos;

	Vector3f p = R.transpose() * npos;
	out.set(p);
	last_pos = pos;
}

/*
 * smooth_move: Linearly interpolate position over time to smooth stage motion..
 *
 * TODO: Acceleration?
 */
void smooth_move(stage& stage, Vector3f to, unsigned int move_time)
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

/*
 * execute_route: Execute a route, calling the given callback for every pont.
 *
 * Waits point_delay between moving to point and calling callback.
 */
void execute_route(stage& stage, route& route, vector<point_callback*> cbs,
		unsigned int point_delay, unsigned int move_time)
{
        stage.move(route.get_pos());
        usleep(10*1000);
	for (; route.has_more(); ++route) {
		Vector3f pos = route.get_pos();
		smooth_move(stage, pos, move_time);
		usleep(point_delay);
		for (auto cb = cbs.begin(); cb != cbs.end(); cb++)
			if (! (**cb)(pos))
				break;
	}
}


Vector3f raster_route::get_pos() {
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
                dir = pos[i]%2 ? -1 : +1;
        }
#endif
        return start.array() + step.array() * pos.cast<float>().array();
}

void raster_route::operator++() {
        n++;
}

bool raster_route::has_more() {
        unsigned int N=1;
        for (int i=0; i<3; i++)
                N *= points[i];
        return n < N;
}

