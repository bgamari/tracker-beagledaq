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

#include <fstream>
#include <Eigen/SVD>
#include "boost/format.hpp"

using namespace Eigen;

void stage::move(const Vector3f pos)
{
        out.set(pos);
        target_pos = pos;
}

void stage::move_rel(const Vector3f delta)
{
        move(target_pos + delta);
}

Vector3f stage::get_target_pos() const {
	return target_pos;
}

Vector3f stage::get_pos() const {
	return target_pos;
}

void fb_stage::calibrate(unsigned int n_pts, unsigned int n_samp) {
        stage raw_stage(out);
        raw_stage.move({0.5, 0.5, 0.5});

	typedef boost::mt19937 engine;
	typedef boost::uniform_real<float> distribution;
	engine e;
	boost::variate_generator<engine&, distribution> vg(e,
			distribution(0.5-cal_range, 0.5+cal_range));
	
	Matrix<float, Dynamic,7> X(n_pts, 7);
        Matrix<float, Dynamic,3> Y(n_pts, 3);
	Matrix<float, Dynamic,3> dev(n_pts, 3);
	Matrix<float, Dynamic,3> samples(n_samp, 3);
	for (unsigned int i=0; i<n_pts; i++) {
		Vector3f out_pos;
		out_pos << vg(), vg(), vg();
                raw_stage.smooth_move(out_pos, 4*1000);
		usleep(25*1000);

                for (unsigned int n=0; n<n_samp; n++)
			samples.row(n) = fb.get();
		Vector3f fb_mean = samples.colwise().sum() / n_samp;
		Vector3f fb_dev = ((samples.rowwise() - fb_mean.transpose()).array().square().colwise().sum() / n_samp).sqrt();

		dev.row(i) = fb_dev;
		X.row(i)[0] = 1; // constant
		X.row(i).segment(1,3) = fb_mean.transpose();
		X.row(i).segment(4,3) = fb_mean.transpose().array().square();
		Y.row(i) = out_pos;
	}
        JacobiSVD<Matrix<float, Dynamic,7> > svd(X);
        R = svd.solve(Y);

	{
		std::ofstream of("stage-cal");
		Matrix<float, Dynamic,3> resid(n_pts, 3);
		resid = X*R - Y;
		of << (MatrixXf(n_pts, 12) << X.col(1),X.col(2),X.col(3), Y, dev, resid).finished();
	}

        usleep(25*1000);
        move({0.5, 0.5, 0.5});
}

void fb_stage::move(const Vector3f pos)
{
	Matrix<float,7,1> npos;
	npos[0] = 1;
	npos.segment(1,3) = pos;
	npos.segment(4,3) = pos.array().square();

	Vector3f p = R.transpose() * npos;
	out.set(p);
	target_pos = pos;
}

Vector3f fb_stage::get_pos() const
{
        return fb.get();
}

void pid_stage::worker() 
{
        bool debug = false;
	unsigned int i=0; 
        std::ofstream fx("stage-x"), fy("stage-y"), fz("stage-z");
	while (!boost::this_thread::interruption_requested()) {
		Vector3f fb_pos = fb.get();
		Vector3f err = fb_pos - target_pos;

		pidx.add_point(i, err.x());
		pidy.add_point(i, err.y());
		pidz.add_point(i, err.z());

		Vector3f resp;
		resp << pidx.get_response(), pidy.get_response(), pidz.get_response();
		pos -= resp;
                if (debug) {
                        fx << (boost::format("%f\t%f\t%f\t%f\n") % target_pos[0] % fb_pos[0] % resp[0] % pos[0]);
                        fy << (boost::format("%f\t%f\t%f\t%f\n") % target_pos[1] % fb_pos[1] % resp[1] % pos[1]);
                        fz << (boost::format("%f\t%f\t%f\t%f\n") % target_pos[2] % fb_pos[2] % resp[2] % pos[2]);
                }
                for (int j=0; j<3; j++) {
                        pos[j] = std::min(1.0f, pos[j]);
                        pos[j] = std::max(0.0f, pos[j]);
                }
		out.set(pos);
		i++;
		usleep(fb_delay);
	}
}

pid_stage::~pid_stage() {
        fb_worker.interrupt();
        fb_worker.join();
}

void pid_stage::move(const Vector3f pos)
{
        for (int i=0; i<3; i++)
                if (pos[i] < 0 || pos[i] > 1) throw clamped_output_error(pos);
	target_pos = pos;
}

Vector3f pid_stage::get_pos() const
{
        return fb.get();
}

/*
 * smooth_move: Linearly interpolate position over time to smooth stage motion
 *
 * TODO: Acceleration?
 */
void stage::smooth_move(Vector3f to, unsigned int move_time)
{
	// How long to wait in between smoothed position updates
	const unsigned int smooth_delay = 50; // us
	const unsigned int smooth_pts = move_time / smooth_delay;
	Vector3f initial = get_pos();
	Vector3f step = (to - initial) / smooth_pts;

	// Smoothly move into position
	for (unsigned int i=0; i<smooth_pts; i++) {
		Vector3f pos = initial + i*step;
		move(pos);
		usleep(smooth_delay);
	}
        // Make sure we are all the way there
	move(to);
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

