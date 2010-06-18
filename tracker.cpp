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
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <boost/random.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/format.hpp>

using std::string;
using std::vector;
using std::array;
using Eigen::Dynamic;

template<typename Matrix>
void dump_matrix(Matrix A, const char* filename)
{
	Eigen::IOFormat fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");
        std::ofstream os(filename);
        os << A.format(fmt);
        os.close();
}

Vector4f tracker::scale_psd_position(Vector4f in)
{
        if (scale_psd_inputs) {
                in.x() /= in[2];
                in.y() /= in[3];
        }
        return in;
}
static void dump_data(std::string file, vector<collect_cb<3>::point> fb_data,
                vector<collect_cb<4>::point> psd_data, string comment="") {
        std::ofstream f(file);
        if (comment.length())
                f << "# " << comment << "\n";
	f << "# pos_x pos_y pos_z\tfb_x fb_y fb_z\tpsd_x psd_y sum_x sum_y\n";
        for (unsigned int i=0; i < psd_data.size(); i++)
		f << boost::format("%f %f %f\t%f %f %f\t%f %f %f %f\n") %
				fb_data[i].position[0] % fb_data[i].position[1] % fb_data[i].position[2] %
                                fb_data[i].values[0] % fb_data[i].values[1] % fb_data[i].values[2] %
				psd_data[i].values[0] % psd_data[i].values[1] % psd_data[i].values[2] % psd_data[i].values[3];
}

template<unsigned int axis>
static bool compare_position(collect_cb<4>::point a, collect_cb<4>::point b)
{
	return a.values[axis] < b.values[axis];
}

template<unsigned int axis>
static Vector3f find_bead(vector<collect_cb<4>::point> psd_data, vector<collect_cb<3>::point> fb_data) {
        unsigned int min=0, max=0;
        for (unsigned int i=0; i < psd_data.size(); i++) {
                if (psd_data[i].values[axis] < psd_data[min].values[axis])
                        min = i;
                if (psd_data[i].values[axis] > psd_data[max].values[axis])
                        max = i;
        }
	return (fb_data[min].values + fb_data[max].values) / 2;
}

Vector3f tracker::rough_calibrate()
{
	Vector3f start, step;
        Vector3u pts;

	float tmp = 0.5 - rough_cal_xy_step * rough_cal_xy_pts / 2;
	start << tmp, tmp, 0.5;
	step << rough_cal_xy_step, rough_cal_xy_step, 0;
	pts << rough_cal_xy_pts, rough_cal_xy_pts, 1;
	raster_route route_xy(start, step, pts);
	collect_cb<4> psd_data(psd_inputs);
        collect_cb<3> fb_data(fb_inputs);
	
        // Run X/Y scan and preprocess data
	execute_route(stage_outputs, route_xy, {&psd_data, &fb_data}, 10);
        for (auto i=psd_data.data.begin(); i != psd_data.data.end(); i++)
                i->values = scale_psd_position(i->values);

	// Find extrema of Vx, Vy
	Vector3f laser_pos;
	laser_pos.x() = find_bead<0>(psd_data.data, fb_data.data).x();
	laser_pos.y() = find_bead<1>(psd_data.data, fb_data.data).y();
        dump_data("rough", fb_data.data, psd_data.data,
                        (boost::format("Center %f %f") % laser_pos.x() % laser_pos.y()).str());

	// Scan in Z direction
	laser_pos.z() = 0.5 - rough_cal_z_step * rough_cal_z_pts/2;
	step = (Vector3f() << 0, 0, rough_cal_z_step).finished();
	pts = (Vector3u() << 1, 1, rough_cal_z_pts).finished();
	raster_route route_z(laser_pos, step, pts);
        psd_data.data.clear();
        fb_data.data.clear();

        // Run Z scan and preprocess data
	execute_route(stage_outputs, route_z, {&psd_data, &fb_data}, 100);
        for (auto i=psd_data.data.begin(); i != psd_data.data.end(); i++)
                i->values = scale_psd_position(i->values);

	// Find extrema of Vz
        float max_deriv = 0;
        for (unsigned int i=1; i < rough_cal_z_pts-1; i++) {
                float sum1 = psd_data.data[i+1].values[2] - psd_data.data[i+1].values[3];
                float z1 = fb_data.data[i+1].values[2];
                float sum2 = psd_data.data[i-1].values[2] - psd_data.data[i-1].values[3];
                float z2 = fb_data.data[i-1].values[2];
                float deriv = (sum1 - sum2) / (z1 - z2);
                if (deriv > max_deriv) {
                        max_deriv = deriv;
                        laser_pos.z() = fb_data.data[i].values[2];
                }
        }
        dump_data("rough_z", fb_data.data, psd_data.data);
        laser_pos.z() = 0.5;
	return laser_pos;
}

/*
 * pack_psd_inputs(): Pack input data into vector with higher order terms
 */
static Matrix<float,9,1> pack_psd_inputs(Vector4f data) {
	Matrix<float,9,1> R;

	// First order
	R[0] = data[0];			// Vx
	R[1] = data[1];			// Vy
	R[2] = -data[2] + data[3];	// Vsum = -Vsum_x + Vsum_y
	
	// Second order
	R[3] = R[0]*R[0];		// Vx^2
	R[4] = R[1]*R[1];		// Vy^2
	R[5] = R[2]*R[2];		// Vsum^2

	// Cross terms
	R[6] = R[0]*R[1];		// Vx*Vy
	R[7] = R[0]*R[2];		// Vx*Vsum
	R[8] = R[1]*R[2];		// Vy*Vsum

	return R;
}

tracker::fine_cal_result tracker::fine_calibrate(Vector3f rough_pos)
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
        fine_cal_result res;

	execute_route(stage_outputs, rt, {&psd_collect, &fb_collect}, fine_cal_pt_delay);

        // Scale data and find mean
        res.psd_mean = Vector4f::Zero();
        for (unsigned int i=0; i < fine_cal_pts; i++) {
                psd_collect.data[i].values = scale_psd_position(psd_collect.data[i].values);
                res.psd_mean += psd_collect.data[i].values;
        }
        res.psd_mean /= 1.0 * fine_cal_pts;

	// Fill R and S matricies with collected data
	Matrix<double, Dynamic,9> R(fine_cal_pts, 9);
        Matrix<double, Dynamic,3> S(fine_cal_pts, 3);
	for (unsigned int i=0; i < fine_cal_pts; i++) {
		R.row(i) = pack_psd_inputs(psd_collect.data[i].values - res.psd_mean).cast<double>();
		S.row(i) = (fb_collect.data[i].values - rough_pos).cast<double>();
	}

        dump_data("fine", fb_collect.data, psd_collect.data);

	// Solve regression coefficients
        SVD<Matrix<double, Dynamic,9> > svd(R);
        Matrix<double, 9,3> bt = svd.solve(S);
        res.beta = bt.transpose().cast<float>();

        bool compute_residuals = true;
        if (compute_residuals) {
                std::ofstream of("fine-resid");
                of << "# fb_x fb_y fb_z\tP_Lx P_Ly P_Lz\tresid_x resid_y resid_z\n";
                Vector3d rms = Vector3d::Zero();
                for (unsigned int i=0; i < fine_cal_pts; i++) {
                        Vector3f fb = fb_collect.data[i].values;
                        Vector3d s = S.row(i).transpose().cast<double>();
                        Vector3d resid = bt.transpose() * R.row(i).transpose() - s;
                        of << boost::format("%f %f %f\t%f %f %f\t%f %f %f\n") %
                                fb.x() % fb.y() % fb.z() %
                                s.x() % s.y() % s.z() %
                                resid.x() % resid.y() % resid.z();
                        rms += resid.cwiseProduct(resid);
                }
                rms = (rms / fine_cal_pts).cwiseSqrt();
                fprintf(stderr, "RMS Residuals: %f %f %f\n", rms.x(), rms.y(), rms.z());
        }

#ifdef DUMP_MATRICIES
        dump_matrix(R, "R");
        dump_matrix(S, "S");
        dump_matrix(bt, "beta");
#endif
        return res;
}

void tracker::feedback(fine_cal_result cal)
{
        unsigned int n = 0;
        unsigned int bad_pts = 0, good_pts = 0;
        std::ofstream f("pos");
        struct timespec start_time;
        clock_gettime(CLOCK_REALTIME, &start_time);
        float last_report_t = 0, rate_report_period = 5;
        unsigned int last_report_n = 0;

        _running = true;
	while (!boost::this_thread::interruption_requested()) {
		Vector3f fb = fb_inputs.get();
                Vector4f psd = psd_inputs.get();
                psd = scale_psd_position(psd) - cal.psd_mean;
		Matrix<float, 9,1> psd_in = pack_psd_inputs(psd);
		Vector3f delta = cal.beta * psd_in;

                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                float t = (ts.tv_sec - start_time.tv_sec) +
                        (ts.tv_nsec - start_time.tv_nsec)*1e-9;
                for (int i=0; i<3; i++) {
                        fb_pids[i].add_point(t, delta[i]);
                        delta[i] = fb_pids[i].get_response();
                        delta[i] += otf_amp * sin(2*M_PI/otf_freqs[i]*t);
                }

                if (delta.norm() > fb_max_delta) {
                        fprintf(stderr, "Error: Delta exceeded maximum, likely lost tracking\n");
                        bad_pts++;
                        continue;
                }

                Vector3f new_pos = fb - delta + fb_setpoint;
		new_pos.z() = 0.5;
		f << boost::format("%f\t%f\t%f\t%f\t%f\t%f\n") %
				delta.x() % delta.y() % delta.z() %
				new_pos.x() % new_pos.y() % new_pos.z();
                try {
                        stage_outputs.move(new_pos);
                } catch (clamped_output_error e) {
                        bad_pts++;
                        fprintf(stderr, "Clamped\n");
                        continue;
                }

                if (good_pts > 10)
                        good_pts = bad_pts = 0;
                if (bad_pts > 10) {
                        fprintf(stderr, "Lost tracking\n");
                        break;
                }

                good_pts++;
                n++;
                if (fb_show_rate && t > (last_report_t + rate_report_period)) {
                        float rate = (n - last_report_n) / (t - last_report_t);
                        fprintf(stderr, "Feedback loop rate: %f updates/sec\n", rate);
                        last_report_t = t;
                        last_report_n = n;
                }
		usleep(fb_delay);
	}

        _running = false;
        if (feedback_ended_cb)
                feedback_ended_cb();
}

void tracker::start_feedback(fine_cal_result cal)
{
        feedback_thread = boost::thread(&tracker::feedback, this, cal);
}

bool tracker::running()
{
        return _running;
}

void tracker::stop_feedback()
{
        feedback_thread.interrupt();
}

