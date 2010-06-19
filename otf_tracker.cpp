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

#include "otf_tracker.h"

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

Vector4f otf_tracker::scale_psd_position(Vector4f in)
{
        if (scale_psd_inputs) {
                in.x() /= in[2];
                in.y() /= in[3];
        }
        return in;
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

otf_tracker::perturb_response otf_tracker::find_perturb_response(
                unsigned int axis, float freq,
                boost::circular_buffer<otf_tracker::pos_log_entry>& log_data)
{
        std::vector<float> T(log_data.size());
        otf_tracker::perturb_response resp;
        float Ts = fb_delay * 1e-6;
        
        // Generate template sinusoid
        for (unsigned int i=0; i < log_data.size(); i++)
                T[i] = sin(2*M_PI*freq*i*Ts);
        
        // Auto-correlate for phase
        unsigned int max_lag = Ts / freq;
        float max_corr = 0;
        for (unsigned int lag=0; lag < max_lag; lag++) {
                float corr = 0;
                for (unsigned int i=0; i < log_data.size()-max_lag; i++)
                        corr += log_data[i].fb[axis] * log_data[i+lag].fb[axis];

                if (corr > max_corr) {
                        max_corr = corr;
                        resp.phase = corr;
                }
        }

        // Recompute template sinusoid with phase
        for (unsigned int i=0; i < log_data.size(); i++)
                T[i] = sin(2*M_PI*freq*i*Ts + resp.phase);

        // Find amplitude
        float a=0, b=0;
        for (unsigned int i=0; i < log_data.size(); i++) {
                a += log_data[i].fb[axis] * T[i];
                b += T[i] * T[i];
        }
        resp.amp = a / b;

        return resp;
}

/*
 * Responsible for periodically updating the regression matrix from data
 * collected in the feedback thread.
 */
void otf_tracker::recal_worker(Matrix<float, 3,9>& beta, Vector4f& psd_mean)
{
        while (true) {
                usleep(recal_delay);
                // Swap log buffers
                {
                        boost::mutex::scoped_lock lock(log_mutex);
                        std::swap(active_log, inactive_log);
                }

                // Generate sinusoid data for regression
                unsigned int samples = inactive_log->size();
                Matrix<double, Dynamic,9> R(samples,9);
                Matrix<double, Dynamic,3> S(samples,3);
                for (unsigned int axis=0; axis<3; axis++) {
                        float freq = perturb_freqs[axis];
                        perturb_response resp = find_perturb_response(axis, freq, *inactive_log);
                        for (unsigned int i=0; i<samples; i++) {
                                pos_log_entry& ent = inactive_log->at(i);
                                S(i,axis) = resp.amp * sin(2*M_PI*freq*ent.time + resp.phase);
                        }
                }

                // Compute new PSD mean
                Vector4f new_psd_mean = Vector4f::Zero();
                for (unsigned int i=0; i<samples; i++)
                        new_psd_mean += inactive_log->at(i).psd;
                new_psd_mean /= samples;

                // Pack PSD inputs
                for (unsigned int i=0; i<samples; i++)
                        R.row(i) = pack_psd_inputs(inactive_log->at(i).psd - new_psd_mean).cast<double>();

                // Solve regression
                SVD<Matrix<double, Dynamic,9> > svd(R);
                Matrix<double, 9,3> bt = svd.solve(S);
                beta = bt.transpose().cast<float>();
                psd_mean = new_psd_mean;

                inactive_log->clear();
        }
}

void otf_tracker::feedback()
{
        unsigned int n = 0;
        struct timespec start_time;
        clock_gettime(CLOCK_REALTIME, &start_time);
        float last_report_t = 0;
        unsigned int last_report_n = 0;
        std::ofstream f("pos");
        Matrix<float, 3,9> beta = Matrix<float,3,9>::Zero();
        Vector3f position;
        Vector4f psd_mean = Vector4f::Zero();
        boost::thread recal_thread(&otf_tracker::recal_worker, this, beta, psd_mean);

        _running = true;
	while (true) {
                // Get sensor values
		Vector3f fb = fb_inputs.get();
                Vector4f psd = psd_inputs.get();
                psd = scale_psd_position(psd);
                n++;

                // Add datum to log
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                float t = (ts.tv_sec - start_time.tv_sec) +
                        (ts.tv_nsec - start_time.tv_nsec)*1e-9;
                {
                        boost::mutex::scoped_lock lock(log_mutex);
                        pos_log_entry ent = { t, fb, psd };
                        active_log->push_back(ent);
                }

                // Compute estimated position
                psd -= psd_mean;
		Matrix<float, 9,1> psd_in = pack_psd_inputs(psd);
		Vector3f delta = beta * psd_in;

                if (delta.norm() > fb_max_delta) {
                        fprintf(stderr, "Error: Delta exceeded maximum, likely lost tracking\n");
                        continue;
                }

                // Get PID response and apply perturbation
                for (int i=0; i<3; i++) {
                        delta[i] += perturb_amp * sin(2*M_PI/perturb_freqs[i]*t);
                        fb_pids[i].add_point(t, delta[i]);
                        delta[i] = fb_pids[i].get_response();
                }

                // Move stage
                Vector3f new_pos = fb - delta + fb_setpoint;
		f << boost::format("%f\t%f\t%f\t%f\t%f\t%f\n") %
				delta.x() % delta.y() % delta.z() %
				position.x() % position.y() % position.z();
		if (n % move_skip_cycles == 0)
                        try {
                                stage_outputs.move(new_pos);
                        } catch (clamped_output_error e) {
                                fprintf(stderr, "Clamped\n");
                                continue;
                        }

                // Show feedback rate report
                if (fb_show_rate && t > (last_report_t + fb_rate_report_period)) {
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

unsigned int otf_tracker::get_log_length()
{
        return active_log->capacity();
}

void otf_tracker::set_log_length(unsigned int len)
{
        boost::mutex::scoped_lock lock(log_mutex);
        active_log->set_capacity(len);
        inactive_log->set_capacity(len);
}

void otf_tracker::start_feedback()
{
        feedback_thread = boost::thread(&otf_tracker::feedback, this);
}

bool otf_tracker::running()
{
        return _running;
}

void otf_tracker::stop_feedback()
{
        feedback_thread.interrupt();
}
