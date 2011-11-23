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
#include "utils.h"

#include <time.h>
#include <utility>
#include <array>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <tr1/random>
#include <Eigen/SVD>

using std::string;
using std::vector;
using std::array;
using Eigen::Dynamic;

Vector4f otf_tracker::scale_psd_position(Vector4f in)
{
        if (scale_psd_inputs) {
                float sum = in[3] - in[2];
                in /= sum;
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
                ring_buffer<otf_tracker::pos_log_entry>& log_data)
{
        otf_tracker::perturb_response resp = {0, 0};
        
        // Cross-correlate for phase
        float max_corr = 0;
        for (float ph=0; ph < phase_max; ph += phase_step) {
                float corr = 0;
                for (unsigned int i=0; i < log_data.size(); i++)
                        corr += sin(2*M_PI*freq*log_data[i].time + ph) * log_data[i].fb[axis];

                if (corr > max_corr) {
                        max_corr = corr;
                        resp.phase = corr;
                }
        }

        // Find amplitude
        float a=0, b=0;
        for (unsigned int i=0; i < log_data.size(); i++) {
                float y = sin(2*M_PI*freq*log_data[i].time + resp.phase);
                a += log_data[i].fb[axis] * y;
                b += y*y;
        }
        resp.amp = a / b;

        return resp;
}

/*
 * Responsible for periodically updating the regression matrix from data
 * collected in the feedback thread.
 */
void otf_tracker::recal_worker(Vector4f& psd_mean, unsigned int& recal_count)
{
	while (!stop) {
                usleep(recal_delay);

                // Swap log buffers
                {
                        std::lock_guard<std::mutex> lock(log_mutex);
                        std::swap(active_log, inactive_log);
                        active_log->clear();
                }

                unsigned int samples = inactive_log->size();
                if (!samples) {
                        std::cout << "recal_worker: No samples.\n";
                        continue;
                }

                if (record_data_cnt) {
                        char fname[256];
                        snprintf(fname, 256, "recal-data-%d", recal_count);
                        FILE* f = fopen(fname, "w");
                        for (unsigned int i=0; i<samples; i++) {
                                pos_log_entry& e = (*inactive_log)[i];
                                fprintf(f, "%f %f %f\t%f %f %f %f\n",
                                        e.fb[0], e.fb[1], e.fb[2],
                                        e.psd[0], e.psd[1], e.psd[2], e.psd[3]);
                        }
                        fclose(f);
                        record_data_cnt--;
                }

                // Generate sinusoid data for regression
                Matrix<double, Dynamic,9> R(samples,9);
                Matrix<double, Dynamic,3> S(samples,3);
                printf("phase/amp\t");
                for (unsigned int axis=0; axis<3; axis++) {
                        float freq = perturb_freqs[axis];
                        perturb_response resp = find_perturb_response(axis, freq, *inactive_log);
                        printf("%f  %f\t", resp.phase, resp.amp);
                        for (unsigned int i=0; i<samples; i++) {
                                pos_log_entry& ent = (*inactive_log)[i];
                                S(i,axis) = resp.amp * sin(2*M_PI*freq*ent.time + resp.phase);
                        }
                }
                printf("\n");

                // Compute new PSD mean
                Vector4f new_psd_mean = Vector4f::Zero();
                for (unsigned int i=0; i<samples; i++)
                        new_psd_mean += (*inactive_log)[i].psd;
                new_psd_mean /= samples;

                // Pack PSD inputs
                for (unsigned int i=0; i<samples; i++)
                        R.row(i) = pack_psd_inputs((*inactive_log)[i].psd - new_psd_mean).cast<double>();

                // Solve regression
                JacobiSVD<Matrix<double, Dynamic,9> > svd = R.jacobiSvd(ComputeFullU | ComputeFullV);
                Matrix<double, 9,3> bt = svd.solve(S);
                std::cout << "First singular value: " << svd.singularValues()[0] << "\n";
                {
                        std::lock_guard<std::mutex> lock(beta_mutex);
                        beta = bt.transpose().cast<float>();
                        psd_mean = new_psd_mean;
                }

                inactive_log->clear();
                recal_count++;
        }
}

void otf_tracker::feedback()
{
        unsigned int n = 0;
        unsigned int recal_count = 0;
        struct timespec start_time;
        clock_gettime(CLOCK_REALTIME, &start_time);
        float last_report_t = 0;
        unsigned int last_report_n = 0;
        FILE* f = fopen("pos", "w");
        Vector4f psd_mean = Vector4f::Zero();
        std::thread recal_thread(&otf_tracker::recal_worker, this, psd_mean, recal_count);
        Vector3f last_pos = stage_outputs.get_pos();

        _running = true;
        while (!stop) {
                // Get sensor values
                Vector3f fb = stage_outputs.get_pos();
                Vector4f psd = psd_inputs.get();
                psd = scale_psd_position(psd);
                n++;

                // Add datum to log
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                float t = (ts.tv_sec - start_time.tv_sec) +
                        (ts.tv_nsec - start_time.tv_nsec)*1e-9;
                {
                        std::lock_guard<std::mutex> lock(log_mutex);
                        pos_log_entry ent = { t, fb, psd };
                        active_log->add(ent);
                }

                // Compute estimated position
                Vector3f delta;
                {
                        std::lock_guard<std::mutex> lock(beta_mutex);
                        psd -= psd_mean;
                        Matrix<float, 9,1> psd_in = pack_psd_inputs(psd);
                        delta = beta * psd_in;
                }

                if (n % 1 == 0)
                        fprintf(f, "%f  %f  %f      %f  %f  %f     %f  %f  %f\n"
                                 , delta.x(), delta.y(), delta.z()
                                 , fb.x(), fb.y(), fb.z()
                                 , last_pos.x(), last_pos.y(), last_pos.z());

                if (delta.norm() > fb_max_delta) {
                        fprintf(stderr, "Error: Delta exceeded maximum, likely lost tracking\n");
                        delta = Vector3f::Zero();
                }

                // Get PID response
                for (int i=0; i<3; i++) {
                        fb_pids[i].add_point(t, delta[i]);
                        delta[i] = fb_pids[i].get_response();
                }

                // Compute perturbation
                Vector3f perturb;
                for (int i=0; i<3; i++)
                        perturb[i] = perturb_amp[i] * sin(2*M_PI*perturb_freqs[i]*t);

                // Move stage
                if (n % move_skip_cycles == 0) {
                        try {
                                stage_outputs.move(last_pos + delta + perturb);
                                last_pos = last_pos + delta;
                        } catch (clamped_output_error e) {
                                fprintf(stderr, "Clamped\n");
                                continue;
                        }
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

        fclose(f);
        recal_thread.join();
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
        std::lock_guard<std::mutex> lock(log_mutex);
        active_log->resize(len);
        inactive_log->resize(len);
}

void otf_tracker::start_feedback()
{
	stop = false;
        feedback_thread = std::thread(&otf_tracker::feedback, this);
}

bool otf_tracker::running()
{
        return _running;
}

void otf_tracker::stop_feedback()
{
	stop = true;
        feedback_thread.join();
}

otf_tracker::~otf_tracker() {
        stop_feedback();
}

