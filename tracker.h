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
#include "pid.h"
#include "stage.h"

#include <array>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <Eigen/Eigen>

using namespace Eigen;

struct tracker {
        // General parameters
        bool scale_psd_inputs;

        // Rough calibration parameters
        float rough_cal_xy_step, rough_cal_z_step;
        unsigned int rough_cal_xy_pts, rough_cal_z_pts;

        // Fine calibration parameters
        float fine_cal_range;
        unsigned int fine_cal_pts;
        unsigned int fine_cal_pt_delay;

        // Feedback parameters
        unsigned int fb_delay;  	// us
        float fb_max_delta;             // Maximum allowed delta
        bool fb_show_rate;              // Show periodic messages reporting the update rate of the feedback loop
        float fb_rate_report_period;
        array<pid_loop,3> fb_pids;
        Vector3f fb_setpoint;
        
        input_channels<4>& psd_inputs;
        stage& stage_outputs;
        input_channels<3>& fb_inputs;

        struct fine_cal_result {
                Matrix<float, 3,9> beta;
                Vector4f psd_mean;
        };

        boost::function<void()> feedback_ended_cb;

private:
        bool _running;
        boost::thread feedback_thread;
        Vector4f scale_psd_position(Vector4f in);
        void feedback(fine_cal_result cal);

public:
        Vector3f rough_calibrate();
        fine_cal_result fine_calibrate(Vector3f rough_pos);
        void start_feedback(fine_cal_result cal);
        bool running();
        void stop_feedback();

        tracker(input_channels<4>& psd_inputs,
		stage& stage_outputs, input_channels<3>& fb_inputs) :
                scale_psd_inputs(true),
                rough_cal_xy_step(0.01), rough_cal_z_step(0.02),
                rough_cal_xy_pts(20), rough_cal_z_pts(20),
                fine_cal_range(0.02), fine_cal_pts(1000), fine_cal_pt_delay(1000),
                fb_delay(100), fb_max_delta(0.5),
                fb_show_rate(false), fb_rate_report_period(5),
                fb_setpoint(Vector3f::Zero()),
                psd_inputs(psd_inputs),
                stage_outputs(stage_outputs),
                fb_inputs(fb_inputs),
                _running(false)
        {
                fb_pids[0] = pid_loop(0.6, 1e-3, 0, 10);
                fb_pids[1] = pid_loop(0.6, 1e-3, 0, 10);
                fb_pids[2] = pid_loop(1, 0, 0, 1);
        }
};

struct otf_tracker {
        // General parameters
        bool scale_psd_inputs;

        // On-the-fly feedback parameters
        array<float,3> perturb_freqs;
        float perturb_amp;
        unsigned int recal_delay, fb_delay, move_skip_cycles;
        float fb_max_delta;
        Vector3f fb_setpoint;
        array<pid_loop,3> fb_pids;
        bool fb_show_rate;
        float fb_rate_report_period;

        input_channels<4>& psd_inputs;
        stage& stage_outputs;
        input_channels<3>& fb_inputs;

private:
        struct pos_log_entry {
                float time;
                Vector3f fb;
                Vector4f psd;
        };
        // These are where the position data is stored for the on-the-fly calibration
        // There are two ring buffers, one being filled with new data by the
        // feedback loop (the active log) and the other being consumed by the
        // calibration worker (the inactive log).
        boost::circular_buffer<pos_log_entry> *active_log, *inactive_log;
        boost::mutex log_mutex;

        struct perturb_response {
                unsigned int phase;
                float amp;
        };
        perturb_response find_perturb_response(
                        unsigned int axis, float freq,
                        boost::circular_buffer<pos_log_entry>& log_data);

        Vector4f scale_psd_position(Vector4f in);
        void recal_worker(Matrix<float, 3,9>& beta, Vector4f& psd_mean);
        void feedback();
        bool _running;

public:
        boost::function<void()> feedback_ended_cb;
        void start_feedback();
        bool running();
        void stop_feedback();
        unsigned int get_log_length();
        void set_log_length(unsigned int len);

        otf_tracker(input_channels<4>& psd_inputs,
                        stage& stage_outputs, input_channels<3>& fb_inputs) :
                scale_psd_inputs(true),
                perturb_amp(0),
                recal_delay(100*1000), fb_delay(500),
                move_skip_cycles(100),
                fb_max_delta(0.2),
                fb_setpoint(Vector3f::Zero()),
                fb_show_rate(false), fb_rate_report_period(5),
                psd_inputs(psd_inputs),
                stage_outputs(stage_outputs),
                fb_inputs(fb_inputs)
        {
                set_log_length(1000);

                fb_pids[0] = pid_loop(0.6, 1e-3, 0, 10);
                fb_pids[1] = pid_loop(0.6, 1e-3, 0, 10);
                fb_pids[2] = pid_loop(1, 0, 0, 1);

                // Work around apparent gcc bug concerning initializer lists
                perturb_freqs[0] = 67;
                perturb_freqs[1] = 61;
                perturb_freqs[2] = 53;
        }
};

