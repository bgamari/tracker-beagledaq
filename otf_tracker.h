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
        boost::thread feedback_thread;
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
                scale_psd_inputs(false),
                perturb_amp(0.05),
                recal_delay(100*1000), fb_delay(500),
                move_skip_cycles(10),
                fb_max_delta(0.2),
                fb_setpoint(Vector3f::Zero()),
                fb_show_rate(false), fb_rate_report_period(5),
                psd_inputs(psd_inputs),
                stage_outputs(stage_outputs),
                fb_inputs(fb_inputs),
		_running(false)
        {
		active_log = new boost::circular_buffer<pos_log_entry>(1000);
		inactive_log = new boost::circular_buffer<pos_log_entry>(1000);

                fb_pids[0] = pid_loop(0.6, 1e-3, 0, 10);
                fb_pids[1] = pid_loop(0.6, 1e-3, 0, 10);
                fb_pids[2] = pid_loop(0.1, 0, 0, 10);

                // Work around apparent gcc bug concerning initializer lists
                perturb_freqs[0] = 67;
                perturb_freqs[1] = 61;
                perturb_freqs[2] = 53;
        }
};

