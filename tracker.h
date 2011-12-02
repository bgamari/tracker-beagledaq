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
#include <tr1/functional>
#include <thread>
#include <Eigen/Eigen>

using namespace Eigen;

// General
extern bool scale_psd_inputs;

Vector4f scale_psd_position(Vector4f in);

// Rough Calibration
struct rough_cal_params {
        float xy_range, z_range;
        unsigned int xy_npts, z_npts;
        unsigned int xy_dwell, z_dwell;
        unsigned int z_avg_window;
};

struct rough_cal_result {
	Vector3f center;
	float xy_size, z_size;
};

rough_cal_result rough_calibrate( stage& stage
                                , input_channels<4>& psd
				, rough_cal_params& params
		                , Vector3f center);

// Fine calibration
struct fine_cal_params {
        float xy_range, z_range;
        unsigned int npts, dwell;
};

struct fine_cal_result {
	Matrix<float, 3,9> beta;
	Vector4f psd_mean;
	double max_singular_value;
};

fine_cal_result fine_calibrate( stage& stage
                              , input_channels<4>& psd
		              , fine_cal_params& params
			      , Vector3f rough_pos);

// Feedback
struct feedback_params {
        unsigned int delay;          // us
        float max_delta;             // Maximum allowed delta
        bool show_rate;              // Show periodic messages reporting the
                                     // update rate of the feedback loop
        float rate_report_period;
        array<pid_loop,3> pids;
        Vector3f setpoint;
        float min_sing_value;        // Minimum singular value necessary to use feedback signal

        // On-the-fly recalibration
        unsigned int recal_delay;    // Delay in us between recalibrations
                                     // 0 disables recalibration
        float recal_weight;          // Amount to weigh recalibration by
        unsigned int min_recal_samples; // Minimum number of samples to require for recalibration
        Vector3f perturb_freqs;
        Vector3f perturb_amp;
        double phase_max;            // Maximum compute to probe in cross-correlation
        double phase_step;
};

struct feedback {
        input_channels<4>& psd;
        input_channels<3>& fb;
        stage& _stage;
        std::mutex cal_mutex;
        fine_cal_result& cal;
        feedback_params& params;

        std::function<void()> feedback_ended_cb;

private:
        bool _running, _stop;
        std::thread worker;
        void loop();

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
        ring_buffer<pos_log_entry> *active_log, *inactive_log;
        std::mutex log_mutex;

        struct perturb_response {
                float phase, amp;
        };
        perturb_response find_perturb_response(
                unsigned int axis, float freq,
                ring_buffer<pos_log_entry>& log_data);
        void recal();
        std::thread* recal_worker;

public:
        void start();
        bool running();
        void stop();

        void set_log_length(unsigned int length);
        unsigned int get_log_length();

        feedback( input_channels<4>& psd
                , input_channels<3>& fb
                , stage& _stage
                , feedback_params& params
                , fine_cal_result& cal
                , unsigned int log_length=10000
                )
                : psd(psd)
                , fb(fb)
                , _stage(_stage)
                , cal(cal)
                , params(params)
                , _running(false)
                , recal_worker(NULL)
        {
                active_log = new ring_buffer<pos_log_entry>(log_length);
                inactive_log = new ring_buffer<pos_log_entry>(log_length);
        }

        ~feedback();
};

