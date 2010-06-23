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
        float rough_cal_xy_range, rough_cal_z_range;
        unsigned int rough_cal_xy_pts, rough_cal_z_pts;
        unsigned int rough_cal_xy_dwell, rough_cal_z_dwell;
        unsigned int rough_cal_z_avg_win;

        // Fine calibration parameters
        float fine_cal_xy_range, fine_cal_z_range;
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
        Vector3f rough_calibrate(Vector3f center=0.5*Vector3f::Ones());
        fine_cal_result fine_calibrate(Vector3f rough_pos);
        void start_feedback(fine_cal_result cal);
        bool running();
        void stop_feedback();

        tracker(input_channels<4>& psd_inputs,
		stage& stage_outputs, input_channels<3>& fb_inputs) :
                scale_psd_inputs(false),
                rough_cal_xy_range(0.4), rough_cal_z_range(0.6),
                rough_cal_xy_pts(40), rough_cal_z_pts(200),
                rough_cal_xy_dwell(1000), rough_cal_z_dwell(1000),
                rough_cal_z_avg_win(5),
                fine_cal_xy_range(0.008), fine_cal_z_range(0.02), 
                fine_cal_pts(1000), fine_cal_pt_delay(1000),
                fb_delay(2000), fb_max_delta(0.05),
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

