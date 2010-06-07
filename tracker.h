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

#include <cstdint>
#include <array>
#include <Eigen/Eigen>
#include <boost/program_options.hpp>

using namespace Eigen;

class stage {
	output_channels<3>& out;
	input_channels<3>& fb;
        float cal_range;
	Vector3f last_pos;
	Matrix<float, 4,3> R;

public:
	stage(output_channels<3>& out, input_channels<3>& fb, float cal_range=0.2)
		: out(out), fb(fb), cal_range(cal_range) { }

	/*
	 * calibrate():
	 * Perform basic first-order OLS regression to map feedback coordinate
	 * space to stage input space
	 */
	void calibrate(unsigned int n_pts=10);
	void move(const Vector3f pos);
	Vector3f get_last_pos();
};

struct tracker {
        // General parameters
        bool scale_psd_inputs;

        // Rough calibration parameters
        float rough_cal_xy_step, rough_cal_z_step;
        unsigned int rough_cal_xy_pts, rough_cal_z_pts;

        // Fine calibration parameters
        float fine_cal_range;
        unsigned int fine_cal_pts;

        // On-the-fly calibration parameters
        array<float,3> otf_freqs;
        float otf_amp;

        // Feedback parameters
        unsigned int fb_delay;  	// us
        float fb_max_delta;             // Maximum allowed delta
        bool fb_show_rate;              // Show periodic messages reporting the update rate of the feedback loop
        array<pid_loop,3> fb_pids;
        
        input_channels<4>& psd_inputs;
        stage& stage_outputs;
        input_channels<3>& fb_inputs;

private:
        Vector4f scale_psd_position(Vector4f in);

public:
        Vector3f rough_calibrate();
        Matrix<float, 3,10> fine_calibrate(Vector3f rough_pos);
        void feedback(Matrix<float,3,10> R);

        tracker(input_channels<4>& psd_inputs,
		stage& stage_outputs, input_channels<3>& fb_inputs) :
                scale_psd_inputs(false),
                rough_cal_xy_step(0.01), rough_cal_z_step(0.02),
                rough_cal_xy_pts(20), rough_cal_z_pts(20),
                fine_cal_range(0.02), fine_cal_pts(1000),
                otf_amp(0.01),
                fb_delay(100), fb_max_delta(0.5), fb_show_rate(false),
                psd_inputs(psd_inputs),
                stage_outputs(stage_outputs),
                fb_inputs(fb_inputs)
        {
                // Work around apparent gcc bug concerning initializer lists
                otf_freqs[0] = 67;
                otf_freqs[1] = 61;
                otf_freqs[2] = 53;

                fb_pids[0] = pid_loop(0.6, 1e-2, 0, 10);
                fb_pids[1] = pid_loop(0.55, 1e-3, 0e-5, 10);
                fb_pids[2] = pid_loop(1, 0, 0, 1);
        }

        void track();
};

