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

#include "bitfield.h"
#include "max5134.h"
#include "max1302.h"
#include "tracker.h"
#include "config.h"

#include <cstdint>
#include <readline/readline.h>
#include <readline/history.h>
#include <iostream>
#include <array>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>

using std::array;
using std::string;

void dump_data_test(input_channels<4>& psd_inputs, input_channels<1>& pd_input,
		stage& stage, input_channels<3>& fb_inputs)
{
	printf("# psd_x psd_y sum_x sum_y\tfb_x fb_y fb_z\tpd\n");
	unsigned int n=0;
	while (true) {
		Vector4f psd = psd_inputs.get();
#define SCALE_INPUTS 1
#if SCALE_INPUTS
                psd.x() /= psd[2];
                psd.y() /= psd[3];
#endif
		Vector3f fb = fb_inputs.get();
                Matrix<float,1,1> pd = pd_input.get();
		for (int i=0; i<4; i++) printf("%f ", psd[i]);
		printf("\t");
		for (int i=0; i<3; i++) printf("%f ", fb[i]);
		printf("\t%f\n", pd[0]);

		usleep(1000*10);
		n++;
                Vector3f pos;
                pos << 0.5 + 0.2*sin(0.01*n), 0.5, 0.5;
                //stage.move(pos);
	}
}

void start_tracker(tracker& tracker)
{
        tracker.track();
}

std::vector<parameter*> parameters;

template<typename T>
void def_param(string name, T& value, string description) {
        parameter* p = new typed_value<T>(name, description, value);
        parameters.push_back(p);
}

void add_tracker_params(tracker& tracker)
{
        def_param("scale_psd_inputs", tracker.scale_psd_inputs,
                        "Scale PSD positions by sums");

        def_param("rough_cal.xy_step", tracker.rough_cal_xy_step,
                        "Step size of rough calibration raster scan (X and Y axes)");
        def_param("rough_cal.xy_points", tracker.rough_cal_xy_pts,
                        "Number of points in rough calibration raster scan (X and Y axes)");
        def_param("rough_cal.z_step", tracker.rough_cal_z_step,
                        "Step size of rough calibration raster scan (X and Y axes)");
        def_param("rough_cal.xy_points", tracker.rough_cal_xy_pts,
                        "Number of points in rough calibration raster scan (X and Y axes)");

        def_param("fine_cal.range", tracker.fine_cal_range,
                        "Amplitude of fine calibration perturbations");
        def_param("fine_cal.points", tracker.fine_cal_pts,
                        "Number of points in fine calibration scan");

        def_param("otf.freq-x", tracker.otf_freqs[0],
                        "Frequencies of on-the-fly calibration perturbations (X axis)");
        def_param("otf.freq-y", tracker.otf_freqs[1],
                        "Frequencies of on-the-fly calibration perturbations (Y axis)");
        def_param("otf.freq-z", tracker.otf_freqs[2],
                        "Frequencies of on-the-fly calibration perturbations (Z axis)");
        def_param("otf.amp", tracker.otf_amp,
                        "Amplitude of on-the-fly calibration perturbations");

        def_param("feedback.delay", tracker.fb_delay,
                        "Delay between feedback loop iterations");
        def_param("feedback.max_delta", tracker.fb_max_delta,
                        "Maximum allowed position change during feedback");
        def_param("feedback.show_rate", tracker.fb_show_rate,
                        "Report on feedback loop iteration rate");

        def_param("pids.x_prop", tracker.fb_pids[0].prop_gain,
                        "X axis proportional gain");
        def_param("pids.y_prop", tracker.fb_pids[1].prop_gain,
                        "Y axis proportional gain");
        def_param("pids.z_prop", tracker.fb_pids[2].prop_gain,
                        "Z axis proportional gain");
        def_param("pids.x_int", tracker.fb_pids[0].int_gain,
                        "X axis integral gain");
        def_param("pids.y_int", tracker.fb_pids[1].int_gain,
                        "Y axis integral gain");
        def_param("pids.z_int", tracker.fb_pids[2].int_gain,
                        "Z axis integral gain");
        def_param("pids.x_diff", tracker.fb_pids[0].diff_gain,
                        "X axis derivative gain");
        def_param("pids.y_diff", tracker.fb_pids[1].diff_gain,
                        "Y axis derivative gain");
        def_param("pids.z_diff", tracker.fb_pids[2].diff_gain,
                        "Z axis derivative gain");
}

int main(int argc, char** argv)
{
//#define TEST
#ifndef TEST
	max1302 psd_adc(psd_adc_dev);
	max1302 fb_adc(fb_adc_dev);
	max1302_inputs<4> psd_inputs(psd_adc, psd_chans, max1302::SE_MINUS_VREF_PLUS_VREF);
	max1302_inputs<3> fb_inputs(fb_adc, fb_chans, max1302::SE_ZERO_PLUS_VREF);
        max1302_inputs<1> pd_input(fb_adc, pd_chans, max1302::SE_ZERO_PLUS_VREF);

	max5134 dac(stage_pos_dac_dev);
	max5134_outputs<3> stage_outputs(dac, stage_chans);
#else	
	test_inputs<4> psd_inputs;
	test_inputs<3> fb_inputs;
	test_outputs<3> stage_outputs;
#endif

	stage stage(stage_outputs, fb_inputs);
	stage.calibrate();
	stage.move({0.5, 0.5, 0.5});
        tracker tracker(psd_inputs, stage, fb_inputs);
        add_tracker_params(tracker);
	usleep(10*1000);
	Vector3f fb = fb_inputs.get();
	fprintf(stderr, "Feedback position: %f %f %f\n", fb.x(), fb.y(), fb.z());

        boost::thread* tracker_thread = NULL;
        boost::char_separator<char> sep("\t ");
	while (true) {
                char* tmp = readline("> ");
                string line = tmp;
                if (line != "")
                        add_history(tmp);
                free(tmp);
	        std::getline(std::cin, line);
                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		tokenizer tokens(line);
		tokenizer::iterator tok = tokens.begin();

		string cmd = *tok; tok++;
		if (cmd == "set") {
			string param = *tok; tok++;
			string value = *tok;
			parameter* p = find_parameter(parameters, param);
			if (!p)
                                std::cout << "Unknown parameter\n";
                        else
                                *p = value;
		} else if (cmd == "get") {
			string param = *tok;
			parameter* p = find_parameter(parameters, param);
			if (!p)
                                std::cout << "Unknown parameter\n";
                        else
                                std::cout << param << " = " << *p << "\n";
		} else if (cmd == "list") {
			for (auto p=parameters.begin(); p != parameters.end(); p++)
				std::cout << (**p).name << " = " << **p << "\n";
                } else if (cmd == "start-tracking") {
                        if (tracker_thread)
                                std::cout << "Already tracking\n";
                        else
                                tracker_thread = new boost::thread(start_tracker,tracker);
                } else if (cmd == "stop-tracking") {
                        if (!tracker_thread)
                                std::cout << "Not tracking\n";
                        else {
                                tracker_thread->interrupt();
                                tracker_thread = NULL;
                        }
                } else if (cmd == "exit" || cmd == "quit") {
                        exit(0);
                } else if (cmd == "help") {
                        std::cout << "Valid Commands:\n";
		} else
			std::cout << "Invalid command\n";
	}

	fprintf(stderr, "Position bead. Press any key.\n");
	getchar();
	Vector4f psd = psd_inputs.get();
	fprintf(stderr, "PSD: pos=(%f, %f); sum=(%f, %f)\n",
			psd[0], psd[1], psd[2], psd[3]);
	tracker.track();
	return 0;
}

