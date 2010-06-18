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
#include "parameters.h"

#include <cstdint>
#include <readline/readline.h>
#include <readline/history.h>
#include <iostream>
#include <array>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using std::array;
using std::string;

std::vector<parameter*> parameters;

template<typename T>
void def_param(string name, T& value, string description) {
        parameter* p = new typed_value<T>(name, description, value);
        parameters.push_back(p);
}

struct pid_tau_param : parameter {
        pid_loop& pid;
        pid_tau_param(string name, pid_loop& pid, string description) :
                parameter(name, description), pid(pid) { }
        void operator=(string s) {
                pid.set_tau(boost::lexical_cast<unsigned int>(s));
        }
        void put(std::ostream& os) const {
                os << pid.tau();
        }
};

void add_tracker_params(tracker& tracker)
{
        def_param("scale_psd_inputs", tracker.scale_psd_inputs,
                        "Scale PSD positions by sums");

        def_param("rough_cal.xy_step", tracker.rough_cal_xy_step,
                        "Step size of rough calibration raster scan (X and Y axes)");
        def_param("rough_cal.xy_points", tracker.rough_cal_xy_pts,
                        "Number of points in rough calibration raster scan (X and Y axes)");
        def_param("rough_cal.z_step", tracker.rough_cal_z_step,
                        "Step size of rough calibration raster scan (Z axis)");
        def_param("rough_cal.z_points", tracker.rough_cal_z_pts,
                        "Number of points in rough calibration raster scan (Z axis)");

        def_param("fine_cal.range", tracker.fine_cal_range,
                        "Amplitude of fine calibration perturbations");
        def_param("fine_cal.points", tracker.fine_cal_pts,
                        "Number of points in fine calibration scan");
        def_param("fine_cal.point_delay", tracker.fine_cal_pt_delay,
                        "Delay in usec between fine calibration points");

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
        def_param("feedback.setpoint-x", tracker.fb_setpoint.x(),
                        "X axis setpoint");
        def_param("feedback.setpoint-y", tracker.fb_setpoint.y(),
                        "Y axis setpoint");
        def_param("feedback.setpoint-z", tracker.fb_setpoint.z(),
                        "Z axis setpoint");

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
        parameters.push_back(new pid_tau_param("pids.x_tau", tracker.fb_pids[0],
                                "X axis integral time constant"));
        parameters.push_back(new pid_tau_param("pids.y_tau", tracker.fb_pids[1],
                                "Y axis integral time constant"));
        parameters.push_back(new pid_tau_param("pids.z_tau", tracker.fb_pids[2],
                                "Z axis integral time constant"));
        def_param("pids.x_diff", tracker.fb_pids[0].diff_gain,
                        "X axis derivative gain");
        def_param("pids.y_diff", tracker.fb_pids[1].diff_gain,
                        "Y axis derivative gain");
        def_param("pids.z_diff", tracker.fb_pids[2].diff_gain,
                        "Z axis derivative gain");
}

std::string cmd_help =
"Valid commands:\n"
"  set [parameter] [value]      Set a parameter value\n"
"  get [parameter]              Get the value of a parameter\n"
"  list                         List all parameters and their values\n"
"  read-psd                     Read PSD values\n"
"  read-fb                      Read stage feedback sensor values\n"
"  move [x] [y] [z]             Move stage to position (x, y, z)\n"
"  center                       Move stage to position (0.5, 0.5, 0.5)\n"
"  rough-cal                    Run rough calibration\n"
"  fine-cal                     Run fine calibration (requires rough-cal)\n"
"  show-coeffs                  Show fine calibration regression matrix\n"
"  feedback-start               Start feedback (requires fine-cal)\n"
"  feedback-stop                Stop feedback loop\n"
"  exit                         Exit\n"
"  help                         This help message\n";

void feedback_ended() {
        std::cout << "FB-ERR\n";
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

	fb_stage stage(stage_outputs, fb_inputs);
	stage.calibrate();
	stage.move({0.5, 0.5, 0.5});
        def_param("stage.cal_range", stage.cal_range, "Stage calibration range");
	usleep(10*1000);
        tracker tracker(psd_inputs, stage, fb_inputs);
        add_tracker_params(tracker);
        tracker.feedback_ended_cb = &feedback_ended;

        boost::char_separator<char> sep("\t ");
	Eigen::IOFormat mat_fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");
        tracker::fine_cal_result fine_cal;
        Vector3f rough_pos = Vector3f::Zero();
	while (true) {
                char* tmp = readline("> ");
                if (!tmp) break;
                string line = tmp;
                if (line == "" || line[0] == '#') {
                        free(tmp);
                        continue;
                }
                add_history(tmp);
                free(tmp);
                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
		tokenizer tokens(line, sep);
		tokenizer::iterator tok = tokens.begin();

		string cmd = *tok; tok++;
		if (cmd == "set") {
			string param = *tok; tok++;
			string value = *tok;
			parameter* p = find_parameter(parameters, param);
			if (!p)
                                std::cout << "Unknown parameter\n";
                        else {
                                try {
                                        *p = value;
                                } catch (std::exception e) {
                                        std::cout << "ERR\tInvalid value\n";
                                }
                        }
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
                } else if (cmd == "read-psd") {
                        Vector4f psd = psd_inputs.get();
                        std::cout << psd.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "read-fb") {
                        Vector3f fb = fb_inputs.get();
                        std::cout << fb.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "move") {
                        using boost::lexical_cast;
                        Vector3f pos;
                        pos.x() = lexical_cast<float>(*tok); tok++;
                        pos.y() = lexical_cast<float>(*tok); tok++;
                        pos.z() = lexical_cast<float>(*tok);
                        stage.move(pos);
                        std::cout << "OK\n";
                } else if (cmd == "center") {
                        Vector3f pos = 0.5 * Vector3f::Ones();
                        stage.move(pos);
                        std::cout << "OK\n";
                } else if (cmd == "stage-cal") {
                        stage.calibrate();
                        std::cout << "OK\n";
                } else if (cmd == "rough-cal") {
                        rough_pos = tracker.rough_calibrate();
                        stage.move(rough_pos);
                        std::cout << rough_pos.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "fine-cal") {
                        fine_cal = tracker.fine_calibrate(rough_pos);
                        stage.move(rough_pos);
                } else if (cmd == "show-coeffs") {
                        std::cout << fine_cal.beta.format(mat_fmt) << "\n";
                } else if (cmd == "feedback-start") {
                        if (tracker.running())
                                std::cout << "ERR\tAlready running\n";
                        else {
                                tracker.start_feedback(fine_cal);
                                std::cout << "OK\tFeedback running\n";
                        }
                } else if (cmd == "feedback-stop") {
                        if (!tracker.running())
                                std::cout << "ERR\tNot running\n";
                        else {
                                tracker.stop_feedback();
                                std::cout << "OK\tFeedback stopped\n";
                        }
                } else if (cmd == "pause") {
                        std::cout << "Press enter when ready\n";
                        getchar();
                } else if (cmd == "wait") {
                        float time = boost::lexical_cast<float>(*tok);
                        usleep(time * 1e6);
                } else if (cmd == "exit" || cmd == "quit") {
                        return 0;
                } else if (cmd == "help") {
                        std::cout << "Valid Commands:\n";
                        std::cout << cmd_help << "\n";
		} else
			std::cout << "ERR\tInvalid command\n";
	}

	return 0;
}

