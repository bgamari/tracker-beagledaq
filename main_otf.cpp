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

#include "version.h"
#include "otf_tracker.h"
#include "parameters.h"
#include "config.h"

#include <cstdint>
#include <readline/readline.h>
#include <readline/history.h>
#include <iostream>
#include <array>

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
                unsigned int a;
                std::istringstream(s) >> a;
                pid.set_tau(a);
        }
        void put(std::ostream& os) const {
                os << pid.tau();
        }
};

void add_tracker_params(otf_tracker& tracker)
{
        def_param("scale_psd_inputs", tracker.scale_psd_inputs,
                        "Scale PSD positions by sums");

        def_param("otf.freq_x", tracker.perturb_freqs[0],
                        "Frequencies of on-the-fly calibration perturbations (X axis)");
        def_param("otf.freq_y", tracker.perturb_freqs[1],
                        "Frequencies of on-the-fly calibration perturbations (Y axis)");
        def_param("otf.freq_z", tracker.perturb_freqs[2],
                        "Frequencies of on-the-fly calibration perturbations (Z axis)");
	def_param("cal.perturb_amp_x", tracker.perturb_amp[0],
			"Perturbation amplitude (X axis)");
	def_param("cal.perturb_amp_y", tracker.perturb_amp[1],
			"Perturbation amplitude (Y axis)");
	def_param("cal.perturb_amp_z", tracker.perturb_amp[2],
			"Perturbation amplitude (Z axis)");
	def_param("cal.delay", tracker.recal_delay,
			"Calibration delay");

        def_param("feedback.delay", tracker.fb_delay,
                        "Delay between feedback loop iterations");
	def_param("feedback.move_skip_cycles", tracker.move_skip_cycles,
			"Number of feedback cycles to skip between moves");
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
	init_hardware();
	pid_stage stage(*stage_out, *stage_in);
	stage.smooth_move({0.5, 0.5, 0.5}, 10000);

	usleep(10*1000);
        otf_tracker tracker(*psd_in, stage);
        add_tracker_params(tracker);
        tracker.feedback_ended_cb = &feedback_ended;

	Eigen::IOFormat mat_fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");

        std::cout << "Tracker " << version << "\n";
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

		std::istringstream ss(line);
		string cmd;
		ss >> cmd;	

		if (cmd == "set") {
                        string param, value;
                        ss >> param; ss >> value;
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
                        string param;
                        ss >> param;
			parameter* p = find_parameter(parameters, param);
			if (!p)
                                std::cout << "Unknown parameter\n";
                        else
                                std::cout << param << " = " << *p << "\n";
		} else if (cmd == "list") {
			for (auto p=parameters.begin(); p != parameters.end(); p++)
				std::cout << (**p).name << " = " << **p << "\n";
                } else if (cmd == "read-psd") {
                        Vector4f psd = psd_in->get();
                        std::cout << psd.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "read-pos") {
                        Vector3f fb = stage.get_pos();
                        std::cout << fb.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "move") {
                        Vector3f pos;
                        ss >> pos.x();
                        ss >> pos.y();
                        ss >> pos.z();
                        stage.smooth_move(pos, 10000);
                        std::cout << "OK\n";
                } else if (cmd == "center") {
                        Vector3f pos = 0.5 * Vector3f::Ones();
                        stage.smooth_move(pos, 10000);
                        std::cout << "OK\n";
                } else if (cmd == "feedback-start") {
                        if (tracker.running())
                                std::cout << "ERR\tAlready running\n";
                        else {
                                tracker.start_feedback();
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
                } else if (cmd == "exit" || cmd == "quit") {
                        return 0;
                } else if (cmd == "help") {
                        std::cout << "Valid Commands:\n";
                        std::cout << cmd_help << "\n";
                } else if (cmd == "version") {
                        std::cout << branch << "\t" << version << "\n";
		} else
			std::cout << "ERR\tInvalid command\n";
	}

	return 0;
}

