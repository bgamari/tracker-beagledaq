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
#include "bitfield.h"
#include "max5134.h"
#include "max1302.h"
#include "tracker.h"
#include "parameters.h"
#include "utils.h"
#include "config.h"

#include <cstdint>
#include <readline/readline.h>
#include <readline/history.h>
#include <iostream>
#include <array>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using std::array;
using std::string;

static Eigen::IOFormat mat_fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");

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

class invalid_syntax : std::exception { };

static std::string cmd_help =
"Valid commands:\n"
"  source [file]                Read commands from file\n"
"  set [parameter] [value]      Set a parameter value\n"
"  get [parameter]              Get the value of a parameter\n"
"  list                         List all parameters and their values\n"
"  read-psd                     Read PSD values\n"
"  read-fb                      Read stage feedback sensor values\n"
"  move [x] [y] [z]             Move stage to position (x, y, z)\n"
"  move-rough-pos               Move stage to rough position\n"
"  center                       Move stage to position (0.5, 0.5, 0.5)\n"
"  stage-cal                    Run stage calibration\n"
"  rough-cal                    Run rough calibration\n"
"  fine-cal                     Run fine calibration (requires rough-cal)\n"
"  show-coeffs                  Show fine calibration regression matrix\n"
"  feedback-start               Start feedback (requires fine-cal)\n"
"  feedback-stop                Stop feedback loop\n"
"  scan                         Run manual scan (configure with scan.* parameters)\n"
"  exit                         Exit\n"
"  version                      Show version information\n"
"  help                         This help message\n";

struct tracker_cli {
        std::vector<parameter*> parameters;
        input_channels<4>& psd_inputs;
        input_channels<3>& fb_inputs;
        output_channels<3>& stage_outputs;
        fb_stage stage;
        tracker tr;
        Vector3f rough_pos;
        tracker::fine_cal_result fine_cal;
        Vector3f scan_center, scan_range;
        Vector3u scan_points;
        unsigned int scan_delay;

        template<typename T>
        void def_param(string name, T& value, string description) {
                parameter* p = new typed_value<T>(name, description, value);
                parameters.push_back(p);
        }

        void add_tracker_params(tracker& tracker)
        {
                def_param("scale_psd_inputs", tracker.scale_psd_inputs,
                                "Scale PSD positions by sums");

                def_param("rough_cal.xy_range", tracker.rough_cal_xy_range,
                                "Scan size of rough calibration raster scan (X/Y scan");
                def_param("rough_cal.xy_points", tracker.rough_cal_xy_pts,
                                "Number of points in rough calibration raster scan (X/Y scan");
                def_param("rough_cal.xy_dwell", tracker.rough_cal_xy_dwell,
                                "Rough calibration point dwell time (X/Y scan)");
                def_param("rough_cal.z_range", tracker.rough_cal_z_range,
                                "Scan size of rough calibration raster scan (Z scan)");
                def_param("rough_cal.z_points", tracker.rough_cal_z_pts,
                                "Number of points in rough calibration raster scan (Z scan)");
                def_param("rough_cal.z_dwell", tracker.rough_cal_z_dwell,
                                "Rough calibration point dwell time (Z scan)");
                def_param("rough_cal.z_avg_win", tracker.rough_cal_z_avg_win,
                                "Averaging window for filtering of Z axis rough calibration data");

                def_param("fine_cal.xy_range", tracker.fine_cal_xy_range,
                                "Amplitude of fine calibration perturbations (X and Y axes)");
                def_param("fine_cal.z_range", tracker.fine_cal_z_range,
                                "Amplitude of fine calibration perturbations (Z axis)");
                def_param("fine_cal.points", tracker.fine_cal_pts,
                                "Number of points in fine calibration scan");
                def_param("fine_cal.point_delay", tracker.fine_cal_pt_delay,
                                "Delay in usec between fine calibration points");

                def_param("feedback.delay", tracker.fb_delay,
                                "Delay between feedback loop iterations");
                def_param("feedback.max_delta", tracker.fb_max_delta,
                                "Maximum allowed position change during feedback");
                def_param("feedback.show_rate", tracker.fb_show_rate,
                                "Report on feedback loop iteration rate");
                def_param("feedback.setpoint_x", tracker.fb_setpoint.x(),
                                "X axis setpoint");
                def_param("feedback.setpoint_y", tracker.fb_setpoint.y(),
                                "Y axis setpoint");
                def_param("feedback.setpoint_z", tracker.fb_setpoint.z(),
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

        static void feedback_ended() {
                std::cout << "$ FB-ERR\n";
        }

        tracker_cli(input_channels<4>& psd_inputs,
                        input_channels<3>& fb_inputs,
                        output_channels<3>& stage_outputs) :
                psd_inputs(psd_inputs), fb_inputs(fb_inputs), stage_outputs(stage_outputs),
                stage(stage_outputs, fb_inputs),
                tr(psd_inputs, stage, fb_inputs),
                scan_delay(100)
        {
                stage.calibrate();
                stage.smooth_move({0.5, 0.5, 0.5}, 10000);
                def_param("stage.cal_range", stage.cal_range, "Stage calibration range");
                usleep(10*1000);

                add_tracker_params(tr);
                tr.feedback_ended_cb = &tracker_cli::feedback_ended;

                rough_pos << 0.5, 0.5, 0.5;
                def_param("rough_pos.x", rough_pos.x(), "Rough calibration position (X axis)");
                def_param("rough_pos.y", rough_pos.y(), "Rough calibration position (Y axis)");
                def_param("rough_pos.z", rough_pos.z(), "Rough calibration position (Z axis)");

                scan_center << 0.5, 0.5, 0.5;
                scan_range << 0.1, 0.1, 0.1;
                scan_points << 100, 100, 100;
                def_param("scan.center_x", scan_center.x(), "Center of manual scan (X axis)");
                def_param("scan.center_y", scan_center.y(), "Center of manual scan (Y axis)");
                def_param("scan.center_z", scan_center.z(), "Center of manual scan (Z axis)");
                def_param("scan.range_x", scan_range.x(), "Range of manual scan (X axis)");
                def_param("scan.range_y", scan_range.y(), "Range of manual scan (Y axis)");
                def_param("scan.range_z", scan_range.z(), "Range of manual scan (Z axis)");
                def_param("scan.points_x", scan_points.x(), "Number of points in manual scan (X axis)");
                def_param("scan.points_y", scan_points.y(), "Number of points in manual scan (Y axis)");
                def_param("scan.points_z", scan_points.z(), "Number of points in manual scan (Z axis)");
                def_param("scan.delay", scan_delay, "Delay between points in manual scan");

                std::cout << "Tracker " << version << "\n";
        }

        void source(string file) 
        {
                std::ifstream is(file);
                while (is.good()) {
                        string line;
                        std::getline(is, line);
                        try {
                                do_command(line);
                        } catch (std::exception e) {
                                std::cout << "! Command failed\n";
                        }
                }
        }

        bool do_command(string cmdline)
        {
                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> sep("\t ");
		tokenizer tokens(cmdline, sep);
		tokenizer::iterator tok = tokens.begin();

		string cmd = *tok; tok++;
                vector<string> args;
                for (; tok != tokens.end(); tok++)
                        args.push_back(*tok);

                if (cmd == "source") {
                        if (args.size() != 1) throw invalid_syntax();
                        source(args[0]);
                } else if (cmd == "set") {
                        if (args.size() != 2) throw invalid_syntax();
			string param = args[0];
			string value = args[1];
			parameter* p = find_parameter(parameters, param);
			if (!p)
                                std::cout << "! Unknown parameter\n";
                        else {
                                try {
                                        *p = value;
                                } catch (std::exception e) {
                                        std::cout << "! ERR\tInvalid value\n";
                                }
                        }
		} else if (cmd == "get") {
                        if (args.size() != 1) throw invalid_syntax();
			string param = args[0];
			parameter* p = find_parameter(parameters, param);
			if (!p)
                                std::cout << "! Unknown parameter\n";
                        else
                                std::cout << param << " = " << *p << "\n";
		} else if (cmd == "list") {
                        string match = args.size() ? args[0] : "";
			for (auto p=parameters.begin(); p != parameters.end(); p++)
                                if ((**p).name.compare(0, match.size(), match) == 0)
                                        std::cout << (boost::format("%-30s\t%10s\t\t%50s\n") % (**p).name % **p % (**p).description);
                } else if (cmd == "read-psd") {
                        Vector4f psd = psd_inputs.get();
                        std::cout << psd.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "read-fb") {
                        Vector3f fb = fb_inputs.get();
                        std::cout << fb.transpose().format(mat_fmt) << "\n";
                } else if (cmd == "move") {
                        using boost::lexical_cast;
                        if (args.size() != 3) throw invalid_syntax();
                        Vector3f pos;
                        pos.x() = lexical_cast<float>(args[0]);
                        pos.y() = lexical_cast<float>(args[1]);
                        pos.z() = lexical_cast<float>(args[2]);
                        stage.smooth_move(pos, 10000);
                        std::cout << "! OK\n";
                } else if (cmd == "move-rough-pos") {
                        stage.smooth_move(rough_pos, 10000);
                        std::cout << "! OK\n";
                } else if (cmd == "center") {
                        Vector3f pos = 0.5 * Vector3f::Ones();
                        stage.smooth_move(pos, 10000);
                        std::cout << "! OK\n";
                } else if (cmd == "stage-cal") {
                        stage.calibrate();
                        std::cout << "! OK\n";
                } else if (cmd == "rough-cal") {
                        try {
                                rough_pos = tr.rough_calibrate();
                                stage.smooth_move(rough_pos, 5000);
                                std::cout << rough_pos.transpose().format(mat_fmt) << "\n";
                        } catch (clamped_output_error e) {
                                std::cout << "! ERR Clamped output\n";
                        }
                } else if (cmd == "fine-cal") {
                        fine_cal = tr.fine_calibrate(rough_pos);
                        stage.smooth_move(rough_pos, 1000);
                        std::cout << "! OK\n";
                } else if (cmd == "show-coeffs") {
                        std::cout << fine_cal.beta.format(mat_fmt) << "\n";
                } else if (cmd == "feedback-start") {
                        if (tr.running())
                                std::cout << "! ERR\tAlready running\n";
                        else {
                                tr.start_feedback(fine_cal);
                                std::cout << "! OK\tFeedback running\n";
                        }
                } else if (cmd == "feedback-stop") {
                        if (!tr.running())
                                std::cout << "ERR\tNot running\n";
                        else {
                                tr.stop_feedback();
                                std::cout << "OK\tFeedback stopped\n";
                        }
                } else if (cmd == "exit") {
                        return true;
                } else if (cmd == "help") {
                        std::cout << cmd_help << "\n";
                } else if (cmd == "version") {
                        std::cout << branch << "\t" << version << "\n";
                } else if (cmd == "scan") {
                        Vector3f start = scan_center - scan_range / 2;
                        Vector3f step = scan_range.array() / scan_points.array().cast<float>();
                        raster_route r(start, step, scan_points);
                        collect_cb<4> psd_data(psd_inputs);
                        collect_cb<3> fb_data(fb_inputs);
                        execute_route(stage, r, {&psd_data, &fb_data}, scan_delay);
                        dump_data("scan", fb_data.data, psd_data.data);
		} else
			std::cout << "! ERR\tInvalid command\n";
                return false;
        }

        void mainloop() {
                bool stop = false;
                while (!stop) {
                        char* tmp = readline("> ");
                        if (!tmp) break;
                        string line = tmp;
                        if (line == "" || line[0] == '#') {
                                free(tmp);
                                continue;
                        }
                        add_history(tmp);
                        free(tmp);
                        try {
                                stop = do_command(line);
                        } catch (std::exception& e) {
                                std::cout << "! Command failed\n";
                        } catch (invalid_syntax e) {
                                std::cout << "! Invalid syntax\n";
                        }
                }
        }
};

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

        tracker_cli cli(psd_inputs, fb_inputs, stage_outputs);
        cli.mainloop();

	return 0;
}

