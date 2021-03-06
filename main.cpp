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
#include "tracker.h"
#include "parameters.h"
#include "utils.h"
#include "config.h"

#include <cstdint>
#include <readline/readline.h>
#include <readline/history.h>
#include <sstream>
#include <iostream>
#include <array>

using std::array;
using std::string;

static Eigen::IOFormat mat_fmt = Eigen::IOFormat(Eigen::FullPrecision, 0, "\t", "\n");

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

class invalid_syntax : std::exception { };

static std::string cmd_help =
"Valid commands:\n"
"  source [file]                Read commands from file\n"
"  set [parameter] [value]      Set a parameter value\n"
"  get [parameter]              Get the value of a parameter\n"
"  list                         List all parameters and their values\n"
"  read-psd                     Read PSD values\n"
"  read-pos                     Read stage position\n"
"  move [x] [y] [z]             Move stage to position (x, y, z)\n"
"  move-rough-pos               Move stage to rough position\n"
"  center                       Move stage to position (0.5, 0.5, 0.5)\n"
"  rough-cal                    Run rough calibration\n"
"  fine-cal                     Run fine calibration (requires rough-cal)\n"
"  show-coeffs                  Show fine calibration regression matrix\n"
"  save-coeffs [file]           Save fine calibration regression matrix from file\n"
"  load-coeffs [file]           Load fine calibration regression matrix from file\n"
"  feedback-start               Start feedback (requires fine-cal)\n"
"  feedback-stop                Stop feedback loop\n"
"  scan [file]                  Run a manual scan (see scan.* parameters)\n"
"  wait [n]                     Wait n milliseconds\n"
"  exit                         Exit\n"
"  version                      Show version information\n"
"  help                         This help message\n";

template<typename T>
void def_param(std::vector<parameter*>& params, string name, T& value, string description) {
        parameter* p = new typed_value<T>(name, description, value);
        params.push_back(p);
}

void add_pid_params(std::vector<parameter*>& params, string name, pid_loop& pid)
{
        def_param(params, name + ".prop", pid.prop_gain, "PID Proportional gain");
        def_param(params, name + ".int", pid.int_gain, "PID Integral gain");
        params.push_back(new pid_tau_param(name + ".tau", pid, "PID Integral time constant"));
        def_param(params, name + ".diff", pid.diff_gain, "PID Differential gain");
}

template<class Stage>
void add_stage_params(std::vector<parameter*>& params, Stage& s)
{
        throw std::runtime_error("Stage parameters unimplemented for given stage\n");
}

template<>
void add_stage_params<fb_stage>(std::vector<parameter*>& params, fb_stage& s)
{ }

template<>
void add_stage_params<pid_stage>(std::vector<parameter*>& params, pid_stage& s)
{
        add_pid_params(params, "stage.pid.x", s.pidx);
        add_pid_params(params, "stage.pid.y", s.pidy);
        add_pid_params(params, "stage.pid.z", s.pidz);
        def_param(params, "stage.delay", s.fb_delay, "Stage feedback loop delay");
}

template<class Stage>
struct tracker_cli {
        std::vector<parameter*> params;
        input_channels<4>& psd_inputs;
        Stage& _stage;
        stage& fb_stage;
        Vector3f rough_pos;

        rough_cal_params rough_params;
        fine_cal_params fine_params;
        feedback_params fb_params;
        fine_cal_result fine_cal;
        feedback* fb;

        float auto_xy_range_factor;
        Vector3f scan_center, scan_range;
        Vector3u scan_points;
        unsigned int scan_delay;

        template<typename T>
        void def_param(string name, T& value, string description) {
                parameter* p = new typed_value<T>(name, description, value);
                params.push_back(p);
        }

        void add_tracker_params()
        {
                def_param("scale_psd_inputs", scale_psd_inputs,
                                "Scale PSD positions by sums");

                def_param("rough_cal.xy.range", rough_params.xy_range,
                                "Scan size of rough calibration raster scan (X/Y scan");
                def_param("rough_cal.xy.points", rough_params.xy_npts,
                                "Number of points in rough calibration raster scan (X/Y scan");
                def_param("rough_cal.xy.dwell", rough_params.xy_dwell,
                                "Rough calibration point dwell time (X/Y scan)");

                def_param("rough_cal.z.range", rough_params.z_range,
                                "Scan size of rough calibration raster scan (Z scan)");
                def_param("rough_cal.z.points", rough_params.z_npts,
                                "Number of points in rough calibration raster scan (Z scan)");
                def_param("rough_cal.z.dwell", rough_params.z_dwell,
                                "Rough calibration point dwell time (Z scan)");
                def_param("rough_cal.z.avg_win", rough_params.z_avg_window,
                                "Averaging window for filtering of Z axis rough calibration data");

                def_param("fine_cal.range.xy", fine_params.xy_range,
                                "Amplitude of fine calibration perturbations (X and Y axes)");
                def_param("fine_cal.range.z", fine_params.z_range,
                                "Amplitude of fine calibration perturbations (Z axis)");
                def_param("fine_cal.points", fine_params.npts,
                                "Number of points in fine calibration scan");
                def_param("fine_cal.dwell", fine_params.dwell,
                                "Delay in usec between fine calibration points");
                def_param("fine_cal.compute_residuals", fine_params.compute_residuals,
                                "Compute residuals of calibration");

                def_param("feedback.delay", fb_params.delay,
                                "Delay between feedback loop iterations");
                def_param("feedback.max_delta", fb_params.max_delta,
                                "Maximum allowed position change during feedback");
                def_param("feedback.show_rate", fb_params.show_rate,
                                "Report on feedback loop iteration rate");
                def_param("feedback.setpoint.x", fb_params.setpoint.x(),
                                "X axis setpoint");
                def_param("feedback.setpoint.y", fb_params.setpoint.y(),
                                "Y axis setpoint");
                def_param("feedback.setpoint.z", fb_params.setpoint.z(),
                                "Z axis setpoint");
                def_param("feedback.min_singular_value", fb_params.min_singular_value,
                                "Minimum singular value of a reasonable regression");
                def_param("feedback.recal.delay", fb_params.recal_delay,
                                "Recalibration delay (in microseconds)");
                def_param("feedback.recal.weight", fb_params.recal_weight,
                                "Recalibration weight");
                def_param("feedback.recal.min_samples", fb_params.min_recal_samples,
                                "Minimum number of samples to perform recalibration");
                def_param("feedback.perturb.freq.x", fb_params.perturb_freqs.x(),
                                "X axis perturbation frequency");
                def_param("feedback.perturb.freq.y", fb_params.perturb_freqs.y(),
                                "Y axis perturbation frequency");
                def_param("feedback.perturb.freq.z", fb_params.perturb_freqs.z(),
                                "Z axis perturbation frequency");
                def_param("feedback.perturb.amp.x", fb_params.perturb_amp.x(),
                                "X axis perturbation frequency");
                def_param("feedback.perturb.amp.y", fb_params.perturb_amp.y(),
                                "Y axis perturbation frequency");
                def_param("feedback.perturb.amp.z", fb_params.perturb_amp.z(),
                                "Z axis perturbation frequency");

                add_pid_params(params, "pids.x", fb_params.pids[0]);
                add_pid_params(params, "pids.y", fb_params.pids[1]);
                add_pid_params(params, "pids.z", fb_params.pids[2]);
        }

        void feedback_ended() {
                std::cout << "$ FB-ERR\n";
                Vector3f pos = fb_stage.get_pos();
                fb_stage.stop();
                _stage.move(pos);
                _stage.start();
        }

        tracker_cli( input_channels<4>& psd_inputs
                   , Stage& _stage
                   , stage& fb_stage
                   ) : psd_inputs(psd_inputs)
                     , _stage(_stage), fb_stage(fb_stage)
                     , rough_params(def_rough_cal_params())
                     , fine_params(def_fine_cal_params())
                     , fb_params(def_feedback_params())
                     , fb(NULL)
                     , auto_xy_range_factor(0)
                     , scan_delay(100)
        {
                _stage.start();
                _stage.smooth_move({0.5, 0.5, 0.5}, 10000);
                usleep(10*1000);

                add_tracker_params();
                add_stage_params(params, _stage);

                rough_pos << 0.5, 0.5, 0.5;
                def_param("rough_pos.x", rough_pos.x(), "Rough calibration position (X axis)");
                def_param("rough_pos.y", rough_pos.y(), "Rough calibration position (Y axis)");
                def_param("rough_pos.z", rough_pos.z(), "Rough calibration position (Z axis)");

                def_param("auto_xy_range_factor", auto_xy_range_factor, "Fine calibration range as a fraction of rough calibration extrema separation");

                scan_center << 0.5, 0.5, 0.5;
                scan_range << 0.1, 0.1, 0.1;
                scan_points << 100, 100, 100;
                def_param("scan.center.x", scan_center.x(), "Center of manual scan (X axis)");
                def_param("scan.center.y", scan_center.y(), "Center of manual scan (Y axis)");
                def_param("scan.center.z", scan_center.z(), "Center of manual scan (Z axis)");
                def_param("scan.range.x", scan_range.x(), "Range of manual scan (X axis)");
                def_param("scan.range.y", scan_range.y(), "Range of manual scan (Y axis)");
                def_param("scan.range.z", scan_range.z(), "Range of manual scan (Z axis)");
                def_param("scan.points.x", scan_points.x(), "Number of points in manual scan (X axis)");
                def_param("scan.points.y", scan_points.y(), "Number of points in manual scan (Y axis)");
                def_param("scan.points.z", scan_points.z(), "Number of points in manual scan (Z axis)");
                def_param("scan.delay", scan_delay, "Delay between points in manual scan (ms)");

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
                                std::cout << "! ERR\tCommand failed during source\n";
                        }
                }
        }

        bool do_command(string cmdline)
        {
                std::istringstream ss(cmdline);
                string cmd;
                ss >> cmd;

                if (cmd == "source") {
                        string file;
                        ss >> file;
                        source(file);
                } else if (cmd == "set") {
                        string param, value;
                        ss >> param; ss >> value;
                        parameter* p = find_parameter(params, param);
                        if (!p)
                                std::cout << "! ERR\tUnknown parameter\n";
                        else {
                                try {
                                        *p = value;
                                } catch (std::exception e) {
                                        std::cout << "! ERR\tInvalid value\n";
                                }
                        }
                        std::cout << "! OK\n";
                } else if (cmd == "get") {
                        string param;
                        ss >> param;
                        parameter* p = find_parameter(params, param);
                        if (!p)
                                std::cout << "! ERR\tUnknown parameter\n";
                        else
                                std::cout << param << " = " << *p << "\n";
                        std::cout << "! OK\n";
                } else if (cmd == "list") {
                        string match = "";
                        if (!ss.eof())
                                ss >> match;
                        for (auto p=params.begin(); p != params.end(); p++)
                                if ((**p).name.compare(0, match.size(), match) == 0) {
                                        std::ostringstream tmp;
                                        tmp << **p;
                                        printf("%-30s\t%10s\t\t%50s\n",
                                                (**p).name.c_str(),
                                                tmp.str().c_str(),
                                                (**p).description.c_str());
                                }
                        std::cout << "! OK\n";
                } else if (cmd == "read-psd") {
                        Vector4f psd = psd_inputs.get();
                        std::cout << psd.transpose().format(mat_fmt) << "\n";
                        std::cout << "! OK\n";
                } else if (cmd == "read-pos") {
                        Vector3f fb = _stage.get_pos();
                        std::cout << fb.transpose().format(mat_fmt) << "\n";
                        std::cout << "! OK\n";
                } else if (cmd == "move") {
                        Vector3f pos;
                        ss >> pos.x();
                        ss >> pos.y();
                        ss >> pos.z();
                        _stage.smooth_move(pos, 10000);
                        std::cout << "! OK\n";
                } else if (cmd == "move-rough-pos") {
                        _stage.smooth_move(rough_pos, 10000);
                        std::cout << "! OK\n";
                } else if (cmd == "center") {
                        Vector3f pos = 0.5 * Vector3f::Ones();
                        _stage.smooth_move(pos, 10000);
                        std::cout << "! OK\n";
                } else if (cmd == "rough-cal") {
                        try {
                                rough_cal_result res = rough_calibrate(_stage, psd_inputs, rough_params, Vector3f(0.5, 0.5, 0.5));
                                rough_pos = res.center;
                                if (auto_xy_range_factor)
                                        fine_params.xy_range = res.xy_size * auto_xy_range_factor;
                                _stage.move(rough_pos);
                                std::cout << rough_pos.transpose().format(mat_fmt) << "\n";
                                std::cout << "! OK\n";
                        } catch (clamped_output_error e) {
                                std::cout << "! ERR\tClamped output\n";
                        }
                } else if (cmd == "fine-cal") {
                        fine_cal = fine_calibrate(_stage, psd_inputs, fine_params, rough_pos);
                        _stage.move(rough_pos);
                        std::cout << "! OK\n";
                } else if (cmd == "show-coeffs") {
                        std::cout << fine_cal.beta.format(mat_fmt) << "\n";
                        std::cout << "! OK\n";
                } else if (cmd == "save-coeffs") {
                        std::string s;
                        ss >> s;
                        try {
                                std::ofstream f(s);
                                for (int r=0; r<fine_cal.beta.rows(); r++) {
                                        for (int c=0; c<fine_cal.beta.cols(); c++) {
                                                if (c > 0) f << "\t";
                                                f << fine_cal.beta(r,c);
                                        }
                                        f << "\n";
                                }
                                std::cout << "! OK\n";
                        } catch (std::exception& e) {
                                std::cout << "! ERR\t" << e.what() << "\n";
                        }
                        std::cout << "! OK\n";
                } else if (cmd == "load-coeffs") {
                        std::string s;
                        ss >> s;
                        try {
                                std::ifstream f(s);
                                for (int r=0; r<fine_cal.beta.rows(); r++) {
                                        for (int c=0; c<fine_cal.beta.cols(); c++)
                                                ss >> fine_cal.beta(r,c);
                                }
                                std::cout << "! OK\n";
                        } catch (std::exception& e) {
                                std::cout << "! ERR\t" << e.what() << "\n";
                        }
                } else if (cmd == "feedback-start") {
                        if (fb && fb->running())
                                std::cout << "! ERR\tAlready running\n";
                        else {
                                _stage.stop();
                                fb_stage.move(fb_stage.out.get());
                                if (fb) delete fb;
                                fb = new feedback(psd_inputs, *stage_in, fb_stage, fb_params, fine_cal);
                                fb->feedback_ended_cb = [&](){ this->feedback_ended(); };
                                fb->start();
                                std::cout << "! OK\tFeedback running\n";
                        }
                } else if (cmd == "feedback-stop") {
                        if (fb == NULL || !fb->running())
                                std::cout << "ERR\tNot running\n";
                        else {
                                fb->stop();
                                delete fb;
                                fb = NULL;
                                std::cout << "OK\tFeedback stopped\n";
                                fb_stage.stop();
                                _stage.start();
                        }
                } else if (cmd == "wait") {
                        int a;
                        ss >> a;
                        usleep(a*1000);
                        std::cout << "! OK\n";
                } else if (cmd == "exit") {
                        return true;
                } else if (cmd == "quit") {
                        return true;
                } else if (cmd == "help") {
                        std::cout << cmd_help << "\n";
                        std::cout << "! OK\n";
                } else if (cmd == "version") {
                        std::cout << branch << "\t" << version << "\n";
                        std::cout << "! OK\n";
                } else if (cmd == "scan") {
                        std::string fname;
                        ss >> fname;
                        if (fname.length() == 0) fname = "scan";
                        Vector3f start = scan_center - scan_range / 2;
                        Vector3f step = scan_range.array() / scan_points.array().cast<float>();
                        raster_route rt(start, step, scan_points);
                        _stage.smooth_move(rt.get_pos(), 4000);
                        std::ofstream of(fname);
                        for (int i=0; rt.has_more(); ++i, ++rt) {
                                _stage.move(rt.get_pos());
                                Vector3f fb = _stage.get_pos();
                                Vector4f psd = psd_inputs.get();
                                of << fb[0] << "  " << fb[1] << "  " << fb[2] << "\t"
                                   << psd[0] << "  " << psd[1] << "  " << psd[2] << "  " << psd[3] << "\n";
                                usleep(scan_delay);
                        }
                        std::cout << "! OK\n";
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
                        } catch (invalid_syntax e) {
                                std::cout << "! ERR\tInvalid syntax\n";
                        } catch (std::exception& e) {
                                std::cout << "! ERR\tCommand failed: " << e.what() << "\n";
                        }
                }
        }
};

int main(int argc, char** argv)
{
        init_hardware();
        //fb_stage stage(*stage_out, *stage_in);
        stage raw_stage(*stage_out);
        pid_stage stage(*stage_out, *stage_in);
        tracker_cli<pid_stage> cli(*psd_in, stage, raw_stage);
        cli.mainloop();

        return 0;
}

