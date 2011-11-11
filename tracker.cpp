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

#include "utils.h"
#include "tracker.h"

#include <time.h>
#include <utility>
#include <array>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <tr1/random>
#include <Eigen/SVD>

using std::string;
using std::vector;
using std::array;
using Eigen::Dynamic;

Vector4f tracker::scale_psd_position(Vector4f in)
{
        if (scale_psd_inputs) {
                float sum = in[3] - in[2];
                in /= sum;
        }
        return in;
}

template<typename Matrix>
static unsigned int max_row(Matrix a) {
        int row, col;
        a.maxCoeff(&row, &col);
        return row;
}

template<typename Matrix>
static unsigned int min_row(Matrix a) {
        int row, col;
        a.minCoeff(&row, &col);
        return row;
}

tracker::rough_cal_xy_result tracker::rough_calibrate_xy(Vector3f center)
{
        Vector3f tmp;
        Vector3f start, step;
        Vector3u pts;

        tmp << rough_cal_xy_range, rough_cal_xy_range, 0;
        start = center - tmp/2;
        step << rough_cal_xy_range / rough_cal_xy_pts, rough_cal_xy_range / rough_cal_xy_pts, 0;
        pts << rough_cal_xy_pts, rough_cal_xy_pts, 1;

        raster_route rt(start, step, pts);
        Matrix<float, Dynamic, 3> pos_data(rough_cal_xy_pts*rough_cal_xy_pts, 3);
        Matrix<float, Dynamic, 4> psd_data(rough_cal_xy_pts*rough_cal_xy_pts, 4);
        Matrix<float, Dynamic, 3> fb_data(rough_cal_xy_pts*rough_cal_xy_pts, 3);
        
        stage_outputs.move(start);
        nsleep(1000*1000);

        // Run X/Y scan and preprocess data
        for (int i=0; rt.has_more(); ++i, ++rt) {
                Vector3f pos = rt.get_pos();
                stage_outputs.move(pos);
                usleep(rough_cal_xy_dwell);
                pos_data.row(i) = pos;
                psd_data.row(i) = scale_psd_position(psd_inputs.get());
                fb_data.row(i) = stage_outputs.get_pos();
        }

        dump_matrix((MatrixXf(rough_cal_xy_pts*rough_cal_xy_pts,10) << pos_data, fb_data, psd_data).finished(), "rough");

        // Find extrema of Vx, Vy
        rough_cal_xy_result res;
        res.xmin = fb_data.row(min_row(psd_data.col(0)));
        res.xmax = fb_data.row(max_row(psd_data.col(0)));
        res.ymin = fb_data.row(min_row(psd_data.col(1)));
        res.ymax = fb_data.row(max_row(psd_data.col(1)));
        return res;
}


Vector3f tracker::rough_calibrate_z(Vector3f center)
{
        Vector3f laser_pos = center;
        laser_pos.z() -= rough_cal_z_range / 2;
        stage_outputs.move(laser_pos);
        nsleep(1000*1000);

        Vector3f step; 
        Vector3u pts;
        step << 0, 0, rough_cal_z_range / rough_cal_z_pts;
        pts << 1, 1, rough_cal_z_pts;

        raster_route rt(laser_pos, step, pts);
        Matrix<float, Dynamic, 3> pos_data(rough_cal_z_pts, 3);
        Matrix<float, Dynamic, 4> psd_data(rough_cal_z_pts, 4);
        Matrix<float, Dynamic, 3> fb_data(rough_cal_z_pts, 3);

        // Run Z scan and preprocess data
        for (int i=0; rt.has_more(); ++i, ++rt) {
                Vector3f pos = rt.get_pos();
                stage_outputs.move(pos);
                usleep(rough_cal_z_dwell);
                pos_data.row(i) = pos;
                psd_data.row(i) = scale_psd_position(psd_inputs.get());
                fb_data.row(i) = stage_outputs.get_pos();
        }

        // Preprocess Z data with moving average
        if (rough_cal_z_avg_win) {
                for (unsigned int i = rough_cal_z_avg_win; i < rough_cal_z_pts - rough_cal_z_avg_win; i++) {
                        Vector4f mean = Vector4f::Zero();
                        for (int j = -rough_cal_z_avg_win; j < (int) rough_cal_z_avg_win; j++)
                                mean += psd_data.row(i+j);
                        mean /= 2.0*rough_cal_z_avg_win;
                        psd_data.row(i) = mean;
                }
        }

        // Find extrema of dVz/dz
        float max_deriv = 0;
        for (unsigned int i = rough_cal_z_avg_win + 1; i < rough_cal_z_pts - rough_cal_z_avg_win - 1; i++) {
                float sum1 = psd_data(i+1,2) - psd_data(i+1,3);
                float z1 = fb_data(i+1,2);
                float sum2 = psd_data(i-1,2) - psd_data(i-1,3);
                float z2 = fb_data(i-1,2);
                float deriv = (sum1 - sum2) / (z1 - z2);
                if (deriv > max_deriv) {
                        max_deriv = deriv;
                        laser_pos.z() = fb_data(i,2);
                }
        }
        dump_matrix((MatrixXf(rough_cal_z_pts,10) << pos_data, fb_data, psd_data).finished(), "rough_z");
        return laser_pos;
}

tracker::rough_cal_result tracker::rough_calibrate(Vector3f center)
{
        rough_cal_xy_result res_xy = rough_calibrate_xy(center);
        Vector3f laser_pos;

        float dist = (res_xy.xmin - res_xy.ymin).norm();
        std::cout << "Extrema distance: " << dist << "\n";
        laser_pos.x() = (res_xy.xmax.x() - res_xy.xmin.x())/2 + res_xy.xmin.x();
        laser_pos.y() = (res_xy.ymax.y() - res_xy.ymin.y())/2 + res_xy.ymin.y();
        laser_pos.z() = center.z();

        Vector3f z = rough_calibrate_z(laser_pos);
        laser_pos.z() = z.z();

        rough_cal_result res;
        res.center = laser_pos;
        res.xy_size = dist;
        res.z_size = 0.1;
        return res;
}

/*
 * pack_psd_inputs(): Pack input data into vector with higher order terms
 */
static Matrix<float,Dynamic,9> pack_psd_inputs(Matrix<float,Dynamic,4> data) {
        Matrix<float,Dynamic,9> R(data.rows(), 9);

        // First order
        R.col(0) = data.col(0);                         // Vx
        R.col(1) = data.col(1);                         // Vy
        R.col(2) = -data.col(2) + data.col(3);          // Vsum = -Vsum_x + Vsum_y
        
        // Second order
        R.col(3) = R.col(0).array().square();           // Vx^2
        R.col(4) = R.col(1).array().square();           // Vy^2
        R.col(5) = R.col(2).array().square();           // Vsum^2

        // Cross terms
        R.col(6) = R.col(0).array() * R.col(1).array(); // Vx*Vy
        R.col(7) = R.col(0).array() * R.col(2).array(); // Vx*Vsum
        R.col(8) = R.col(1).array() * R.col(2).array(); // Vy*Vsum

        return R;
}

tracker::fine_cal_result tracker::fine_calibrate(Vector3f rough_pos)
{
        bool dump_matricies = true;
        Vector3f range(fine_cal_xy_range, fine_cal_xy_range, fine_cal_z_range);
        Matrix<float, Dynamic, 4> psd_data(fine_cal_pts,4);
        Matrix<float, Dynamic, 3> fb_data(fine_cal_pts,3);
        fine_cal_result res;
        std::default_random_engine eng;
        std::uniform_real_distribution<float> rng;

        // Setup stage
        stage_outputs.move(rough_pos);

        // Collect data
        for (unsigned int i=0; i < fine_cal_pts; i++) {
                Vector3f pos = Vector3f(rng(eng), rng(eng), rng(eng)).cwiseProduct(range);
                pos += rough_pos;
                stage_outputs.move(pos);
                usleep(fine_cal_dwell);
                psd_data.row(i) = scale_psd_position(psd_inputs.get());
                fb_data.row(i) = stage_outputs.get_pos();
        }

        // Find and subtract out PSD mean
        res.psd_mean = psd_data.colwise().mean();
        psd_data.rowwise() -= res.psd_mean;
        dump_matrix((MatrixXf(fine_cal_pts,7) << fb_data, psd_data).finished(), "fine");

        // Fill R and S matricies with collected data
        Matrix<double, Dynamic,9> R = pack_psd_inputs(psd_data).cast<double>();
        Matrix<double, Dynamic,3> S = (fb_data.rowwise() - rough_pos.transpose()).cast<double>();

        // Solve regression coefficients
        JacobiSVD<Matrix<double, Dynamic,9> > svd(R);
        Matrix<double, 9,3> bt = svd.solve(S);
        res.singular_values = svd.singularValues();
        std::cout << "Singular values: " << svd.singularValues() << "\n";
        res.beta = bt.transpose().cast<float>();

        bool compute_residuals = true;
        if (compute_residuals) {
                FILE* of = fopen("fine-resid", "w");
                fprintf(of, "# fb_x fb_y fb_z\tP_Lx P_Ly P_Lz\tresid_x resid_y resid_z\n");
                Vector3d rms = Vector3d::Zero();
                for (unsigned int i=0; i < fine_cal_pts; i++) {
                        Vector3f fb = fb_data.row(i);
                        Vector3d s = S.row(i).transpose().cast<double>();
                        Vector3d resid = bt.transpose() * R.row(i).transpose() - s;
                        fprintf(of, "%f %f %f\t%f %f %f\t%f %f %f\n",
                                fb.x(), fb.y(), fb.z(),
                                s.x(), s.y(), s.z(),
                                resid.x(), resid.y(), resid.z());
                        rms += resid.cwiseProduct(resid);
                }
                rms = (rms / fine_cal_pts).cwiseSqrt();
                fprintf(stderr, "RMS Residuals: %f %f %f\n", rms.x(), rms.y(), rms.z());
                fclose(of);
        }

        if (dump_matricies) {
                dump_matrix(R, "R");
                dump_matrix(S, "S");
                dump_matrix(bt, "beta");
        }
        return res;
}

void tracker::feedback(fine_cal_result cal)
{
        unsigned int n = 0;
        unsigned int bad_pts = 0, good_pts = 0;
        std::ofstream f("pos");
        struct timespec start_time;
        clock_gettime(CLOCK_REALTIME, &start_time);
        float last_report_t = 0;
        unsigned int last_report_n = 0;

        _running = true;
        while (!stop) {
                // Make sure recent points are generally sane
                if (good_pts > 10)
                        good_pts = bad_pts = 0;
                if (bad_pts > 10) {
                        fprintf(stderr, "Lost tracking\n");
                        break;
                }

                // Get sensor values
                Vector4f psd = psd_inputs.get();
                n++;

                // Compute estimated position
                psd = scale_psd_position(psd) - cal.psd_mean;
                Matrix<float, Dynamic,9> psd_in = pack_psd_inputs(psd.transpose());
                Vector3f delta = cal.beta * psd_in.transpose();

                // Get PID response
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                float t = (ts.tv_sec - start_time.tv_sec) +
                        (ts.tv_nsec - start_time.tv_nsec)*1e-9;
                for (int i=0; i<3; i++) {
                        fb_pids[i].add_point(t, delta[i]);
                        delta[i] = fb_pids[i].get_response();
                }

                // Check sanity of point
                if (delta.norm() > fb_max_delta) {
                        fprintf(stderr, "Error: Delta exceeded maximum, likely lost tracking\n");
                        bad_pts++;
                        continue;
                }

                // Move stage
                Vector3f new_pos = delta + fb_setpoint;
                //new_pos.z() = 0.5 + fb_setpoint.z();
                f << delta.x() << delta.y() << delta.z() <<
                     new_pos.x() << new_pos.y() << new_pos.z();
                try {
                        stage_outputs.move_rel(delta + fb_setpoint);
                } catch (clamped_output_error e) {
                        bad_pts++;
                        fprintf(stderr, "Clamped\n");
                        continue;
                }
                good_pts++;

                // Show feedback rate report
                if (fb_show_rate && t > (last_report_t + fb_rate_report_period)) {
                        float rate = (n - last_report_n) / (t - last_report_t);
                        fprintf(stderr, "Feedback loop rate: %f updates/sec\n", rate);
                        last_report_t = t;
                        last_report_n = n;
                }
                usleep(fb_delay);
        }

        _running = false;
        if (feedback_ended_cb)
                feedback_ended_cb();
}

void tracker::start_feedback(fine_cal_result cal)
{
        stop = false;
        feedback_thread = std::thread(&tracker::feedback, this, cal);
}

bool tracker::running()
{
        return _running;
}

void tracker::stop_feedback()
{
        stop = true;
        feedback_thread.join();
}

tracker::~tracker() {
        stop_feedback();
}

