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

Vector4f scale_psd_position(Vector4f in)
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

struct rough_cal_xy_result {
	Vector3f xmin, ymin, xmax, ymax;
};

rough_cal_xy_result rough_calibrate_xy( stage& _stage
                                      , input_channels<4>& psd
                                      , rough_cal_params& params
                                      , Vector3f center)
{
        Vector3f tmp;
        Vector3f start, step;
        Vector3u pts;

        tmp << params.xy_range
             , params.xy_range
             , 0;
        start = center - tmp/2;
        step << params.xy_range / params.xy_npts
              , params.xy_range / params.xy_npts
              , 0;
        pts << params.xy_npts
             , params.xy_npts
             , 1;

        raster_route rt(start, step, pts);
        Matrix<float, Dynamic, 3> pos_data(params.xy_npts*params.xy_npts, 3);
        Matrix<float, Dynamic, 4> psd_data(params.xy_npts*params.xy_npts, 4);
        Matrix<float, Dynamic, 3> fb_data(params.xy_npts*params.xy_npts, 3);
        
        _stage.move(start);
        nsleep(1000*1000);

        // Run X/Y scan and preprocess data
        for (int i=0; rt.has_more(); ++i, ++rt) {
                Vector3f pos = rt.get_pos();
                _stage.move(pos);
                nsleep(1000*params.xy_dwell);
                pos_data.row(i) = pos;
                fb_data.row(i) = _stage.get_pos();
                psd_data.row(i) = scale_psd_position(psd.get(false));
        }

        dump_matrix((MatrixXf(params.xy_npts*params.xy_npts,10) << pos_data, fb_data, psd_data).finished(), "rough");

        // Find extrema of Vx, Vy
        rough_cal_xy_result res;
        res.xmin = fb_data.row(min_row(psd_data.col(0)));
        res.xmax = fb_data.row(max_row(psd_data.col(0)));
        res.ymin = fb_data.row(min_row(psd_data.col(1)));
        res.ymax = fb_data.row(max_row(psd_data.col(1)));
        return res;
}

Vector3f rough_calibrate_z( stage& _stage
                          , input_channels<4>& psd
                          , rough_cal_params& params
                          , Vector3f center)
{
        Vector3f laser_pos = center;
        laser_pos.z() -= params.z_range / 2;
        _stage.move(laser_pos);
        nsleep(1000*1000);

        Vector3f step; 
        Vector3u pts;
        step << 0, 0, params.z_range / params.z_npts;
        pts << 1, 1, params.z_npts;

        raster_route rt(laser_pos, step, pts);
        Matrix<float, Dynamic, 3> pos_data(params.z_npts, 3);
        Matrix<float, Dynamic, 4> psd_data(params.z_npts, 4);
        Matrix<float, Dynamic, 3> fb_data(params.z_npts, 3);

        // Run Z scan and preprocess data
        for (int i=0; rt.has_more(); ++i, ++rt) {
                Vector3f pos = rt.get_pos();
                _stage.move(pos);
                nsleep(1000*params.z_dwell);
                pos_data.row(i) = pos;
                fb_data.row(i) = _stage.get_pos();
                psd_data.row(i) = scale_psd_position(psd.get(false));
        }

        // Preprocess Z data with moving average
        if (params.z_avg_window) {
                for (unsigned int i = params.z_avg_window; i < params.z_npts - params.z_avg_window; i++) {
                        Vector4f mean = Vector4f::Zero();
                        for (int j = -params.z_avg_window; j < (int) +params.z_avg_window; j++)
                                mean += psd_data.row(i+j);
                        mean /= 2.0*params.z_avg_window;
                        psd_data.row(i) = mean;
                }

                pos_data = pos_data.bottomRows(params.z_npts - params.z_avg_window)
                                   .topRows(params.z_npts - 2*params.z_avg_window);
                psd_data = psd_data.bottomRows(params.z_npts - params.z_avg_window)
                                   .topRows(params.z_npts - 2*params.z_avg_window);
                fb_data = fb_data.bottomRows(params.z_npts - params.z_avg_window)
                                 .topRows(params.z_npts - 2*params.z_avg_window);
        }

//#define ROUGH_Z_DERIV
#ifdef ROUGH_Z_DERIV
        // Find extrema of dVz/dz
        float max_deriv = 0;
        for (unsigned int i = 0; i < psd_data.rows(); i++) {
                float sum1 = psd_data(i+1,2) - psd_data(i+1,3);
                float z1 = fb_data(i+1,2);
                float sum2 = psd_data(i-1,2) - psd_data(i-1,3);
                float z2 = fb_data(i-1,2);
                float deriv = (sum1 - sum2) / (z1 - z2);
                if (fabs(deriv) > fabs(max_deriv)) {
                        max_deriv = deriv;
                        laser_pos.z() = fb_data(i,2);
                }
        }
#else
        int min_idx, max_idx;
        psd_data.col(2).minCoeff(&min_idx);
        psd_data.col(2).maxCoeff(&max_idx);
        int center_idx = (max_idx - min_idx) / 2 + min_idx;
        laser_pos.z() = fb_data(center_idx,2);
#endif

        dump_matrix((MatrixXf(pos_data.rows(),10) << pos_data, fb_data, psd_data).finished(), "rough_z");
        return laser_pos;
}

rough_cal_result rough_calibrate( stage& _stage
                                , input_channels<4>& psd
				, rough_cal_params& params
		                , Vector3f center)
{
        rough_cal_xy_result res_xy = rough_calibrate_xy(_stage, psd, params, center);
        Vector3f laser_pos;

        float dist = (res_xy.xmin - res_xy.ymin).norm();
        std::cout << "Extrema distance: " << dist << "\n";
        laser_pos.x() = (res_xy.xmax.x() - res_xy.xmin.x())/2 + res_xy.xmin.x();
        laser_pos.y() = (res_xy.ymax.y() - res_xy.ymin.y())/2 + res_xy.ymin.y();
        laser_pos.z() = center.z();

        Vector3f z = rough_calibrate_z(_stage, psd, params, laser_pos);
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

fine_cal_result fine_calibrate( stage& stage
                              , input_channels<4>& psd
		              , fine_cal_params& params
			      , Vector3f rough_pos)
{
        bool dump_matrices = true;
        Vector3f range(params.xy_range, params.xy_range, params.z_range);
        Matrix<float, Dynamic, 4> psd_data(params.npts,4);
        Matrix<float, Dynamic, 3> fb_data(params.npts,3);
        fine_cal_result res;
        std::default_random_engine eng;
        std::uniform_real_distribution<float> rng;

        // Setup stage
        stage.move(rough_pos);

        // Collect data
        for (unsigned int i=0; i < params.npts; i++) {
                Vector3f pos;
                pos = Vector3f(rng(eng), rng(eng), rng(eng)).cwiseProduct(range/2);
                pos += range/2;
                pos += rough_pos;
                stage.move(pos);
                nsleep(1000*params.dwell);
                fb_data.row(i) = stage.get_pos();
                psd_data.row(i) = scale_psd_position(psd.get(false));
        }

        // Find and subtract out PSD mean
        res.psd_mean = psd_data.colwise().mean();
        psd_data.rowwise() -= res.psd_mean;
        dump_matrix((MatrixXf(params.npts,7) << fb_data, psd_data).finished(), "fine");

        // Fill R and S matricies with collected data
        Matrix<double, Dynamic,9> R = pack_psd_inputs(psd_data).cast<double>();
        Matrix<double, Dynamic,3> S = (fb_data.rowwise() - rough_pos.transpose()).cast<double>();

        // Solve regression coefficients
        JacobiSVD<Matrix<double, Dynamic,9> > svd = R.jacobiSvd(ComputeFullU | ComputeFullV);
        Matrix<double, 9,3> bt = svd.solve(S);
        res.singular_values = svd.singularValues();
        std::cout << "Singular values: " << svd.singularValues() << "\n";
        res.beta = bt.transpose().cast<float>();

        bool compute_residuals = true;
        if (compute_residuals) {
                FILE* of = fopen("fine-resid", "w");
                fprintf(of, "# fb_x fb_y fb_z\tP_Lx P_Ly P_Lz\tresid_x resid_y resid_z\n");
                Vector3d rms = Vector3d::Zero();
                for (unsigned int i=0; i < params.npts; i++) {
                        Vector3f fb = fb_data.row(i);
                        Vector3d s = S.row(i).transpose().cast<double>();
                        Vector3d resid = bt.transpose() * R.row(i).transpose() - s;
                        fprintf(of, "%f %f %f\t%f %f %f\t%f %f %f\n",
                                fb.x(), fb.y(), fb.z(),
                                s.x(), s.y(), s.z(),
                                resid.x(), resid.y(), resid.z());
                        rms += resid.cwiseProduct(resid);
                }
                rms = (rms / params.npts).cwiseSqrt();
                fprintf(stderr, "RMS Residuals: %f %f %f\n", rms.x(), rms.y(), rms.z());
                fclose(of);
        }

        if (dump_matrices) {
                dump_matrix(R, "R");
                dump_matrix(S, "S");
                dump_matrix(bt, "beta");
        }
        return res;
}

void feedback::loop()
{
        unsigned int n = 0;
        unsigned int bad_pts = 0, good_pts = 0;
        FILE* f = fopen("pos", "w");
        struct timespec start_time;
        clock_gettime(CLOCK_REALTIME, &start_time);
        float last_report_t = 0;
        unsigned int last_report_n = 0;

        _running = true;
        for (int i=0; i<3; i++)
                params.pids[i].clear();

        while (!_stop) {
                // Make sure recent points are generally sane
                if (good_pts > 10)
                        good_pts = bad_pts = 0;
                if (bad_pts > 10) {
                        fprintf(stderr, "Lost tracking\n");
                        break;
                }

                // Get sensor values
                Vector4f psd_sample = psd.get();
                n++;

                // Compute estimated position
                psd_sample = scale_psd_position(psd_sample) - cal.psd_mean;
                Matrix<float, Dynamic,9> psd_in = pack_psd_inputs(psd_sample.transpose());
                Vector3f delta = cal.beta * psd_in.transpose();

                // Get PID response
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                float t = (ts.tv_sec - start_time.tv_sec) +
                        (ts.tv_nsec - start_time.tv_nsec)*1e-9;
                for (int i=0; i<3; i++) {
                        params.pids[i].add_point(t, delta[i]);
                        delta[i] = params.pids[i].get_response();
                }

                fprintf(f, "%f  %f  %f  %f      %f  %f  %f     %f  %f  %f\n",
                        psd_sample[0], psd_sample[1], psd_sample[2], psd_sample[3],
                        delta.x(), delta.y(), delta.z(),
                        _stage.get_target_pos().x(), _stage.get_target_pos().y(), _stage.get_target_pos().z());

                // Check sanity of point
                if (delta.norm() > params.max_delta) {
                        fprintf(stderr, "Error: Delta exceeded maximum, likely lost tracking\n");
                        bad_pts++;
                        continue;
                }

                // Move stage
                try {
                        _stage.move_rel(delta);
                } catch (clamped_output_error e) {
                        bad_pts++;
                        fprintf(stderr, "Clamped\n");
                        continue;
                }
                good_pts++;

                // Show feedback rate report
                if (params.show_rate && t > (last_report_t + params.rate_report_period)) {
                        float rate = (n - last_report_n) / (t - last_report_t);
                        fprintf(stderr, "Feedback loop rate: %f updates/sec\n", rate);
                        last_report_t = t;
                        last_report_n = n;
                }
                nsleep(1000*params.delay);
        }

        fclose(f);
        _running = false;
        if (feedback_ended_cb)
                feedback_ended_cb();
}

void feedback::start()
{
        _stop = false;
        worker = std::thread(&feedback::loop, this);
}

bool feedback::running()
{
        return _running;
}

void feedback::stop()
{
        _stop = true;
        worker.join();
}

feedback::~feedback() {
        stop();
}

