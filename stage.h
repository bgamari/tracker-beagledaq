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

#include <vector>
#include <tr1/random>
#include <thread>
#include <Eigen/Eigen>

using std::vector;

typedef Matrix<unsigned int,3,1> Vector3u;

class stage {
protected:
        Vector3f target_pos;
public:
        const output_channels<3>& out;
        stage(const output_channels<3>& out) :
                target_pos(0.5*Vector3f::Ones()), out(out) { }
        virtual void move(const Vector3f pos);
        virtual void move_rel(const Vector3f delta);
        void smooth_move(Vector3f to, unsigned int move_time);
        virtual Vector3f get_target_pos() const;
        virtual Vector3f get_pos() const;
};

/*
 * fb_stage: Stage backend with calibration to feedback sensor
 *
 * This stage backend specifies positions relative to a feedback sensor.
 * The output position values are computed from a calibration between
 * the feedback sensor and the output values.
 *
 */
class fb_stage : public stage {
        Matrix<float, 7,3> R;
public:
        const input_channels<3>& fb;
        float cal_range;

        fb_stage(const output_channels<3>& out, const input_channels<3>& fb, float cal_range=0.4)
                : stage(out), fb(fb), cal_range(cal_range) {
                calibrate();
        }

        /*
         * calibrate():
         * Perform basic first-order OLS regression to map feedback coordinate
         * space to stage input space
         */
        void calibrate(unsigned int n_pts=400, unsigned int n_samp=10);
        void move(const Vector3f pos);
        Vector3f get_pos() const;
};

/*
 * pid_stage: Stage backend with PID feedback
 *
 * This stage backend implements a PID loop feeding back from a
 * feedback sensor
 */
class pid_stage : public stage {
        Vector3f pos; // current output value
        const input_channels<3>& fb;
public:
        pid_loop pidx, pidy, pidz;
        unsigned int fb_delay;
private:
        std::thread fb_worker;
        void worker();
        bool stop;

public:
        pid_stage(const output_channels<3>& out, const input_channels<3>& fb) :
                stage(out), pos(0.5*Vector3f::Ones()), fb(fb),
                pidx(0.1), pidy(0.1), pidz(0.1),
                fb_delay(6000), fb_worker(&pid_stage::worker, this),
                stop(false) { }
        ~pid_stage();

        void move(const Vector3f pos);
        Vector3f get_pos() const;
};

/*
 * route: Represents a path of points through 3-space
 */
struct route {
        virtual Vector3f get_pos() = 0;
        virtual void operator++() = 0;
        virtual bool has_more() = 0;
};

struct random_route : route {
        std::tr1::mt19937 eng;
        std::tr1::uniform_real<float> rng;
        std::array<float,3> ranges;
        unsigned int n_pts;
        Vector3f a;

        random_route(std::array<float,3> ranges, unsigned int n_pts) : 
                rng(0, 1), ranges(ranges), n_pts(n_pts) { }

        void operator++();
        Vector3f get_pos();
        bool has_more();
};

struct raster_route : route {
        const Vector3f start;
        const Vector3f step;
        const Vector3u points;

        unsigned int n;

        raster_route(Vector3f start, Vector3f step, Vector3u points) :
                start(start), step(step), points(points), n(0) {  }

        Vector3f get_pos();
        void operator++();
        bool has_more();
};

